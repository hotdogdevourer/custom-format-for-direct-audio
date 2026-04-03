"""
CFDA (Custom Format for Direct Audio) Audio Format (.cfda) - Reader, Writer & Player
Format Specification:
  Offset | Size | Name          | Description
  0      | 16   | Header        | "CFDA" + 0xCFDAy8746CDF2A6DEADBEEFFE
  16     | 2    | NumChannels   | 1=mono, 2=stereo, etc.
  18     | 4    | SampleRate    | Samples per second
  22     | 4    | ByteRate      | SampleRate * NumChannels * BitsPerSample / 8
  26     | 2    | BitsPerSample | Bits per sample (1-32)
  28     | 4    | FileSize      | Total file size - 32
  33+    | N    | PCM Data      | Raw, bit-packed uncompressed PCM data

Bit layout - all depths use MSB-first group packing:

  bps  samples/group  bytes/group  byte layout
  ---  -------------  -----------  --------------------------------------------
    1       8              1       np.packbits (MSB first)
    2       4              1       [s0[1:0]<<6|s1[1:0]<<4|s2[1:0]<<2|s3[1:0]]
    4       2              1       [s0[3:0]<<4|s1[3:0]]
    6       4              3       s0<<2|s1>>4 / (s1&F)<<4|s2>>2 / (s2&3)<<6|s3
    8       1              1       plain uint8
   10       4              5       s0>>2 / (s0&3)<<6|s1>>4 / ...
   12       2              3       s0>>4 / (s0&F)<<4|s1>>8 / s1&FF
   14       4              7       s0>>6 / (s0&3F)<<2|s1>>12 / ...
   16       1              2       little-endian signed int16
   18       4              9       s0>>10 / s0>>2&FF / (s0&3)<<6|s1>>12 / ...
   20       2              5       s0>>12 / s0>>4&FF / (s0&F)<<4|s1>>16 / ...
   22       4             11       s0>>14 / s0>>6&FF / (s0&3F)<<2|s1>>20 / ...
   24       1              3       big-endian 3-byte unsigned
   26       4             13       s0>>18 / s0>>10&FF / s0>>2&FF / (s0&3)<<6|s1>>20 / ...
   28       2              7       s0>>20 / s0>>12&FF / s0>>4&FF / (s0&F)<<4|s1>>24 / ...
   30       4             15       s0>>22 / s0>>14&FF / s0>>6&FF / (s0&3F)<<2|s1>>28 / ...
   32       1              4       big-endian 4-byte unsigned
"""

import struct
import numpy as np
import sounddevice as sd
import argparse
import sys
import wave
import os

CFDA_MAGIC = b'CFDA' + bytes.fromhex('CFDA8746CDF2A6DEADBEEFFE')
HEADER_SIZE = 32
METADATA_STRUCT = '<HIIHI'


class CFDAAudio:

    @staticmethod
    def _pack_pcm(flat_float: np.ndarray, bps: int) -> bytes:
        levels = 1 << bps
        max_val = levels - 1
        
        idx = np.floor(((flat_float + 1.0) * 0.5) * levels).astype(np.uint64)
        idx = np.clip(idx, 0, max_val)
        n = len(idx)

        if bps == 1:
            idx = (flat_float >= 0).astype(np.uint8)
            return np.packbits(idx).tobytes()

        if bps == 2:
            pad = (-n) % 4
            p = np.zeros(n + pad, dtype=np.uint64); p[:n] = idx
            s = p.reshape(-1, 4)
            out = (((s[:,0]&3)<<6) | ((s[:,1]&3)<<4) |
                ((s[:,2]&3)<<2) |  (s[:,3]&3)).astype(np.uint8)
            return out.tobytes()

        if bps == 4:
            pad = n % 2
            p = np.zeros(n + pad, dtype=np.uint64); p[:n] = idx
            s = p.reshape(-1, 2)
            out = (((s[:,0]&0xF)<<4) | (s[:,1]&0xF)).astype(np.uint8)
            return out.tobytes()

        if bps == 6:
            pad = (-n) % 4
            p = np.zeros(n + pad, dtype=np.uint64); p[:n] = idx
            s = p.reshape(-1, 4)
            s0,s1,s2,s3 = s[:,0]&0x3F, s[:,1]&0x3F, s[:,2]&0x3F, s[:,3]&0x3F
            b0 = ((s0<<2) | (s1>>4)).astype(np.uint8)
            b1 = (((s1&0xF)<<4) | (s2>>2)).astype(np.uint8)
            b2 = (((s2&0x3) <<6) |  s3   ).astype(np.uint8)
            return np.stack([b0,b1,b2], axis=1).tobytes()

        if bps == 8:
            return (idx & 0xFF).astype(np.uint8).tobytes()

        if bps == 10:
            pad = (-n) % 4
            p = np.zeros(n + pad, dtype=np.uint64); p[:n] = idx
            s = p.reshape(-1, 4)
            s0,s1,s2,s3 = s[:,0]&0x3FF, s[:,1]&0x3FF, s[:,2]&0x3FF, s[:,3]&0x3FF
            b0 = (s0>>2).astype(np.uint8)
            b1 = (((s0&0x03)<<6) | (s1>>4)).astype(np.uint8)
            b2 = (((s1&0x0F)<<4) | (s2>>6)).astype(np.uint8)
            b3 = (((s2&0x3F)<<2) | (s3>>8)).astype(np.uint8)
            b4 = (s3&0xFF).astype(np.uint8)
            return np.stack([b0,b1,b2,b3,b4], axis=1).tobytes()

        if bps == 12:
            pad = n % 2
            p = np.zeros(n + pad, dtype=np.uint64); p[:n] = idx
            s = p.reshape(-1, 2)
            s0,s1 = s[:,0]&0xFFF, s[:,1]&0xFFF
            b0 = (s0>>4).astype(np.uint8)
            b1 = (((s0&0xF)<<4) | (s1>>8)).astype(np.uint8)
            b2 = (s1&0xFF).astype(np.uint8)
            return np.stack([b0,b1,b2], axis=1).tobytes()

        if bps == 14:
            pad = (-n) % 4
            p = np.zeros(n + pad, dtype=np.uint64); p[:n] = idx
            s = p.reshape(-1, 4)
            s0,s1,s2,s3 = s[:,0]&0x3FFF, s[:,1]&0x3FFF, s[:,2]&0x3FFF, s[:,3]&0x3FFF
            b0 = (s0>>6).astype(np.uint8)
            b1 = (((s0&0x3F)<<2) | (s1>>12)).astype(np.uint8)
            b2 = ((s1>>4)&0xFF).astype(np.uint8)
            b3 = (((s1&0xF)<<4) | (s2>>10)).astype(np.uint8)
            b4 = ((s2>>2)&0xFF).astype(np.uint8)
            b5 = (((s2&0x3) <<6) | (s3>>8)).astype(np.uint8)
            b6 = (s3&0xFF).astype(np.uint8)
            return np.stack([b0,b1,b2,b3,b4,b5,b6], axis=1).tobytes()

        if bps == 16:
            signed = (idx.astype(np.int32) - 32768).astype(np.int16)
            return signed.tobytes()

        if bps == 18:
            pad = (-n) % 4
            p = np.zeros(n + pad, dtype=np.uint64); p[:n] = idx
            s = p.reshape(-1, 4)
            s0,s1,s2,s3 = s[:,0]&0x3FFFF, s[:,1]&0x3FFFF, s[:,2]&0x3FFFF, s[:,3]&0x3FFFF
            b0 = (s0>>10).astype(np.uint8)
            b1 = ((s0>>2)&0xFF).astype(np.uint8)
            b2 = (((s0&0x3) <<6) | (s1>>12)).astype(np.uint8)
            b3 = ((s1>>4)&0xFF).astype(np.uint8)
            b4 = (((s1&0xF) <<4) | (s2>>14)).astype(np.uint8)
            b5 = ((s2>>6)&0xFF).astype(np.uint8)
            b6 = (((s2&0x3F)<<2) | (s3>>16)).astype(np.uint8)
            b7 = ((s3>>8)&0xFF).astype(np.uint8)
            b8 = (s3&0xFF).astype(np.uint8)
            return np.stack([b0,b1,b2,b3,b4,b5,b6,b7,b8], axis=1).tobytes()

        if bps == 20:
            pad = n % 2
            p = np.zeros(n + pad, dtype=np.uint64); p[:n] = idx
            s = p.reshape(-1, 2)
            s0,s1 = s[:,0]&0xFFFFF, s[:,1]&0xFFFFF
            b0 = (s0>>12).astype(np.uint8)
            b1 = ((s0>>4)&0xFF).astype(np.uint8)
            b2 = (((s0&0xF)<<4) | (s1>>16)).astype(np.uint8)
            b3 = ((s1>>8)&0xFF).astype(np.uint8)
            b4 = (s1&0xFF).astype(np.uint8)
            return np.stack([b0,b1,b2,b3,b4], axis=1).tobytes()

        if bps == 22:
            pad = (-n) % 4
            p = np.zeros(n + pad, dtype=np.uint64); p[:n] = idx
            s = p.reshape(-1, 4)
            s0,s1,s2,s3 = (s[:,0]&0x3FFFFF, s[:,1]&0x3FFFFF,
                            s[:,2]&0x3FFFFF, s[:,3]&0x3FFFFF)
            b0  = (s0>>14).astype(np.uint8)
            b1  = ((s0>>6)&0xFF).astype(np.uint8)
            b2  = (((s0&0x3F)<<2) | (s1>>20)).astype(np.uint8)
            b3  = ((s1>>12)&0xFF).astype(np.uint8)
            b4  = ((s1>>4)&0xFF).astype(np.uint8)
            b5  = (((s1&0xF) <<4) | (s2>>18)).astype(np.uint8)
            b6  = ((s2>>10)&0xFF).astype(np.uint8)
            b7  = ((s2>>2)&0xFF).astype(np.uint8)
            b8  = (((s2&0x3) <<6) | (s3>>16)).astype(np.uint8)
            b9  = ((s3>>8)&0xFF).astype(np.uint8)
            b10 = (s3&0xFF).astype(np.uint8)
            return np.stack([b0,b1,b2,b3,b4,b5,b6,b7,b8,b9,b10], axis=1).tobytes()

        if bps == 24:
            s0 = idx & 0xFFFFFF
            b0 = (s0>>16).astype(np.uint8)
            b1 = ((s0>>8)&0xFF).astype(np.uint8)
            b2 = (s0&0xFF).astype(np.uint8)
            return np.stack([b0,b1,b2], axis=1).tobytes()

        if bps == 26:
            pad = (-n) % 4
            p = np.zeros(n + pad, dtype=np.uint64); p[:n] = idx
            s = p.reshape(-1, 4)
            s0,s1,s2,s3 = (s[:,0]&0x3FFFFFF, s[:,1]&0x3FFFFFF,
                            s[:,2]&0x3FFFFFF, s[:,3]&0x3FFFFFF)
            b0  = (s0>>18).astype(np.uint8)
            b1  = ((s0>>10)&0xFF).astype(np.uint8)
            b2  = ((s0>>2)&0xFF).astype(np.uint8)
            b3  = (((s0&0x3) <<6) | (s1>>20)).astype(np.uint8)
            b4  = ((s1>>12)&0xFF).astype(np.uint8)
            b5  = ((s1>>4)&0xFF).astype(np.uint8)
            b6  = (((s1&0xF) <<4) | (s2>>22)).astype(np.uint8)
            b7  = ((s2>>14)&0xFF).astype(np.uint8)
            b8  = ((s2>>6)&0xFF).astype(np.uint8)
            b9  = (((s2&0x3F)<<2) | (s3>>24)).astype(np.uint8)
            b10 = ((s3>>16)&0xFF).astype(np.uint8)
            b11 = ((s3>>8)&0xFF).astype(np.uint8)
            b12 = (s3&0xFF).astype(np.uint8)
            return np.stack([b0,b1,b2,b3,b4,b5,b6,b7,b8,b9,b10,b11,b12], axis=1).tobytes()

        if bps == 28:
            pad = n % 2
            p = np.zeros(n + pad, dtype=np.uint64); p[:n] = idx
            s = p.reshape(-1, 2)
            s0,s1 = s[:,0]&0xFFFFFFF, s[:,1]&0xFFFFFFF
            b0 = (s0>>20).astype(np.uint8)
            b1 = ((s0>>12)&0xFF).astype(np.uint8)
            b2 = ((s0>>4)&0xFF).astype(np.uint8)
            b3 = (((s0&0xF)<<4) | (s1>>24)).astype(np.uint8)
            b4 = ((s1>>16)&0xFF).astype(np.uint8)
            b5 = ((s1>>8)&0xFF).astype(np.uint8)
            b6 = (s1&0xFF).astype(np.uint8)
            return np.stack([b0,b1,b2,b3,b4,b5,b6], axis=1).tobytes()

        if bps == 30:
            pad = (-n) % 4
            p = np.zeros(n + pad, dtype=np.uint64); p[:n] = idx
            s = p.reshape(-1, 4)
            s0,s1,s2,s3 = (s[:,0]&0x3FFFFFFF, s[:,1]&0x3FFFFFFF,
                            s[:,2]&0x3FFFFFFF, s[:,3]&0x3FFFFFFF)
            b0  = (s0>>22).astype(np.uint8)
            b1  = ((s0>>14)&0xFF).astype(np.uint8)
            b2  = ((s0>>6)&0xFF).astype(np.uint8)
            b3  = (((s0&0x3F)<<2) | (s1>>28)).astype(np.uint8)
            b4  = ((s1>>20)&0xFF).astype(np.uint8)
            b5  = ((s1>>12)&0xFF).astype(np.uint8)
            b6  = ((s1>>4)&0xFF).astype(np.uint8)
            b7  = (((s1&0xF) <<4) | (s2>>26)).astype(np.uint8)
            b8  = ((s2>>18)&0xFF).astype(np.uint8)
            b9  = ((s2>>10)&0xFF).astype(np.uint8)
            b10 = ((s2>>2)&0xFF).astype(np.uint8)
            b11 = (((s2&0x3) <<6) | (s3>>24)).astype(np.uint8)
            b12 = ((s3>>16)&0xFF).astype(np.uint8)
            b13 = ((s3>>8)&0xFF).astype(np.uint8)
            b14 = (s3&0xFF).astype(np.uint8)
            return np.stack([b0,b1,b2,b3,b4,b5,b6,b7,b8,b9,b10,b11,b12,b13,b14],
                            axis=1).tobytes()

        if bps == 32:
            s0 = idx & 0xFFFFFFFF
            b0 = (s0>>24).astype(np.uint8)
            b1 = ((s0>>16)&0xFF).astype(np.uint8)
            b2 = ((s0>>8)&0xFF).astype(np.uint8)
            b3 = (s0&0xFF).astype(np.uint8)
            return np.stack([b0,b1,b2,b3], axis=1).tobytes()

        raise ValueError(f"X Unsupported bits-per-sample: {bps}")

    @staticmethod
    def _unpack_pcm(raw_bytes: bytes, bps: int, num_samples: int) -> np.ndarray:
        data = np.frombuffer(raw_bytes, dtype=np.uint8)
        n = num_samples
        max_val = (1 << bps) - 1  

        if bps == 1:
            indices = np.unpackbits(data)[:n].astype(np.float32)
            return indices * 2.0 - 1.0  

        if bps == 2:
            nb = (n + 3) // 4; d = data[:nb]
            out = np.empty(nb * 4, dtype=np.uint8)
            out[0::4] = (d>>6)&3; out[1::4] = (d>>4)&3
            out[2::4] = (d>>2)&3; out[3::4] =  d    &3
            return (out[:n].astype(np.float32) / max_val) * 2.0 - 1.0

        if bps == 4:
            nb = (n + 1) // 2; d = data[:nb]
            out = np.empty(nb * 2, dtype=np.uint8)
            out[0::2] = (d>>4)&0xF; out[1::2] = d&0xF
            return (out[:n].astype(np.float32) / max_val) * 2.0 - 1.0

        if bps == 6:
            ng = (n + 3) // 4
            d = data[:ng*3].reshape(-1, 3).astype(np.uint32)
            b0,b1,b2 = d[:,0], d[:,1], d[:,2]
            s = np.empty(ng * 4, dtype=np.uint32)
            s[0::4] = (b0>>2)&0x3F
            s[1::4] = ((b0&0x3)<<4) | (b1>>4)
            s[2::4] = ((b1&0xF)<<2) | (b2>>6)
            s[3::4] =  b2 & 0x3F
            return (s[:n].astype(np.float32) / max_val) * 2.0 - 1.0

        if bps == 8:
            return (data[:n].astype(np.float32) / max_val) * 2.0 - 1.0

        if bps == 10:
            ng = (n + 3) // 4
            d = data[:ng*5].reshape(-1, 5).astype(np.uint32)
            b0,b1,b2,b3,b4 = d[:,0],d[:,1],d[:,2],d[:,3],d[:,4]
            s = np.empty(ng * 4, dtype=np.uint32)
            s[0::4] = (b0<<2) | (b1>>6)
            s[1::4] = ((b1&0x3F)<<4) | (b2>>4)
            s[2::4] = ((b2&0x0F)<<6) | (b3>>2)
            s[3::4] = ((b3&0x03)<<8) |  b4
            return (s[:n].astype(np.float32) / max_val) * 2.0 - 1.0

        if bps == 12:
            ng = (n + 1) // 2
            d = data[:ng*3].reshape(-1, 3).astype(np.uint32)
            b0,b1,b2 = d[:,0], d[:,1], d[:,2]
            s = np.empty(ng * 2, dtype=np.uint32)
            s[0::2] = (b0<<4) | (b1>>4)
            s[1::2] = ((b1&0xF)<<8) | b2
            return (s[:n].astype(np.float32) / max_val) * 2.0 - 1.0

        if bps == 14:
            ng = (n + 3) // 4
            d = data[:ng*7].reshape(-1, 7).astype(np.uint32)
            b = [d[:,i] for i in range(7)]
            s = np.empty(ng * 4, dtype=np.uint32)
            s[0::4] = (b[0]<<6) | (b[1]>>2)
            s[1::4] = ((b[1]&0x03)<<12) | (b[2]<<4) | (b[3]>>4)
            s[2::4] = ((b[3]&0x0F)<<10) | (b[4]<<2) | (b[5]>>6)
            s[3::4] = ((b[5]&0x3F)<<8)  |  b[6]
            return (s[:n].astype(np.float32) / max_val) * 2.0 - 1.0

        if bps == 16:
            signed = np.frombuffer(raw_bytes[:n * 2], dtype=np.int16)
            return signed.astype(np.float32) / 32768.0

        if bps == 18:
            ng = (n + 3) // 4
            d = data[:ng*9].reshape(-1, 9).astype(np.uint64)
            b = [d[:,i] for i in range(9)]
            s = np.empty(ng * 4, dtype=np.uint64)
            s[0::4] = (b[0]<<10) | (b[1]<<2) | (b[2]>>6)
            s[1::4] = ((b[2]&0x3F)<<12) | (b[3]<<4) | (b[4]>>4)
            s[2::4] = ((b[4]&0x0F)<<14) | (b[5]<<6) | (b[6]>>2)
            s[3::4] = ((b[6]&0x03)<<16) | (b[7]<<8) |  b[8]
            return (s[:n].astype(np.float64) / max_val * 2.0 - 1.0).astype(np.float32)

        if bps == 20:
            ng = (n + 1) // 2
            d = data[:ng*5].reshape(-1, 5).astype(np.uint64)
            b = [d[:,i] for i in range(5)]
            s = np.empty(ng * 2, dtype=np.uint64)
            s[0::2] = (b[0]<<12) | (b[1]<<4) | (b[2]>>4)
            s[1::2] = ((b[2]&0xF)<<16) | (b[3]<<8) | b[4]
            return (s[:n].astype(np.float64) / max_val * 2.0 - 1.0).astype(np.float32)

        if bps == 22:
            ng = (n + 3) // 4
            d = data[:ng*11].reshape(-1, 11).astype(np.uint64)
            b = [d[:,i] for i in range(11)]
            s = np.empty(ng * 4, dtype=np.uint64)
            s[0::4] = (b[0]<<14) | (b[1]<<6) | (b[2]>>2)
            s[1::4] = ((b[2]&0x03)<<20) | (b[3]<<12) | (b[4]<<4) | (b[5]>>4)
            s[2::4] = ((b[5]&0x0F)<<18) | (b[6]<<10) | (b[7]<<2) | (b[8]>>6)
            s[3::4] = ((b[8]&0x3F)<<16) | (b[9]<<8)  |  b[10]
            return (s[:n].astype(np.float64) / max_val * 2.0 - 1.0).astype(np.float32)

        if bps == 24:
            d = data[:n*3].reshape(-1, 3).astype(np.uint64)
            s = (d[:,0]<<16) | (d[:,1]<<8) | d[:,2]
            return (s.astype(np.float64) / max_val * 2.0 - 1.0).astype(np.float32)

        if bps == 26:
            ng = (n + 3) // 4
            d = data[:ng*13].reshape(-1, 13).astype(np.uint64)
            b = [d[:,i] for i in range(13)]
            s = np.empty(ng * 4, dtype=np.uint64)
            s[0::4] = (b[0]<<18) | (b[1]<<10) | (b[2]<<2) | (b[3]>>6)
            s[1::4] = ((b[3]&0x3F)<<20) | (b[4]<<12) | (b[5]<<4) | (b[6]>>4)
            s[2::4] = ((b[6]&0x0F)<<22) | (b[7]<<14) | (b[8]<<6) | (b[9]>>2)
            s[3::4] = ((b[9]&0x03)<<24) | (b[10]<<16) | (b[11]<<8) | b[12]
            return (s[:n].astype(np.float64) / max_val * 2.0 - 1.0).astype(np.float32)

        if bps == 28:
            ng = (n + 1) // 2
            d = data[:ng*7].reshape(-1, 7).astype(np.uint64)
            b = [d[:,i] for i in range(7)]
            s = np.empty(ng * 2, dtype=np.uint64)
            s[0::2] = (b[0]<<20) | (b[1]<<12) | (b[2]<<4) | (b[3]>>4)
            s[1::2] = ((b[3]&0xF)<<24) | (b[4]<<16) | (b[5]<<8) | b[6]
            return (s[:n].astype(np.float64) / max_val * 2.0 - 1.0).astype(np.float32)

        if bps == 30:
            ng = (n + 3) // 4
            d = data[:ng*15].reshape(-1, 15).astype(np.uint64)
            b = [d[:,i] for i in range(15)]
            s = np.empty(ng * 4, dtype=np.uint64)
            s[0::4] = (b[0]<<22) | (b[1]<<14) | (b[2]<<6) | (b[3]>>2)
            s[1::4] = ((b[3]&0x03)<<28) | (b[4]<<20) | (b[5]<<12) | (b[6]<<4) | (b[7]>>4)
            s[2::4] = ((b[7]&0x0F)<<26) | (b[8]<<18) | (b[9]<<10) | (b[10]<<2) | (b[11]>>6)
            s[3::4] = ((b[11]&0x3F)<<24) | (b[12]<<16) | (b[13]<<8) | b[14]
            return (s[:n].astype(np.float64) / max_val * 2.0 - 1.0).astype(np.float32)

        if bps == 32:
            d = data[:n*4].reshape(-1, 4).astype(np.uint64)
            s = (d[:,0]<<24) | (d[:,1]<<16) | (d[:,2]<<8) | d[:,3]
            return (s.astype(np.float64) / max_val * 2.0 - 1.0).astype(np.float32)

        raise ValueError(f"X Unsupported bits-per-sample: {bps}")

    @staticmethod
    def read(filepath: str) -> dict:
        with open(filepath, 'rb') as f:
            if f.read(16) != CFDA_MAGIC:
                raise ValueError("X Invalid CFDA file: Incorrect header signature.")
            meta = f.read(16)
            if len(meta) < 16:
                raise ValueError("X Invalid CFDA file: Incomplete metadata.")

            num_ch, sr, br, bps, file_size = struct.unpack(METADATA_STRUCT, meta)
            pcm_bytes = f.read()

            total_bits = len(pcm_bytes) * 8
            num_samples = total_bits // bps
            float_data = CFDAAudio._unpack_pcm(pcm_bytes, bps, num_samples)

            if num_ch > 1:
                float_data = float_data.reshape(-1, num_ch)
            else:
                float_data = float_data.reshape(-1, 1)

            return {
                'num_channels': num_ch, 'sample_rate': sr,
                'byte_rate': br, 'bits_per_sample': bps,
                'file_size': file_size, 'data': float_data
            }

    @staticmethod
    def write(filepath: str, data: np.ndarray, sample_rate: int,
              bits_per_sample: int = 16):
        data = np.asarray(data, dtype=np.float32)
        num_ch = 1 if data.ndim == 1 else data.shape[1]
        if num_ch == 1:
            data = data.reshape(-1, 1)
        flat_data = data.reshape(-1)

        pcm_bytes = CFDAAudio._pack_pcm(flat_data, bits_per_sample)
        byte_rate = int(round(sample_rate * num_ch * bits_per_sample / 8.0))
        total_size = HEADER_SIZE + len(pcm_bytes)
        file_size = total_size - 33

        with open(filepath, 'wb') as f:
            f.write(CFDA_MAGIC)
            f.write(struct.pack(METADATA_STRUCT,
                                num_ch, sample_rate, byte_rate,
                                bits_per_sample, file_size))
            f.write(pcm_bytes)

    @staticmethod
    def play(filepath: str, blocking: bool = True):
        cfda = CFDAAudio.read(filepath)
        print(f"> Playing: {filepath}")
        print(f"  Channels: {cfda['num_channels']} | "
              f"Sample Rate: {cfda['sample_rate']} Hz | "
              f"BPS: {cfda['bits_per_sample']}")
        try:
            sd.play(cfda['data'], samplerate=cfda['sample_rate'], blocking=blocking)
            if not blocking:
                print("Press Ctrl+C to stop playback...")
                sd.wait()
        except KeyboardInterrupt:
            sd.stop()
            print("\n! Playback stopped.")


def _load_wav(path: str):
    with wave.open(path, 'rb') as wf:
        if wf.getcomptype() != 'NONE':
            raise ValueError("Only uncompressed PCM WAV files are supported.")
        sr, ch, width = wf.getframerate(), wf.getnchannels(), wf.getsampwidth()
        raw = wf.readframes(wf.getnframes())

    if width == 1:
        data = (np.frombuffer(raw, np.uint8).astype(np.float32) - 128) / 128.0
    elif width == 2:
        data = np.frombuffer(raw, np.int16).astype(np.float32) / 32768.0
    elif width == 3:
        data = np.array([int.from_bytes(raw[i:i+3], 'little', signed=True)
                         for i in range(0, len(raw), 3)],
                        dtype=np.float32) / 8388608.0
    elif width == 4:
        data = np.frombuffer(raw, np.int32).astype(np.float32) / 2147483648.0
    else:
        raise ValueError(f"X Unsupported WAV width: {width}")
    return data.reshape(-1, ch), sr, ch


def _load_raw(path: str, sr: int, ch: int, bps: int, endian: str):
    ec = '<' if endian == 'little' else '>'
    with open(path, 'rb') as f:
        raw = f.read()
    if bps == 8:
        data = (np.frombuffer(raw, dtype=f'{ec}u1').astype(np.float32) - 128) / 128.0
    elif bps == 16:
        data = np.frombuffer(raw, dtype=f'{ec}i2').astype(np.float32) / 32768.0
    elif bps == 24:
        data = np.array([int.from_bytes(raw[i:i+3], endian, signed=True)
                         for i in range(0, len(raw), 3)],
                        dtype=np.float32) / 8388608.0
    elif bps == 32:
        data = np.frombuffer(raw, dtype=f'{ec}i4').astype(np.float32) / 2147483648.0
    else:
        raise ValueError("RAW conversion supports 8, 16, 24, or 32 bps.")
    return data.reshape(-1, ch), sr, ch


def _pcm_to_bytes(data_int: np.ndarray, bps: int) -> bytes:
    if bps == 8:
        return (data_int + 128).clip(0, 255).astype(np.uint8).tobytes()
    elif bps == 16:
        return data_int.clip(-32768, 32767).astype(np.int16).tobytes()
    elif bps == 24:
        out = bytearray(len(data_int) * 3)
        for i, v in enumerate(data_int):
            v = int(np.clip(v, -8388608, 8388607))
            if v < 0: v += 1 << 24
            out[i*3:i*3+3] = v.to_bytes(3, 'little')
        return bytes(out)
    elif bps == 32:
        return data_int.clip(-2147483648, 2147483647).astype(np.int32).tobytes()
    raise ValueError(f"Unsupported target BPS: {bps}")


def _write_wav(filepath: str, data_float: np.ndarray, sr: int, ch: int, bps: int):
    max_val = 127 if bps == 8 else (1 << (bps - 1)) - 1
    min_val = 0   if bps == 8 else -max_val - 1

    flat = data_float.reshape(-1)
    int_data = np.clip(np.round(flat * max_val), min_val, max_val).astype(np.int64)
    pcm_bytes = _pcm_to_bytes(int_data, bps)

    byte_rate   = sr * ch * (bps // 8)
    block_align = ch * (bps // 8)
    data_size   = len(pcm_bytes)
    chunk_size  = 36 + data_size

    with open(filepath, 'wb') as f:
        f.write(b'RIFF')
        f.write(struct.pack('<I', chunk_size))
        f.write(b'WAVEfmt ')
        f.write(struct.pack('<IHHIIHH', 16, 1, ch, sr, byte_rate, block_align, bps))
        f.write(b'data')
        f.write(struct.pack('<I', data_size))
        f.write(pcm_bytes)


def main():
    parser = argparse.ArgumentParser(description="CFDA Audio Format Tool")
    sub = parser.add_subparsers(dest='command', required=True)

    p_play = sub.add_parser('play', help='Play a CFDA file')
    p_play.add_argument('file', help='Path to .cfda file')

    p_create = sub.add_parser('create', help='Generate a test CFDA file')
    p_create.add_argument('file', help='Output path (.cfda)')
    p_create.add_argument('--duration', type=float, default=2.0)
    p_create.add_argument('--freq',     type=float, default=440.0)
    p_create.add_argument('--sr',       type=int,   default=44100)
    p_create.add_argument('--channels', type=int,   default=2, choices=[1, 2])
    p_create.add_argument('--bps',      type=int,   default=16,
                          choices=[1,2,4,6,8,10,12,14,16,18,20,22,24,26,28,30,32])

    p_conv = sub.add_parser('convert', help='Convert WAV or RAW to CFDA')
    p_conv.add_argument('input',  help='Input file (any extension)')
    p_conv.add_argument('output', help='Output .cfda file')
    p_conv.add_argument('--format',    choices=['wav', 'raw'], required=True)
    p_conv.add_argument('--channels',  type=int, choices=[1, 2],
                        help='Force output channels (downmixes stereo→mono if 1)')
    p_conv.add_argument('--bps',       type=int, default=16,
                        choices=[1,2,4,6,8,10,12,14,16,18,20,22,24,26,28,30,32])
    p_conv.add_argument('--raw-sr',    type=int, default=44100)
    p_conv.add_argument('--raw-ch',    type=int, default=1)
    p_conv.add_argument('--raw-bps',   type=int, default=16)
    p_conv.add_argument('--raw-endian', choices=['little', 'big'], default='little')

    p_dec = sub.add_parser('decode', help='Decode CFDA to WAV or RAW')
    p_dec.add_argument('input',  help='Input .cfda file')
    p_dec.add_argument('output', help='Output file')
    p_dec.add_argument('--format',     choices=['wav', 'raw'], required=True)
    p_dec.add_argument('--target-bps', type=int, choices=[8, 16, 24, 32])
    p_dec.add_argument('--passthrough', action='store_true')

    args = parser.parse_args()

    if args.command == 'play':
        CFDAAudio.play(args.file)

    elif args.command == 'create':
        t = np.linspace(0, args.duration,
                        int(args.sr * args.duration), endpoint=False)
        audio = 0.9 * np.sin(2 * np.pi * args.freq * t)
        if args.channels == 2:
            audio = np.column_stack((audio, audio))
        CFDAAudio.write(args.file, audio, args.sr, bits_per_sample=args.bps)
        print(f"! Created: {args.file}")

    elif args.command == 'convert':
        if args.format == 'wav':
            audio, sr, ch = _load_wav(args.input)
        else:
            audio, sr, ch = _load_raw(args.input, args.raw_sr, args.raw_ch,
                                       args.raw_bps, args.raw_endian)
        if args.channels and args.channels != ch:
            if args.channels == 1 and ch == 2:
                audio = np.mean(audio, axis=1, keepdims=True)
                ch = 1
                print(">< Downmixed stereo to mono")
            else:
                print(f"?  Cannot change channels from {ch} to {args.channels}. Skipping.")
        CFDAAudio.write(args.output, audio, sr, bits_per_sample=args.bps)
        print(f"! Converted: {args.input} -> {args.output} ({args.bps}bps, {ch}ch)")

    elif args.command == 'decode':
        cfda = CFDAAudio.read(args.input)
        sr, ch, bps = (cfda['sample_rate'], cfda['num_channels'],
                       cfda['bits_per_sample'])
        target_bps = args.target_bps or (bps if bps in (8, 16, 24, 32) else 16)

        if args.passthrough:
            if args.format != 'raw':
                print("X --passthrough only works with --format raw")
                sys.exit(1)
            print("@ Raw passthrough: Stripping 32-byte header...")
            with open(args.input, 'rb') as fin:
                fin.seek(32)
                packed_bytes = fin.read()
            with open(args.output, 'wb') as fout:
                fout.write(packed_bytes)
            print(f"! Decoded: {args.input} -> {args.output} (exact bit-packed stream)")
        else:
            print(f"@ Decoding: {bps}bps CFDA -> {target_bps}bps {args.format.upper()}...")
            if args.format == 'wav':
                _write_wav(args.output, cfda['data'], sr, ch, target_bps)
            else:
                flat = cfda['data'].reshape(-1)
                max_val = (1 << (target_bps - 1)) - 1
                int_data = np.clip(np.round(flat * max_val),
                                   -max_val - 1, max_val).astype(np.int64)
                with open(args.output, 'wb') as f:
                    f.write(_pcm_to_bytes(int_data, target_bps))
            print(f"! Decoded: {args.input} -> {args.output} "
                  f"({target_bps}bps, {ch}ch, {sr}Hz)")


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\nExiting.")
        sys.exit(0)
    except Exception as e:
        print(f"X Error: {e}")
        sys.exit(1)
