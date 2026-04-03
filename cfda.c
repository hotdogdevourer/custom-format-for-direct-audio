/*
CFDA (Custom Format for Direct Audio) Audio Format (.cfda) - Reader, Writer & Player
Format Specification:
  Offset | Size | Name          | Description
  0      | 16   | Header        | "CFDA" + 0xCFDAB8746CDF2A6DEADBEEFFE
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
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

typedef unsigned char  uint8_t;
typedef signed   short int16_t;
typedef unsigned short uint16_t;
typedef signed   long  int32_t;
typedef unsigned long  uint32_t;

typedef struct { uint32_t hi; uint32_t lo; } uint64_s;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const uint8_t CFDA_MAGIC[16] = {
    'C','F','D','A',
    0xCF,0xDA,0x87,0x46,0xCD,0xF2,0xA6,0xDE,
    0xAD,0xBE,0xEF,0xFE
};

#define HEADER_SIZE 32


#define write_u16le(f,v) do { uint16_t _v=(v); uint8_t _b[2]; \
    _b[0]=(uint8_t)(_v&0xFF); _b[1]=(uint8_t)((_v>>8)&0xFF); \
    fwrite(_b,1,2,(f)); } while(0)

#define write_u32le(f,v) do { uint32_t _v=(v); uint8_t _b[4]; \
    _b[0]=(uint8_t)(_v&0xFF); _b[1]=(uint8_t)((_v>>8)&0xFF); \
    _b[2]=(uint8_t)((_v>>16)&0xFF); _b[3]=(uint8_t)((_v>>24)&0xFF); \
    fwrite(_b,1,4,(f)); } while(0)

#define read_u16le(p) ((uint16_t)((p)[0] | ((uint16_t)(p)[1] << 8)))
#define read_u32le(p) ((uint32_t)(p)[0] | ((uint32_t)(p)[1]<<8) | \
                       ((uint32_t)(p)[2]<<16) | ((uint32_t)(p)[3]<<24))

typedef struct {
    uint16_t num_channels;
    uint32_t sample_rate;
    uint32_t byte_rate;
    uint16_t bits_per_sample;
    uint32_t file_size;
    float   *data;                                          
    uint32_t num_frames;                                    
} CFDAAudio;

static void cfda_free(CFDAAudio *a) {
    if (a && a->data) { free(a->data); a->data = NULL; }
}

#define FTI_SCALE(mv) (0.5 * (double)((mv) + 1))
static uint32_t float_to_idx(double s, uint32_t max_val) {
    uint32_t u = (uint32_t)((s + 1.0) * FTI_SCALE(max_val));
    return (u > max_val) ? max_val : u;
}
#define idx_to_float(idx, max_val) ((double)(idx) / (double)(max_val) * 2.0 - 1.0)

static uint8_t *pack_pcm(const float *samples, uint32_t n, int bps,
                          uint32_t *out_len) {
    uint32_t max_val = (bps == 32) ? 0xFFFFFFFFUL : ((1UL << bps) - 1);
    uint8_t *buf = NULL;
    uint32_t i, ng, buf_sz;

    switch (bps) {

    case 1: {
        buf_sz = (n + 7) / 8;
        buf = (uint8_t*)calloc(buf_sz, 1);
        if (!buf) return NULL;
        for (i = 0; i < n; i++) {
            if (samples[i] >= 0.0f)
                buf[i >> 3] |= (uint8_t)(0x80u >> (i & 7));
        }
        *out_len = buf_sz;
        return buf;
    }

    case 2: {
        ng = (n + 3) / 4;
        buf_sz = ng;
        buf = (uint8_t*)calloc(buf_sz, 1);
        if (!buf) return NULL;
        for (i = 0; i < ng; i++) {
            uint32_t s0 = (i*4+0 < n) ? float_to_idx(samples[i*4+0], max_val) & 3 : 0;
            uint32_t s1 = (i*4+1 < n) ? float_to_idx(samples[i*4+1], max_val) & 3 : 0;
            uint32_t s2 = (i*4+2 < n) ? float_to_idx(samples[i*4+2], max_val) & 3 : 0;
            uint32_t s3 = (i*4+3 < n) ? float_to_idx(samples[i*4+3], max_val) & 3 : 0;
            buf[i] = (uint8_t)((s0<<6)|(s1<<4)|(s2<<2)|s3);
        }
        *out_len = buf_sz;
        return buf;
    }

    case 4: {
        ng = (n + 1) / 2;
        buf_sz = ng;
        buf = (uint8_t*)calloc(buf_sz, 1);
        if (!buf) return NULL;
        for (i = 0; i < ng; i++) {
            uint32_t s0 = (i*2+0 < n) ? float_to_idx(samples[i*2+0], max_val) & 0xF : 0;
            uint32_t s1 = (i*2+1 < n) ? float_to_idx(samples[i*2+1], max_val) & 0xF : 0;
            buf[i] = (uint8_t)((s0<<4)|s1);
        }
        *out_len = buf_sz;
        return buf;
    }

    case 6: {
        ng = (n + 3) / 4;
        buf_sz = ng * 3;
        buf = (uint8_t*)calloc(buf_sz, 1);
        if (!buf) return NULL;
        for (i = 0; i < ng; i++) {
            uint32_t s0 = (i*4+0 < n) ? float_to_idx(samples[i*4+0], max_val) & 0x3F : 0;
            uint32_t s1 = (i*4+1 < n) ? float_to_idx(samples[i*4+1], max_val) & 0x3F : 0;
            uint32_t s2 = (i*4+2 < n) ? float_to_idx(samples[i*4+2], max_val) & 0x3F : 0;
            uint32_t s3 = (i*4+3 < n) ? float_to_idx(samples[i*4+3], max_val) & 0x3F : 0;
            buf[i*3+0] = (uint8_t)((s0<<2)|(s1>>4));
            buf[i*3+1] = (uint8_t)(((s1&0xF)<<4)|(s2>>2));
            buf[i*3+2] = (uint8_t)(((s2&0x3)<<6)|s3);
        }
        *out_len = buf_sz;
        return buf;
    }

    case 8: {
        buf_sz = n;
        buf = (uint8_t*)malloc(buf_sz);
        if (!buf) return NULL;
        for (i = 0; i < n; i++)
            buf[i] = (uint8_t)(float_to_idx(samples[i], max_val) & 0xFF);
        *out_len = buf_sz;
        return buf;
    }

    case 10: {
        ng = (n + 3) / 4;
        buf_sz = ng * 5;
        buf = (uint8_t*)calloc(buf_sz, 1);
        if (!buf) return NULL;
        for (i = 0; i < ng; i++) {
            uint32_t s0 = (i*4+0 < n) ? float_to_idx(samples[i*4+0], max_val) & 0x3FF : 0;
            uint32_t s1 = (i*4+1 < n) ? float_to_idx(samples[i*4+1], max_val) & 0x3FF : 0;
            uint32_t s2 = (i*4+2 < n) ? float_to_idx(samples[i*4+2], max_val) & 0x3FF : 0;
            uint32_t s3 = (i*4+3 < n) ? float_to_idx(samples[i*4+3], max_val) & 0x3FF : 0;
            buf[i*5+0] = (uint8_t)(s0>>2);
            buf[i*5+1] = (uint8_t)(((s0&0x03)<<6)|(s1>>4));
            buf[i*5+2] = (uint8_t)(((s1&0x0F)<<4)|(s2>>6));
            buf[i*5+3] = (uint8_t)(((s2&0x3F)<<2)|(s3>>8));
            buf[i*5+4] = (uint8_t)(s3&0xFF);
        }
        *out_len = buf_sz;
        return buf;
    }

    case 12: {
        ng = (n + 1) / 2;
        buf_sz = ng * 3;
        buf = (uint8_t*)calloc(buf_sz, 1);
        if (!buf) return NULL;
        for (i = 0; i < ng; i++) {
            uint32_t s0 = (i*2+0 < n) ? float_to_idx(samples[i*2+0], max_val) & 0xFFF : 0;
            uint32_t s1 = (i*2+1 < n) ? float_to_idx(samples[i*2+1], max_val) & 0xFFF : 0;
            buf[i*3+0] = (uint8_t)(s0>>4);
            buf[i*3+1] = (uint8_t)(((s0&0xF)<<4)|(s1>>8));
            buf[i*3+2] = (uint8_t)(s1&0xFF);
        }
        *out_len = buf_sz;
        return buf;
    }

    case 14: {
        ng = (n + 3) / 4;
        buf_sz = ng * 7;
        buf = (uint8_t*)calloc(buf_sz, 1);
        if (!buf) return NULL;
        for (i = 0; i < ng; i++) {
            uint32_t s0 = (i*4+0 < n) ? float_to_idx(samples[i*4+0], max_val) & 0x3FFF : 0;
            uint32_t s1 = (i*4+1 < n) ? float_to_idx(samples[i*4+1], max_val) & 0x3FFF : 0;
            uint32_t s2 = (i*4+2 < n) ? float_to_idx(samples[i*4+2], max_val) & 0x3FFF : 0;
            uint32_t s3 = (i*4+3 < n) ? float_to_idx(samples[i*4+3], max_val) & 0x3FFF : 0;
            buf[i*7+0] = (uint8_t)(s0>>6);
            buf[i*7+1] = (uint8_t)(((s0&0x3F)<<2)|(s1>>12));
            buf[i*7+2] = (uint8_t)((s1>>4)&0xFF);
            buf[i*7+3] = (uint8_t)(((s1&0xF)<<4)|(s2>>10));
            buf[i*7+4] = (uint8_t)((s2>>2)&0xFF);
            buf[i*7+5] = (uint8_t)(((s2&0x3)<<6)|(s3>>8));
            buf[i*7+6] = (uint8_t)(s3&0xFF);
        }
        *out_len = buf_sz;
        return buf;
    }

    case 16: {
        buf_sz = n * 2;
        buf = (uint8_t*)malloc(buf_sz);
        if (!buf) return NULL;
        for (i = 0; i < n; i++) {
            int32_t v = (int32_t)(float_to_idx(samples[i], max_val)) - 32768;
            if (v < -32768) v = -32768;
            if (v >  32767) v =  32767;
            buf[i*2+0] = (uint8_t)(v & 0xFF);
            buf[i*2+1] = (uint8_t)((v >> 8) & 0xFF);
        }
        *out_len = buf_sz;
        return buf;
    }

    case 18: {
        ng = (n + 3) / 4;
        buf_sz = ng * 9;
        buf = (uint8_t*)calloc(buf_sz, 1);
        if (!buf) return NULL;
        for (i = 0; i < ng; i++) {
            uint32_t s0 = (i*4+0 < n) ? float_to_idx(samples[i*4+0], max_val) & 0x3FFFF : 0;
            uint32_t s1 = (i*4+1 < n) ? float_to_idx(samples[i*4+1], max_val) & 0x3FFFF : 0;
            uint32_t s2 = (i*4+2 < n) ? float_to_idx(samples[i*4+2], max_val) & 0x3FFFF : 0;
            uint32_t s3 = (i*4+3 < n) ? float_to_idx(samples[i*4+3], max_val) & 0x3FFFF : 0;
            buf[i*9+0] = (uint8_t)(s0>>10);
            buf[i*9+1] = (uint8_t)((s0>>2)&0xFF);
            buf[i*9+2] = (uint8_t)(((s0&0x3)<<6)|(s1>>12));
            buf[i*9+3] = (uint8_t)((s1>>4)&0xFF);
            buf[i*9+4] = (uint8_t)(((s1&0xF)<<4)|(s2>>14));
            buf[i*9+5] = (uint8_t)((s2>>6)&0xFF);
            buf[i*9+6] = (uint8_t)(((s2&0x3F)<<2)|(s3>>16));
            buf[i*9+7] = (uint8_t)((s3>>8)&0xFF);
            buf[i*9+8] = (uint8_t)(s3&0xFF);
        }
        *out_len = buf_sz;
        return buf;
    }

    case 20: {
        ng = (n + 1) / 2;
        buf_sz = ng * 5;
        buf = (uint8_t*)calloc(buf_sz, 1);
        if (!buf) return NULL;
        for (i = 0; i < ng; i++) {
            uint32_t s0 = (i*2+0 < n) ? float_to_idx(samples[i*2+0], max_val) & 0xFFFFF : 0;
            uint32_t s1 = (i*2+1 < n) ? float_to_idx(samples[i*2+1], max_val) & 0xFFFFF : 0;
            buf[i*5+0] = (uint8_t)(s0>>12);
            buf[i*5+1] = (uint8_t)((s0>>4)&0xFF);
            buf[i*5+2] = (uint8_t)(((s0&0xF)<<4)|(s1>>16));
            buf[i*5+3] = (uint8_t)((s1>>8)&0xFF);
            buf[i*5+4] = (uint8_t)(s1&0xFF);
        }
        *out_len = buf_sz;
        return buf;
    }

    case 22: {
        ng = (n + 3) / 4;
        buf_sz = ng * 11;
        buf = (uint8_t*)calloc(buf_sz, 1);
        if (!buf) return NULL;
        for (i = 0; i < ng; i++) {
            uint32_t s0 = (i*4+0 < n) ? float_to_idx(samples[i*4+0], max_val) & 0x3FFFFF : 0;
            uint32_t s1 = (i*4+1 < n) ? float_to_idx(samples[i*4+1], max_val) & 0x3FFFFF : 0;
            uint32_t s2 = (i*4+2 < n) ? float_to_idx(samples[i*4+2], max_val) & 0x3FFFFF : 0;
            uint32_t s3 = (i*4+3 < n) ? float_to_idx(samples[i*4+3], max_val) & 0x3FFFFF : 0;
            buf[i*11+ 0] = (uint8_t)(s0>>14);
            buf[i*11+ 1] = (uint8_t)((s0>>6)&0xFF);
            buf[i*11+ 2] = (uint8_t)(((s0&0x3F)<<2)|(s1>>20));
            buf[i*11+ 3] = (uint8_t)((s1>>12)&0xFF);
            buf[i*11+ 4] = (uint8_t)((s1>>4)&0xFF);
            buf[i*11+ 5] = (uint8_t)(((s1&0xF)<<4)|(s2>>18));
            buf[i*11+ 6] = (uint8_t)((s2>>10)&0xFF);
            buf[i*11+ 7] = (uint8_t)((s2>>2)&0xFF);
            buf[i*11+ 8] = (uint8_t)(((s2&0x3)<<6)|(s3>>16));
            buf[i*11+ 9] = (uint8_t)((s3>>8)&0xFF);
            buf[i*11+10] = (uint8_t)(s3&0xFF);
        }
        *out_len = buf_sz;
        return buf;
    }

    case 24: {
        buf_sz = n * 3;
        buf = (uint8_t*)malloc(buf_sz);
        if (!buf) return NULL;
        for (i = 0; i < n; i++) {
            uint32_t v = float_to_idx(samples[i], max_val) & 0xFFFFFF;
            buf[i*3+0] = (uint8_t)(v>>16);
            buf[i*3+1] = (uint8_t)((v>>8)&0xFF);
            buf[i*3+2] = (uint8_t)(v&0xFF);
        }
        *out_len = buf_sz;
        return buf;
    }

    case 26: {
        ng = (n + 3) / 4;
        buf_sz = ng * 13;
        buf = (uint8_t*)calloc(buf_sz, 1);
        if (!buf) return NULL;
        for (i = 0; i < ng; i++) {
            uint32_t s0 = (i*4+0 < n) ? float_to_idx(samples[i*4+0], max_val) & 0x3FFFFFF : 0;
            uint32_t s1 = (i*4+1 < n) ? float_to_idx(samples[i*4+1], max_val) & 0x3FFFFFF : 0;
            uint32_t s2 = (i*4+2 < n) ? float_to_idx(samples[i*4+2], max_val) & 0x3FFFFFF : 0;
            uint32_t s3 = (i*4+3 < n) ? float_to_idx(samples[i*4+3], max_val) & 0x3FFFFFF : 0;
            buf[i*13+ 0] = (uint8_t)(s0>>18);
            buf[i*13+ 1] = (uint8_t)((s0>>10)&0xFF);
            buf[i*13+ 2] = (uint8_t)((s0>>2)&0xFF);
            buf[i*13+ 3] = (uint8_t)(((s0&0x3)<<6)|(s1>>20));
            buf[i*13+ 4] = (uint8_t)((s1>>12)&0xFF);
            buf[i*13+ 5] = (uint8_t)((s1>>4)&0xFF);
            buf[i*13+ 6] = (uint8_t)(((s1&0xF)<<4)|(s2>>22));
            buf[i*13+ 7] = (uint8_t)((s2>>14)&0xFF);
            buf[i*13+ 8] = (uint8_t)((s2>>6)&0xFF);
            buf[i*13+ 9] = (uint8_t)(((s2&0x3F)<<2)|(s3>>24));
            buf[i*13+10] = (uint8_t)((s3>>16)&0xFF);
            buf[i*13+11] = (uint8_t)((s3>>8)&0xFF);
            buf[i*13+12] = (uint8_t)(s3&0xFF);
        }
        *out_len = buf_sz;
        return buf;
    }

    case 28: {
        ng = (n + 1) / 2;
        buf_sz = ng * 7;
        buf = (uint8_t*)calloc(buf_sz, 1);
        if (!buf) return NULL;
        for (i = 0; i < ng; i++) {
            uint32_t s0 = (i*2+0 < n) ? float_to_idx(samples[i*2+0], max_val) & 0xFFFFFFF : 0;
            uint32_t s1 = (i*2+1 < n) ? float_to_idx(samples[i*2+1], max_val) & 0xFFFFFFF : 0;
            buf[i*7+0] = (uint8_t)(s0>>20);
            buf[i*7+1] = (uint8_t)((s0>>12)&0xFF);
            buf[i*7+2] = (uint8_t)((s0>>4)&0xFF);
            buf[i*7+3] = (uint8_t)(((s0&0xF)<<4)|(s1>>24));
            buf[i*7+4] = (uint8_t)((s1>>16)&0xFF);
            buf[i*7+5] = (uint8_t)((s1>>8)&0xFF);
            buf[i*7+6] = (uint8_t)(s1&0xFF);
        }
        *out_len = buf_sz;
        return buf;
    }

    case 30: {
        ng = (n + 3) / 4;
        buf_sz = ng * 15;
        buf = (uint8_t*)calloc(buf_sz, 1);
        if (!buf) return NULL;
        for (i = 0; i < ng; i++) {
            uint32_t s0 = (i*4+0 < n) ? float_to_idx(samples[i*4+0], max_val) & 0x3FFFFFFF : 0;
            uint32_t s1 = (i*4+1 < n) ? float_to_idx(samples[i*4+1], max_val) & 0x3FFFFFFF : 0;
            uint32_t s2 = (i*4+2 < n) ? float_to_idx(samples[i*4+2], max_val) & 0x3FFFFFFF : 0;
            uint32_t s3 = (i*4+3 < n) ? float_to_idx(samples[i*4+3], max_val) & 0x3FFFFFFF : 0;
            buf[i*15+ 0] = (uint8_t)(s0>>22);
            buf[i*15+ 1] = (uint8_t)((s0>>14)&0xFF);
            buf[i*15+ 2] = (uint8_t)((s0>>6)&0xFF);
            buf[i*15+ 3] = (uint8_t)(((s0&0x3F)<<2)|(s1>>28));
            buf[i*15+ 4] = (uint8_t)((s1>>20)&0xFF);
            buf[i*15+ 5] = (uint8_t)((s1>>12)&0xFF);
            buf[i*15+ 6] = (uint8_t)((s1>>4)&0xFF);
            buf[i*15+ 7] = (uint8_t)(((s1&0xF)<<4)|(s2>>26));
            buf[i*15+ 8] = (uint8_t)((s2>>18)&0xFF);
            buf[i*15+ 9] = (uint8_t)((s2>>10)&0xFF);
            buf[i*15+10] = (uint8_t)((s2>>2)&0xFF);
            buf[i*15+11] = (uint8_t)(((s2&0x3)<<6)|(s3>>24));
            buf[i*15+12] = (uint8_t)((s3>>16)&0xFF);
            buf[i*15+13] = (uint8_t)((s3>>8)&0xFF);
            buf[i*15+14] = (uint8_t)(s3&0xFF);
        }
        *out_len = buf_sz;
        return buf;
    }

    case 32: {
        buf_sz = n * 4;
        buf = (uint8_t*)malloc(buf_sz);
        if (!buf) return NULL;
        for (i = 0; i < n; i++) {
            uint32_t v = float_to_idx(samples[i], max_val);
            buf[i*4+0] = (uint8_t)(v>>24);
            buf[i*4+1] = (uint8_t)((v>>16)&0xFF);
            buf[i*4+2] = (uint8_t)((v>>8)&0xFF);
            buf[i*4+3] = (uint8_t)(v&0xFF);
        }
        *out_len = buf_sz;
        return buf;
    }

    default:
        fprintf(stderr, "X Unsupported bits-per-sample: %d\n", bps);
        return NULL;
    }
}

static float *unpack_pcm(const uint8_t *data, uint32_t data_len,
                         int bps, uint32_t n) {
    uint32_t max_val = (bps == 32) ? 0xFFFFFFFFUL : ((1UL << bps) - 1);
    float *out = NULL;
    uint32_t i, ng;
    (void)data_len;                                           

    out = (float*)malloc(n * sizeof(float));
    if (!out) return NULL;

    switch (bps) {

    case 1:
        for (i = 0; i < n; i++)
            out[i] = ((data[i >> 3] >> (7 - (i & 7))) & 1) ? 1.0f : -1.0f;
        return out;

    case 2:
        for (i = 0; i < n; i++) {
            uint32_t byte = data[i >> 2];
            int shift = 6 - (int)(i & 3) * 2;
            uint32_t v = (byte >> shift) & 3;
            out[i] = (float)(idx_to_float(v, max_val));
        }
        return out;

    case 4:
        for (i = 0; i < n; i++) {
            uint32_t byte = data[i >> 1];
            uint32_t v = (i & 1) ? (byte & 0xF) : ((byte >> 4) & 0xF);
            out[i] = (float)(idx_to_float(v, max_val));
        }
        return out;

    case 6:
        ng = (n + 3) / 4;
        for (i = 0; i < ng; i++) {
            uint32_t b0 = data[i*3+0], b1 = data[i*3+1], b2 = data[i*3+2];
            uint32_t s[4];
            s[0] = (b0>>2) & 0x3F;
            s[1] = ((b0&0x3)<<4) | (b1>>4);
            s[2] = ((b1&0xF)<<2) | (b2>>6);
            s[3] =  b2 & 0x3F;
            if (i*4+0 < n) out[i*4+0] = (float)(idx_to_float(s[0], max_val));
            if (i*4+1 < n) out[i*4+1] = (float)(idx_to_float(s[1], max_val));
            if (i*4+2 < n) out[i*4+2] = (float)(idx_to_float(s[2], max_val));
            if (i*4+3 < n) out[i*4+3] = (float)(idx_to_float(s[3], max_val));
        }
        return out;

    case 8:
        for (i = 0; i < n; i++)
            out[i] = (float)(idx_to_float(data[i], max_val));
        return out;

    case 10:
        ng = (n + 3) / 4;
        for (i = 0; i < ng; i++) {
            uint32_t b0=data[i*5+0], b1=data[i*5+1], b2=data[i*5+2],
                     b3=data[i*5+3], b4=data[i*5+4];
            uint32_t s[4];
            s[0] = (b0<<2) | (b1>>6);
            s[1] = ((b1&0x3F)<<4) | (b2>>4);
            s[2] = ((b2&0x0F)<<6) | (b3>>2);
            s[3] = ((b3&0x03)<<8) |  b4;
            if (i*4+0 < n) out[i*4+0] = (float)(idx_to_float(s[0], max_val));
            if (i*4+1 < n) out[i*4+1] = (float)(idx_to_float(s[1], max_val));
            if (i*4+2 < n) out[i*4+2] = (float)(idx_to_float(s[2], max_val));
            if (i*4+3 < n) out[i*4+3] = (float)(idx_to_float(s[3], max_val));
        }
        return out;

    case 12:
        ng = (n + 1) / 2;
        for (i = 0; i < ng; i++) {
            uint32_t b0=data[i*3+0], b1=data[i*3+1], b2=data[i*3+2];
            uint32_t s0 = (b0<<4) | (b1>>4);
            uint32_t s1 = ((b1&0xF)<<8) | b2;
            if (i*2+0 < n) out[i*2+0] = (float)(idx_to_float(s0, max_val));
            if (i*2+1 < n) out[i*2+1] = (float)(idx_to_float(s1, max_val));
        }
        return out;

    case 14:
        ng = (n + 3) / 4;
        for (i = 0; i < ng; i++) {
            const uint8_t *p = data + i*7;
            uint32_t s[4];
            s[0] = ((uint32_t)p[0]<<6) | (p[1]>>2);
            s[1] = ((uint32_t)(p[1]&0x03)<<12) | ((uint32_t)p[2]<<4) | (p[3]>>4);
            s[2] = ((uint32_t)(p[3]&0x0F)<<10) | ((uint32_t)p[4]<<2) | (p[5]>>6);
            s[3] = ((uint32_t)(p[5]&0x3F)<<8)  |  p[6];
            if (i*4+0 < n) out[i*4+0] = (float)(idx_to_float(s[0], max_val));
            if (i*4+1 < n) out[i*4+1] = (float)(idx_to_float(s[1], max_val));
            if (i*4+2 < n) out[i*4+2] = (float)(idx_to_float(s[2], max_val));
            if (i*4+3 < n) out[i*4+3] = (float)(idx_to_float(s[3], max_val));
        }
        return out;

    case 16:
        for (i = 0; i < n; i++) {
            int16_t v = (int16_t)(data[i*2+0] | ((uint16_t)data[i*2+1] << 8));
            out[i] = (float)v / 32768.0f;
        }
        return out;

    case 18:
        ng = (n + 3) / 4;
        for (i = 0; i < ng; i++) {
            const uint8_t *p = data + i*9;
            uint32_t s[4];
            s[0] = ((uint32_t)p[0]<<10) | ((uint32_t)p[1]<<2) | (p[2]>>6);
            s[1] = ((uint32_t)(p[2]&0x3F)<<12) | ((uint32_t)p[3]<<4) | (p[4]>>4);
            s[2] = ((uint32_t)(p[4]&0x0F)<<14) | ((uint32_t)p[5]<<6) | (p[6]>>2);
            s[3] = ((uint32_t)(p[6]&0x03)<<16) | ((uint32_t)p[7]<<8) |  p[8];
            if (i*4+0 < n) out[i*4+0] = (float)(idx_to_float(s[0], max_val));
            if (i*4+1 < n) out[i*4+1] = (float)(idx_to_float(s[1], max_val));
            if (i*4+2 < n) out[i*4+2] = (float)(idx_to_float(s[2], max_val));
            if (i*4+3 < n) out[i*4+3] = (float)(idx_to_float(s[3], max_val));
        }
        return out;

    case 20:
        ng = (n + 1) / 2;
        for (i = 0; i < ng; i++) {
            const uint8_t *p = data + i*5;
            uint32_t s0 = ((uint32_t)p[0]<<12) | ((uint32_t)p[1]<<4) | (p[2]>>4);
            uint32_t s1 = ((uint32_t)(p[2]&0xF)<<16) | ((uint32_t)p[3]<<8) | p[4];
            if (i*2+0 < n) out[i*2+0] = (float)(idx_to_float(s0, max_val));
            if (i*2+1 < n) out[i*2+1] = (float)(idx_to_float(s1, max_val));
        }
        return out;

    case 22:
        ng = (n + 3) / 4;
        for (i = 0; i < ng; i++) {
            const uint8_t *p = data + i*11;
            uint32_t s[4];
            s[0] = ((uint32_t)p[0]<<14) | ((uint32_t)p[1]<<6) | (p[2]>>2);
            s[1] = ((uint32_t)(p[2]&0x03)<<20) | ((uint32_t)p[3]<<12) | ((uint32_t)p[4]<<4) | (p[5]>>4);
            s[2] = ((uint32_t)(p[5]&0x0F)<<18) | ((uint32_t)p[6]<<10) | ((uint32_t)p[7]<<2) | (p[8]>>6);
            s[3] = ((uint32_t)(p[8]&0x3F)<<16) | ((uint32_t)p[9]<<8)  |  p[10];
            if (i*4+0 < n) out[i*4+0] = (float)(idx_to_float(s[0], max_val));
            if (i*4+1 < n) out[i*4+1] = (float)(idx_to_float(s[1], max_val));
            if (i*4+2 < n) out[i*4+2] = (float)(idx_to_float(s[2], max_val));
            if (i*4+3 < n) out[i*4+3] = (float)(idx_to_float(s[3], max_val));
        }
        return out;

    case 24:
        for (i = 0; i < n; i++) {
            uint32_t v = ((uint32_t)data[i*3+0]<<16)
                       | ((uint32_t)data[i*3+1]<<8)
                       |  (uint32_t)data[i*3+2];
            out[i] = (float)(idx_to_float(v, max_val));
        }
        return out;

    case 26:
        ng = (n + 3) / 4;
        for (i = 0; i < ng; i++) {
            const uint8_t *p = data + i*13;
            uint32_t s[4];
            s[0] = ((uint32_t)p[0]<<18) | ((uint32_t)p[1]<<10) | ((uint32_t)p[2]<<2) | (p[3]>>6);
            s[1] = ((uint32_t)(p[3]&0x3F)<<20) | ((uint32_t)p[4]<<12) | ((uint32_t)p[5]<<4) | (p[6]>>4);
            s[2] = ((uint32_t)(p[6]&0x0F)<<22) | ((uint32_t)p[7]<<14) | ((uint32_t)p[8]<<6) | (p[9]>>2);
            s[3] = ((uint32_t)(p[9]&0x03)<<24) | ((uint32_t)p[10]<<16) | ((uint32_t)p[11]<<8) | p[12];
            if (i*4+0 < n) out[i*4+0] = (float)(idx_to_float(s[0], max_val));
            if (i*4+1 < n) out[i*4+1] = (float)(idx_to_float(s[1], max_val));
            if (i*4+2 < n) out[i*4+2] = (float)(idx_to_float(s[2], max_val));
            if (i*4+3 < n) out[i*4+3] = (float)(idx_to_float(s[3], max_val));
        }
        return out;

    case 28:
        ng = (n + 1) / 2;
        for (i = 0; i < ng; i++) {
            const uint8_t *p = data + i*7;
            uint32_t s0 = ((uint32_t)p[0]<<20) | ((uint32_t)p[1]<<12) | ((uint32_t)p[2]<<4) | (p[3]>>4);
            uint32_t s1 = ((uint32_t)(p[3]&0xF)<<24) | ((uint32_t)p[4]<<16) | ((uint32_t)p[5]<<8) | p[6];
            if (i*2+0 < n) out[i*2+0] = (float)(idx_to_float(s0, max_val));
            if (i*2+1 < n) out[i*2+1] = (float)(idx_to_float(s1, max_val));
        }
        return out;

    case 30:
        ng = (n + 3) / 4;
        for (i = 0; i < ng; i++) {
            const uint8_t *p = data + i*15;
            uint32_t s[4];
            s[0] = ((uint32_t)p[0]<<22) | ((uint32_t)p[1]<<14) | ((uint32_t)p[2]<<6) | (p[3]>>2);
            s[1] = ((uint32_t)(p[3]&0x03)<<28) | ((uint32_t)p[4]<<20) | ((uint32_t)p[5]<<12) | ((uint32_t)p[6]<<4) | (p[7]>>4);
            s[2] = ((uint32_t)(p[7]&0x0F)<<26) | ((uint32_t)p[8]<<18) | ((uint32_t)p[9]<<10) | ((uint32_t)p[10]<<2) | (p[11]>>6);
            s[3] = ((uint32_t)(p[11]&0x3F)<<24) | ((uint32_t)p[12]<<16) | ((uint32_t)p[13]<<8) | p[14];
            if (i*4+0 < n) out[i*4+0] = (float)(idx_to_float(s[0], max_val));
            if (i*4+1 < n) out[i*4+1] = (float)(idx_to_float(s[1], max_val));
            if (i*4+2 < n) out[i*4+2] = (float)(idx_to_float(s[2], max_val));
            if (i*4+3 < n) out[i*4+3] = (float)(idx_to_float(s[3], max_val));
        }
        return out;

    case 32:
        for (i = 0; i < n; i++) {
            uint32_t v = ((uint32_t)data[i*4+0]<<24)
                       | ((uint32_t)data[i*4+1]<<16)
                       | ((uint32_t)data[i*4+2]<<8)
                       |  (uint32_t)data[i*4+3];
            out[i] = (float)(idx_to_float(v, max_val));
        }
        return out;

    default:
        fprintf(stderr, "X Unsupported bits-per-sample: %d\n", bps);
        free(out);
        return NULL;
    }
}

static int cfda_read(const char *filepath, CFDAAudio *out) {
    FILE *f;
    uint8_t magic[16], meta[16];
    uint8_t *pcm_data;
    long data_start, data_len;
    uint32_t total_samples;

    f = fopen(filepath, "rb");
    if (!f) { fprintf(stderr, "X Cannot open file: %s\n", filepath); return 0; }

    if (fread(magic, 1, 16, f) != 16 || memcmp(magic, CFDA_MAGIC, 16) != 0) {
        fprintf(stderr, "X Invalid CFDA file: Incorrect header signature.\n");
        fclose(f); return 0;
    }
    if (fread(meta, 1, 16, f) != 16) {
        fprintf(stderr, "X Invalid CFDA file: Incomplete metadata.\n");
        fclose(f); return 0;
    }

    out->num_channels   = read_u16le(meta + 0);
    out->sample_rate    = read_u32le(meta + 2);
    out->byte_rate      = read_u32le(meta + 6);
    out->bits_per_sample= read_u16le(meta + 10);
    out->file_size      = read_u32le(meta + 12);

    data_start = ftell(f);
    fseek(f, 0, SEEK_END);
    data_len = ftell(f) - data_start;
    fseek(f, data_start, SEEK_SET);

    pcm_data = (uint8_t*)malloc((size_t)data_len);
    if (!pcm_data) { fclose(f); return 0; }
    if (fread(pcm_data, 1, (size_t)data_len, f) != (size_t)data_len) {
        fprintf(stderr, "X Read error on PCM data.\n");
        free(pcm_data); fclose(f); return 0;
    }
    fclose(f);

    total_samples = (uint32_t)((unsigned long)data_len * 8 / out->bits_per_sample);
    out->data = unpack_pcm(pcm_data, (uint32_t)data_len,
                           out->bits_per_sample, total_samples);
    free(pcm_data);
    if (!out->data) return 0;

    out->num_frames = (out->num_channels > 0)
                      ? total_samples / out->num_channels
                      : total_samples;
    return 1;
}

static int cfda_write(const char *filepath, const float *data,
                      uint32_t num_frames, int num_channels,
                      uint32_t sample_rate, int bits_per_sample) {
    FILE *f;
    uint8_t *pcm;
    uint32_t pcm_len = 0;
    uint32_t total_samples = num_frames * (uint32_t)num_channels;
    uint32_t byte_rate = (sample_rate * (uint32_t)num_channels * (uint32_t)bits_per_sample + 7) / 8;
    uint32_t file_size;

    pcm = pack_pcm(data, total_samples, bits_per_sample, &pcm_len);
    if (!pcm) return 0;

    file_size = (uint32_t)(HEADER_SIZE + pcm_len) - 33;

    f = fopen(filepath, "wb");
    if (!f) { free(pcm); fprintf(stderr, "X Cannot open output: %s\n", filepath); return 0; }

    fwrite(CFDA_MAGIC, 1, 16, f);
    write_u16le(f, (uint16_t)num_channels);
    write_u32le(f, sample_rate);
    write_u32le(f, byte_rate);
    write_u16le(f, (uint16_t)bits_per_sample);
    write_u32le(f, file_size);
    fwrite(pcm, 1, pcm_len, f);
    fclose(f);
    free(pcm);
    return 1;
}

static int load_wav(const char *path, float **data_out,
                    uint32_t *frames_out, int *ch_out, uint32_t *sr_out) {
    FILE *f;
    uint8_t hdr[44];
    uint16_t audio_fmt, num_ch, bps;
    uint32_t sr, data_size;
    uint8_t *raw;
    float *data;
    uint32_t num_samples, i;

    f = fopen(path, "rb");
    if (!f) { fprintf(stderr, "X Cannot open WAV: %s\n", path); return 0; }

    if (fread(hdr, 1, 44, f) != 44) {
        fprintf(stderr, "X WAV header too short.\n"); fclose(f); return 0;
    }
    if (memcmp(hdr, "RIFF", 4) || memcmp(hdr+8, "WAVEfmt ", 8)) {
        fprintf(stderr, "X Not a valid WAV file.\n"); fclose(f); return 0;
    }
    audio_fmt = read_u16le(hdr + 20);
    if (audio_fmt != 1) {
        fprintf(stderr, "X Only uncompressed PCM WAV supported.\n"); fclose(f); return 0;
    }
    num_ch   = read_u16le(hdr + 22);
    sr       = read_u32le(hdr + 24);
    bps      = read_u16le(hdr + 34);
    data_size= read_u32le(hdr + 40);

    {
        uint32_t fmt_size = read_u32le(hdr + 16);
        long expected_data = 12 + 8 + (long)fmt_size;                      
        uint8_t chunk_id[4];
        uint32_t chunk_sz;
        fseek(f, expected_data, SEEK_SET);
        while (1) {
            if (fread(chunk_id, 1, 4, f) != 4) break;
            if (fread(hdr, 1, 4, f) != 4) break;
            chunk_sz = read_u32le(hdr);
            if (memcmp(chunk_id, "data", 4) == 0) { data_size = chunk_sz; break; }
            fseek(f, (long)chunk_sz, SEEK_CUR);
        }
    }

    raw = (uint8_t*)malloc(data_size);
    if (!raw) { fclose(f); return 0; }
    if (fread(raw, 1, data_size, f) != data_size) {
        fprintf(stderr, "X WAV read error.\n"); free(raw); fclose(f); return 0;
    }
    fclose(f);

    num_samples = data_size / (bps / 8);
    data = (float*)malloc(num_samples * sizeof(float));
    if (!data) { free(raw); return 0; }

    if (bps == 8) {
        for (i = 0; i < num_samples; i++)
            data[i] = ((float)raw[i] - 128.0f) / 128.0f;
    } else if (bps == 16) {
        for (i = 0; i < num_samples; i++) {
            int16_t v = (int16_t)(raw[i*2] | ((uint16_t)raw[i*2+1] << 8));
            data[i] = (float)v / 32768.0f;
        }
    } else if (bps == 24) {
        for (i = 0; i < num_samples; i++) {
            int32_t v = (int32_t)raw[i*3]
                      | ((int32_t)raw[i*3+1] << 8)
                      | ((int32_t)raw[i*3+2] << 16);
            if (v & 0x800000) v |= (int32_t)0xFF000000;                  
            data[i] = (float)v / 8388608.0f;
        }
    } else if (bps == 32) {
        for (i = 0; i < num_samples; i++) {
            int32_t v = (int32_t)((uint32_t)raw[i*4]
                      | ((uint32_t)raw[i*4+1] << 8)
                      | ((uint32_t)raw[i*4+2] << 16)
                      | ((uint32_t)raw[i*4+3] << 24));
            data[i] = (float)v / 2147483648.0f;
        }
    } else {
        fprintf(stderr, "X Unsupported WAV bps: %d\n", bps);
        free(raw); free(data); return 0;
    }
    free(raw);

    *data_out  = data;
    *frames_out= num_samples / num_ch;
    *ch_out    = num_ch;
    *sr_out    = sr;
    return 1;
}

static int write_wav(const char *path, const float *data,
                     uint32_t num_frames, int num_ch,
                     uint32_t sr, int bps) {
    FILE *f;
    uint32_t i, total = num_frames * (uint32_t)num_ch;
    uint32_t bytes_per_sample = (uint32_t)bps / 8;
    uint32_t data_size = total * bytes_per_sample;
    uint32_t byte_rate = sr * (uint32_t)num_ch * bytes_per_sample;
    uint16_t block_align = (uint16_t)((uint32_t)num_ch * bytes_per_sample);

    f = fopen(path, "wb");
    if (!f) { fprintf(stderr, "X Cannot open output WAV: %s\n", path); return 0; }

    fwrite("RIFF", 1, 4, f);
    write_u32le(f, 36 + data_size);
    fwrite("WAVEfmt ", 1, 8, f);
    write_u32le(f, 16);                                
    write_u16le(f, 1);                      
    write_u16le(f, (uint16_t)num_ch);
    write_u32le(f, sr);
    write_u32le(f, byte_rate);
    write_u16le(f, block_align);
    write_u16le(f, (uint16_t)bps);
    fwrite("data", 1, 4, f);
    write_u32le(f, data_size);

    if (bps == 8) {
        for (i = 0; i < total; i++) {
            int v = (int)(data[i] * 128.0f + 128.5f);
            if (v < 0)   v = 0;
            if (v > 255) v = 255;
            fputc(v, f);
        }
    } else if (bps == 16) {
        for (i = 0; i < total; i++) {
            int v = (int)(data[i] * 32768.0f);
            if (v < -32768) v = -32768;
            if (v >  32767) v =  32767;
            write_u16le(f, (uint16_t)(int16_t)v);
        }
    } else if (bps == 24) {
        for (i = 0; i < total; i++) {
            int32_t v = (int32_t)(data[i] * 8388608.0f);
            uint32_t u;
            if (v < -8388608) v = -8388608;
            if (v >  8388607) v =  8388607;
            u = (uint32_t)v;
            fputc((int)(u&0xFF),f); fputc((int)((u>>8)&0xFF),f); fputc((int)((u>>16)&0xFF),f);
        }
    } else if (bps == 32) {
        for (i = 0; i < total; i++) {
            double dv = (double)data[i] * 2147483648.0;
            int32_t v;
            if (dv >= 2147483647.0) v = 2147483647;
            else if (dv < -2147483648.0) v = (int32_t)0x80000000;
            else v = (int32_t)dv;
            {
                uint32_t u = (uint32_t)v;
                fputc(u&0xFF,f); fputc((u>>8)&0xFF,f);
                fputc((u>>16)&0xFF,f); fputc((u>>24)&0xFF,f);
            }
        }
    } else {
        fclose(f);
        fprintf(stderr, "X Unsupported target bps: %d\n", bps);
        return 0;
    }
    fclose(f);
    return 1;
}

static int load_raw(const char *path, float **data_out, uint32_t *frames_out,
                    int *ch_out, uint32_t sr, int ch, int bps,
                    const char *endian) {
    FILE *f;
    long file_size;
    uint8_t *raw;
    uint32_t num_samples, i;
    float *data;
    int big = (strcmp(endian, "big") == 0);

    f = fopen(path, "rb");
    if (!f) { fprintf(stderr, "X Cannot open RAW: %s\n", path); return 0; }
    fseek(f, 0, SEEK_END); file_size = ftell(f); fseek(f, 0, SEEK_SET);
    raw = (uint8_t*)malloc((size_t)file_size);
    if (!raw) { fclose(f); return 0; }
    fread(raw, 1, (size_t)file_size, f);
    fclose(f);

    if (bps != 8 && bps != 16 && bps != 24 && bps != 32) {
        fprintf(stderr, "X RAW conversion supports 8, 16, 24, or 32 bps.\n");
        free(raw); return 0;
    }

    num_samples = (uint32_t)file_size / (uint32_t)(bps / 8);
    data = (float*)malloc(num_samples * sizeof(float));
    if (!data) { free(raw); return 0; }

    if (bps == 8) {
        for (i = 0; i < num_samples; i++)
            data[i] = ((float)raw[i] - 128.0f) / 128.0f;
    } else if (bps == 16) {
        for (i = 0; i < num_samples; i++) {
            int16_t v;
            if (big) v = (int16_t)(((uint16_t)raw[i*2] << 8) | raw[i*2+1]);
            else     v = (int16_t)(raw[i*2] | ((uint16_t)raw[i*2+1] << 8));
            data[i] = (float)v / 32768.0f;
        }
    } else if (bps == 24) {
        for (i = 0; i < num_samples; i++) {
            int32_t v;
            if (big) v = ((int32_t)raw[i*3]<<16)|((int32_t)raw[i*3+1]<<8)|raw[i*3+2];
            else     v = (int32_t)raw[i*3]|((int32_t)raw[i*3+1]<<8)|((int32_t)raw[i*3+2]<<16);
            if (v & 0x800000) v |= (int32_t)0xFF000000;
            data[i] = (float)v / 8388608.0f;
        }
    } else {
        for (i = 0; i < num_samples; i++) {
            int32_t v;
            if (big) v = (int32_t)(((uint32_t)raw[i*4]<<24)|((uint32_t)raw[i*4+1]<<16)|((uint32_t)raw[i*4+2]<<8)|raw[i*4+3]);
            else     v = (int32_t)((uint32_t)raw[i*4]|((uint32_t)raw[i*4+1]<<8)|((uint32_t)raw[i*4+2]<<16)|((uint32_t)raw[i*4+3]<<24));
            data[i] = (float)v / 2147483648.0f;
        }
    }
    free(raw);
    *data_out  = data;
    *frames_out= num_samples / (uint32_t)ch;
    *ch_out    = ch;
    (void)sr;
    return 1;
}


static void cmd_play(const char *filepath) {
    CFDAAudio cfda;
    char out_path[4096];
    const char *dot;
    int ok;

    memset(&cfda, 0, sizeof(cfda));
    if (!cfda_read(filepath, &cfda)) return;

    strncpy(out_path, filepath, sizeof(out_path) - 5);
    out_path[sizeof(out_path) - 5] = '\0';
    dot = strrchr(out_path, '.');
    if (dot) out_path[dot - out_path] = '\0';
    strcat(out_path, ".wav");

    printf("> Decoding for playback: %s\n", filepath);
    printf("  Channels   : %u\n",   (unsigned)cfda.num_channels);
    printf("  Sample Rate: %lu Hz\n", (unsigned long)cfda.sample_rate);
    printf("  BPS        : %u\n",   (unsigned)cfda.bits_per_sample);
    printf("> Writing decoded WAV to: %s\n", out_path);

    ok = write_wav(out_path, cfda.data, cfda.num_frames,
                   cfda.num_channels, cfda.sample_rate, 16);
    if (ok)
        printf("! Done. Open %s with any media player.\n", out_path);
    cfda_free(&cfda);
}

static void cmd_create(const char *outfile, double duration, double freq,
                       uint32_t sr, int channels, int bps) {
    uint32_t num_frames = (uint32_t)(sr * duration);
    uint32_t total      = num_frames * (uint32_t)channels;
    float *audio;
    uint32_t i;

    audio = (float*)malloc(total * sizeof(float));
    if (!audio) { fprintf(stderr, "X Out of memory.\n"); return; }

    for (i = 0; i < num_frames; i++) {
        float s = (float)(0.9 * sin(2.0 * M_PI * freq * (double)i / (double)sr));
        audio[i * channels + 0] = s;
        if (channels == 2) audio[i * channels + 1] = s;
    }

    if (cfda_write(outfile, audio, num_frames, channels, sr, bps))
        printf("! Created: %s\n", outfile);
    free(audio);
}

static void cmd_convert(const char *infile, const char *outfile,
                        const char *fmt, int channels, int bps,
                        uint32_t raw_sr, int raw_ch, int raw_bps,
                        const char *raw_endian) {
    float *audio = NULL;
    uint32_t frames = 0;
    int ch = 0;
    uint32_t sr = 0;
    int ok;

    if (strcmp(fmt, "wav") == 0) {
        ok = load_wav(infile, &audio, &frames, &ch, &sr);
    } else {
        sr = raw_sr; ch = raw_ch;
        ok = load_raw(infile, &audio, &frames, &ch, sr, ch, raw_bps, raw_endian);
    }
    if (!ok) return;

    if (channels && channels != ch) {
        if (channels == 1 && ch == 2) {
            uint32_t i;
            for (i = 0; i < frames; i++)
                audio[i] = (audio[i*2] + audio[i*2+1]) * 0.5f;
            ch = 1;
            printf(">< Downmixed stereo to mono\n");
        } else {
            printf("?  Cannot change channels from %d to %d. Skipping.\n", ch, channels);
        }
    }

    if (cfda_write(outfile, audio, frames, ch, sr, bps))
        printf("! Converted: %s -> %s (%dbps, %dch)\n", infile, outfile, bps, ch);
    free(audio);
}

static void cmd_decode(const char *infile, const char *outfile,
                       const char *fmt, int target_bps, int passthrough) {
    CFDAAudio cfda;
    memset(&cfda, 0, sizeof(cfda));

    if (passthrough) {
        FILE *fin, *fout;
        uint8_t buf[4096];
        size_t n;
        if (strcmp(fmt, "raw") != 0) {
            fprintf(stderr, "X --passthrough only works with --format raw\n"); return;
        }
        printf("@ Raw passthrough: Stripping %d-byte header...\n", HEADER_SIZE);
        fin = fopen(infile, "rb");
        if (!fin) { fprintf(stderr, "X Cannot open: %s\n", infile); return; }
        fseek(fin, HEADER_SIZE, SEEK_SET);
        fout = fopen(outfile, "wb");
        if (!fout) { fclose(fin); fprintf(stderr, "X Cannot open output: %s\n", outfile); return; }
        while ((n = fread(buf, 1, sizeof(buf), fin)) > 0)
            fwrite(buf, 1, n, fout);
        fclose(fin); fclose(fout);
        printf("! Decoded: %s -> %s (exact bit-packed stream)\n", infile, outfile);
        return;
    }

    if (!cfda_read(infile, &cfda)) return;

    if (target_bps == 0) {
        int bps = cfda.bits_per_sample;
        target_bps = (bps==8||bps==16||bps==24||bps==32) ? bps : 16;
    }

    printf("@ Decoding: %ubps CFDA -> %dbps %s...\n",
           (unsigned)cfda.bits_per_sample, target_bps, fmt);

    if (strcmp(fmt, "wav") == 0) {
        if (write_wav(outfile, cfda.data, cfda.num_frames,
                      cfda.num_channels, cfda.sample_rate, target_bps))
            printf("! Decoded: %s -> %s (%dbps, %uch, %luHz)\n",
                   infile, outfile, target_bps,
                   (unsigned)cfda.num_channels, (unsigned long)cfda.sample_rate);
    } else {
        FILE *fout;
        uint32_t total = cfda.num_frames * cfda.num_channels;
        uint32_t i;
        fout = fopen(outfile, "wb");
        if (!fout) { fprintf(stderr, "X Cannot open output: %s\n", outfile); cfda_free(&cfda); return; }

        if (target_bps == 8) {
            for (i = 0; i < total; i++) {
                int v = (int)(cfda.data[i] * 128.0f + 128.5f);
                if (v < 0) v = 0;
                if (v > 255) v = 255;
                fputc(v, fout);
            }
        } else if (target_bps == 16) {
            for (i = 0; i < total; i++) {
                int v = (int)(cfda.data[i] * 32768.0f);
                if (v < -32768) v = -32768;
                if (v > 32767) v = 32767;
                fputc(v&0xFF, fout); fputc((v>>8)&0xFF, fout);
            }
        } else if (target_bps == 24) {
            for (i = 0; i < total; i++) {
                int32_t v = (int32_t)(cfda.data[i] * 8388608.0f);
                uint32_t u;
                if (v < -8388608) v = -8388608;
                if (v >  8388607) v =  8388607;
                u = (uint32_t)v;
                fputc((int)(u&0xFF),fout); fputc((int)((u>>8)&0xFF),fout); fputc((int)((u>>16)&0xFF),fout);
            }
        } else if (target_bps == 32) {
            for (i = 0; i < total; i++) {
                double dv = (double)cfda.data[i] * 2147483648.0;
                int32_t v;
                uint32_t u;
                if (dv >= 2147483647.0) v = 2147483647;
                else if (dv < -2147483648.0) v = (int32_t)0x80000000;
                else v = (int32_t)dv;
                u = (uint32_t)v;
                fputc((int)(u&0xFF),fout); fputc((int)((u>>8)&0xFF),fout);
                fputc((int)((u>>16)&0xFF),fout); fputc((int)((u>>24)&0xFF),fout);
            }
        }
        fclose(fout);
        printf("! Decoded: %s -> %s (%dbps, %uch, %luHz)\n",
               infile, outfile, target_bps,
               (unsigned)cfda.num_channels, (unsigned long)cfda.sample_rate);
    }
    cfda_free(&cfda);
}

#define starts_with(s, prefix) (strncmp((s),(prefix),strlen(prefix))==0)
#define arg_val(s, prefix) (starts_with((s),(prefix)) ? (s)+strlen(prefix) : NULL)
static int find_arg(int argc, char **argv, const char *prefix, const char **val) {
    size_t plen = strlen(prefix);
    int i;
    for (i = 0; i < argc; i++) {
        if (strncmp(argv[i], prefix, plen) == 0) {
            if (val) *val = argv[i] + plen;
            return i;
        }
    }
    return -1;
}

static void print_usage(const char *prog) {
    fprintf(stderr, "CFDA Audio Format Tool (C89 port)\n\n");
    fprintf(stderr, "Usage:\n");
    fprintf(stderr, "  %s play   <file.cfda>\n", prog);
    fprintf(stderr, "      Decodes CFDA to a WAV file beside the source for playback.\n\n");
    
    fprintf(stderr, "  %s create <file.cfda> [--duration=<s>] [--freq=<hz>] [--sr=<hz>]\n", prog);
    fprintf(stderr, "                        [--channels=<1|2>] [--bps=<1..32>]\n\n");
    
    fprintf(stderr, "  %s convert <input> <output.cfda> --format=<wav|raw>\n", prog);
    fprintf(stderr, "             [--channels=<1|2>] [--bps=<1..32>]\n");
    fprintf(stderr, "             [--raw-sr=<hz>] [--raw-ch=<n>]\n");
    fprintf(stderr, "             [--raw-bps=<8|16|24|32>]\n");
    fprintf(stderr, "             [--raw-endian=<little|big>]\n\n");
    
    fprintf(stderr, "  %s decode  <file.cfda> <output> --format=<wav|raw>\n", prog);
    fprintf(stderr, "             [--target-bps=<8|16|24|32>]\n");
    fprintf(stderr, "             [--passthrough]\n");
}

int main(int argc, char **argv) {
    const char *cmd, *v;

    if (argc < 2) { print_usage(argv[0]); return 1; }
    cmd = argv[1];

    if (strcmp(cmd, "play") == 0) {
        if (argc < 3) { fprintf(stderr, "Usage: %s play <file.cfda>\n", argv[0]); return 1; }
        cmd_play(argv[2]);

    } else if (strcmp(cmd, "create") == 0) {
        const char *outfile;
        double duration = 2.0, freq = 440.0;
        uint32_t sr = 44100;
        int channels = 2, bps = 16;

        if (argc < 3) { print_usage(argv[0]); return 1; }
        outfile = argv[2];
        if (find_arg(argc-3, argv+3, "--duration=", &v) >= 0) duration = atof(v);
        if (find_arg(argc-3, argv+3, "--freq=",     &v) >= 0) freq     = atof(v);
        if (find_arg(argc-3, argv+3, "--sr=",       &v) >= 0) sr       = (uint32_t)atol(v);
        if (find_arg(argc-3, argv+3, "--channels=", &v) >= 0) channels = atoi(v);
        if (find_arg(argc-3, argv+3, "--bps=",      &v) >= 0) bps      = atoi(v);
        cmd_create(outfile, duration, freq, sr, channels, bps);

    } else if (strcmp(cmd, "convert") == 0) {
        const char *infile, *outfile, *fmt = NULL;
        int channels = 0, bps = 16, raw_ch = 1, raw_bps = 16;
        uint32_t raw_sr = 44100;
        const char *raw_endian = "little";

        if (argc < 4) { print_usage(argv[0]); return 1; }
        infile  = argv[2];
        outfile = argv[3];
        if (find_arg(argc-4, argv+4, "--format=",     &v) >= 0) fmt = v;
        if (find_arg(argc-4, argv+4, "--channels=",   &v) >= 0) channels  = atoi(v);
        if (find_arg(argc-4, argv+4, "--bps=",        &v) >= 0) bps       = atoi(v);
        if (find_arg(argc-4, argv+4, "--raw-sr=",     &v) >= 0) raw_sr    = (uint32_t)atol(v);
        if (find_arg(argc-4, argv+4, "--raw-ch=",     &v) >= 0) raw_ch    = atoi(v);
        if (find_arg(argc-4, argv+4, "--raw-bps=",    &v) >= 0) raw_bps   = atoi(v);
        if (find_arg(argc-4, argv+4, "--raw-endian=", &v) >= 0) raw_endian= v;
        if (!fmt) { fprintf(stderr, "X --format=<wav|raw> required\n"); return 1; }
        cmd_convert(infile, outfile, fmt, channels, bps, raw_sr, raw_ch, raw_bps, raw_endian);

    } else if (strcmp(cmd, "decode") == 0) {
        const char *infile, *outfile, *fmt = NULL;
        int target_bps = 0, passthrough = 0;

        if (argc < 4) { print_usage(argv[0]); return 1; }
        infile  = argv[2];
        outfile = argv[3];
        if (find_arg(argc-4, argv+4, "--format=",     &v) >= 0) fmt        = v;
        if (find_arg(argc-4, argv+4, "--target-bps=", &v) >= 0) target_bps = atoi(v);
        if (find_arg(argc-4, argv+4, "--passthrough",  NULL) >= 0) passthrough = 1;
        if (!fmt) { fprintf(stderr, "X --format=<wav|raw> required\n"); return 1; }
        cmd_decode(infile, outfile, fmt, target_bps, passthrough);

    } else {
        fprintf(stderr, "X Unknown command: %s\n", cmd);
        print_usage(argv[0]);
        return 1;
    }
    return 0;
}
