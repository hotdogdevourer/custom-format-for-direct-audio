# Overview
- Cool audio codec :D
- Compiles with a C89 compiler or higher

# Compile
For C89: `cc -ansi -o cfda cfda.c`

# Usage
Example:

`cfda create output.cfda` This will create:
- 2 second sine wave at 440 hz
- 44100 sample rate
- 16 bits per sample
- Stereo

Can be played (Aka converted to WAV) by `cfda play output.cfda`

# License
MIT License. More licensing information at the LICENSE file in the root directory.
