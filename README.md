#A 4x4x4 LED CUBE

Hardware is based on the 8x8x8 RGB LED cube by Kevin Darrah,
I've been watching all his great tutorials/video's o youtube.

Also the software is based on his Arduino sketch but it is rewritten
to C (my first C project, yeah!) for AVR (also my first AVR project, yeah
again!!).
I'm not using a SPI library anymore, using different timers and (u)int8_t as much
as possible. This reduced the size of the hex file from 18563 bytes to 11056 bytes.

The rest of my choises are explained in the comments in the code.

Great reads and tutorials:
www.kevindarrah.com
www.jumptuck.com
