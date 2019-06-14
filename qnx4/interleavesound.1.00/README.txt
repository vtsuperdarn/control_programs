Radar Control Program Name:
==========================
interleavesound

Control Program ID (CPID):
=========================
195/197

Parameters:
==========
nbeams: 16+
intt: 3 s
scan: 1 min
ngates: 75+
frang: 180 km
rsep: 45 km

Description:
===========
interleavesound is a variant on the interleaved_normalscan and
normalsound radar control programs. interleavesound performs
a scan in a nonsequential manner by "interleaving" the beam
number, e.g. (0-4-8-12)-(2-6-10-14)-(1-5-9-13)-(3-7-11-15).
In the remaining time until the end of the minute it performs
scans through a set of up to 12 frequencies and through all
beams [even/odd]. Note that unlike normalsound, this information
is not used to adjust the radar operating frequency in real-time.

The radar-specific sounder.dat file should contain the
following values (one per line):
Number of sounder frequencies
The sounder frequencies [kHz]

If this file does not exist, default values are used. This
is not a good idea, as the program may try to sound at
forbidden frequencies.

The sounding data are written to *.snd files, for each beam
one header and data record for each good [qflg=1] range.

Source:
======
E.G. Thomas (20190614)
