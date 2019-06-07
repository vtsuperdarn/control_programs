Radar Control Program Name:
==========================
iwdscan

Control Program ID (CPID):
=========================
3385

Parameters:
==========
nbeams: 9
intt: 1 s
scan: 10 s
ngates: 75+
frang: 180 km
rsep: 45 km

Description:
===========
iwdscan was created for the Special Time request by Gareth Perry to
run during the incoherent scatter radar (ISR) World Day campaign.
It was adopted from epopsound.1.02. There are two main ways to operate
this mode - one is in a fixed frequency mode to support ePOP
experiments while the Cassiope satellite is in the radar field of view.
The fixed frequency mode does not use the clear frequency search algorithm.
The second mode is where the frequency noise search algorithm is used
to select the frequency with the lowest noise in a 300 kHz band of the
requested frequency. The mode has 1 second integration time, selectable
9 beams to overlap the ISR field of view, and 10 second scan time.

Source:
======
K. Krieger (20160916)
