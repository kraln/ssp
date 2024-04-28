Shooting Stand Peeper
================

This is the hardware and software for a device which can be used to time various sporting activities, originally developed to be loud enough to use for shooting competitions. 

Hardware
---------
The hardware is based on an Arduino Nano 33 IoT, which provides the WIFI access point as well as the DAC used to drive the speaker. 

Additionally, a battery-charging circuit as well as an amplifier have been added, so that the tone is loud enough. 

The tone is emitted by a Piezoelectric tweeter, which offers an excellent tradeoff of price vs performance. 

The enclosure is 3d printed, and optionally finished with a lovely acoustic nylon cloth.

Firmware
---------
The firmware has three main functions:
* Setting up the WIFI access point
* Serving the control interface
* Emitting tones

Driving of the DAC is done directly, as the Arduino libraries cause issues. 

Control
---------
To keep the project as simple as possible, there is no App required for use. The device is controlled over WIFI, where it appears as an access point with no access control. Once connected, direct a browser to 10.10.10.10 and the interface will appear. Here, you can manage routines ("Abläufe"), select which one to run, and start/stop them.
