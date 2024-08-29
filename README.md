This repository is a fork of the RadioHead library created by Mike McCauley (mikem@airspayce.com) https://www.airspayce.com/mikem/arduino/RadioHead/.

It contains an additional implementation for the HopeRF RFM9x boards which provides FSK/GFSK/OOK capabilities. The driver class to use for such cases is RH_RF95_FSKOOK.

Transmission of packets works quite well. Transmissions have been verified using SDR software and RTL_433 which is able to decode properly custom transmission payloads.

The reception does not work yet properly (interrupts do not fire for some unknown reason).

Code comes without any warranties. Use at your own risk. Respect the license and copyrights when using it.
