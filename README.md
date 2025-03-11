
# LMIC-node-ESP32 & RFM95-GPS Tracker
 
This repository is a fork of [roelwolf/LMIC-node-gps-tracker](https://github.com/roelwolf/LMIC-node-gps-tracker) and adds a gps functionality to the default LMIC-node

The code makes use of the library [mikalhart/TinyGPSPlus](https://github.com/mikalhart/TinyGPSPlus)

The code was originally written for the [TTGO T-Beam](https://github.com/LilyGO/TTGO-T-Beam) but has been adapted for any ESP32 & RFM95 Lora Module connected via SPI.

I changed as less as posible to the original code in ```LMIC-node.h``` and ```LMIC-node.cpp```. The additions are mainly between the lines `"USER CODE BEGIN"` and `"USER CODE END"`

It is possible to change the uplink interval by sending a downlink command on channel 100 to the node. There a 4 possible intervals: 0c1E (30 sec), 0x3C (60 sec), 0x78 (120 sec) and 0xB4 (180 sec). When needed, change this in the code of LMIC-node.cpp.

I also added the files gps.h and gps.cpp that I borrowed from [sbiermann Lora-TTNMapper-ESP32](https://github.com/DeuxVis/Lora-TTNMapper-T-Beam) with a few changes.

In platfomio.ini the are also some additions:
Added some new variables:
- HOME_LATITUDE, HOME_LONGITUDE Here you can fill in the coordinates of your starting point. Used i.c.w. MIN_DISTANCE. 
- MIN_DISTANCE meters around your starting point where the mapper is not allowed to upload packages.
- GPS_DEVIATION To ignore small displacements, send only packages when the tracker moves.
- GPS_TX_PIN, GPS_RX_PIN The pin numbers of  the gps module. Depends on the board type.
- a payload formatter for version 3.

Additional Features Added:
- Battery Monitoring via ADC1_CHANNEL_3, ADC_ATTEN_DB_11 (GPIO 39) with vBatt added to the Lora Payload.
- Dual Uplink Intervals: The device can send data at two different intervals (normal and long).Both Intervals are set via platfomio.ini
- DO_WORK_INTERVAL_SECONDS is the normal Lora interval (triggers processWork).
- DO_WORK_INTERVAL_LONG_SECONDS Is a longer interval triggers processWorkLong it Ignores MIN_DISTANCE & GPS_DEVIATION!!! (A heartbeat when the tracker is static for long periods)
- Number of Satellites added to Lora Payload

- ESP32 Processor speed reduced (setCpuFrequencyMhz(20);) to improve battery life of tracker.

