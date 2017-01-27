# 6lbr-on-Telecontrolli-Devices
Contiki is an open source Operating System that connects tiny low-power microcontrollers to the Internet and allows to develope a wild range of applications (which have an efficient use in the Hardware), providing powerful low-power wireless communication. 
Telecontrolli srl developed a Contiki-based application for its own Smart Devices (CC1310-based):
- X.IP4T SmartModule
- X.IP5 SmartMachine

This application v1.0 enables communication for sensors reading and Devices activation through 6lowpan/CoAP protocols between  a “6lbr-Slip-radio” (i.e. a router device) and a “cc26xx-web-demo” device.
This Repository contains all files to operate and build the firmware for X.IP4T and X.IP5 devices.

Devices setting Procedure:

1.	Download Contiki OS
2.	Download “6lbr-on-Telecontrolli-Devices-XIP” from this repository
3.	Replace the folder “cc26xx” in “home/pi/contiki/example/cc26xx” with the folder “cc26xx” you find in “6lbr-on-Telecontrolli-Devices-XIP”
4.	Replace the folder “dev” in “home/pi/contiki/core/dev” with the folder “dev” you find in “6lbr-on-Telecontrolli-Devices-XIP”
5.	Replace the folders “slip-radio” and “rpl-border-router” in “home/pi/contiki/examples/ipv6” with the folders “slip-radio” and “rpl-border-router” you find in “6lbr-on-Telecontrolli-Devices-XIP”
6.	Replace the folder “srf06” in “home/pi/contiki/platform/srf06-cc26xx/srf06” with the folder “srf06”  you find in “6lbr-on-Telecontrolli-Devices-XIP”. This folder includes “board.h”.

Find below the X.IP4T e X.IP5 datasheetX.IP4T
-	X.ip4T SmartModule
 http://www.telecontrolli.com/resource/downloads/datasheet/iot-system/x-ip-transceiver-family/81-x-   ip4t-datasheet/file.html
-	X.ip5 SmartMachine
http://www.telecontrolli.com/resource/downloads/datasheet/iot-system/x-ip-transceiver-family/82-x-ip5-datasheet/file.html

