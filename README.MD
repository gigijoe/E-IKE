# Enhanced IKE - DME/EGS data stream displayed in the instrument cluster

## Supports

* DME : Bosch Motronic v7.2 (M62TU) - KWP2000 protocol
* EGS : Automatic Transmission ZF5HP24 v8.60.2 (E38/E39/E53)

## Functions

@ Display the following information
1. Coolant temperature.
2. Coolant outlet temperature.
3. Transmission temperature.
4. Intake air temperature.
5. Intake air mass flow.
6. Battery voltage.
7. Engine RPM.
8. Engine load.
9. Fuel trim additive.
10. Fuel trum multiplicative.
11. Fuel Injection time
12. CAM position
13. Engine smoothness

@ Display switch by steering wheel buttons NEXT / PREV or RT_TELEPHONE.

@ Data request interval 2 secs.

@ Automatic external scanner collision detection.

## Principle

Basiclly, the project is a small OBDII scanner that requesting information from DME and EGS.
It reads data from D-Bus then write display string to I-Bus. There are 4 wires needed to connect to wire harness of IKE (12V / GND / D-BUS / I-BUS)

## Hardware

Base on STM32F103C8T6 (20k RAM / 64k FLASH)

## Software

All bugs are made by my self @@

## Referance

http://eurokclub.bike/index.php?topic=2734.0

http://lb9mg.no/2018/01/02/reading-obd2-data-without-elm327-part-2-k-line/

https://github.com/arturlangner/Open-OBD2-datalogger/blob/master/software/obdlogger/Sources/obd/obd_k_line.c

https://github.com/inferiorhumanorgans/DS2PlusPlus/blob/master/dpp-json/dme/me-7.2/dme_me7_2.json

https://github.com/inferiorhumanorgans/DS2PlusPlus/blob/master/dpp-json/egs/egs-8.60.2/egs_8_60_2.json

https://github.com/gigijoe/pBmwScanner
