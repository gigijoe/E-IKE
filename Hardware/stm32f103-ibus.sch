EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:stm32
LIBS:relays
LIBS:diode
LIBS:stm32f103-ibus-cache
EELAYER 25 0
EELAYER END
$Descr A3 16535 11693
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Text Label 700  4300 0    60   ~ 0
I-Bus
$Comp
L PC817-RESCUE-stm32f103-ibus IC3
U 1 1 59524955
P 2500 4850
F 0 "IC3" H 2290 5040 40  0000 C CNN
F 1 "PC817" H 2650 4660 40  0000 C CNN
F 2 "DIP4" H 2300 4670 30  0000 C CIN
F 3 "" H 2500 4850 60  0000 C CNN
	1    2500 4850
	1    0    0    -1  
$EndComp
$Comp
L PC817-RESCUE-stm32f103-ibus IC1
U 1 1 59524976
P 2050 5850
F 0 "IC1" H 1840 6040 40  0000 C CNN
F 1 "PC817" H 2200 5660 40  0000 C CNN
F 2 "DIP4" H 1850 5670 30  0000 C CIN
F 3 "" H 2050 5850 60  0000 C CNN
	1    2050 5850
	-1   0    0    -1  
$EndComp
$Comp
L R-RESCUE-stm32f103-ibus R17
U 1 1 595249B7
P 3450 4950
F 0 "R17" V 3530 4950 40  0000 C CNN
F 1 "330" V 3457 4951 40  0000 C CNN
F 2 "Discret:R3" V 3380 4950 30  0001 C CNN
F 3 "" H 3450 4950 30  0000 C CNN
	1    3450 4950
	0    1    1    0   
$EndComp
Text GLabel 3850 4750 2    60   Output ~ 0
IBUS_RX
$Comp
L GND-RESCUE-stm32f103-ibus #PWR01
U 1 1 59524ADF
P 2150 5100
F 0 "#PWR01" H 2150 5100 30  0001 C CNN
F 1 "GND" H 2150 5030 30  0001 C CNN
F 2 "" H 2150 5100 60  0000 C CNN
F 3 "" H 2150 5100 60  0000 C CNN
	1    2150 5100
	1    0    0    -1  
$EndComp
$Comp
L S8050 Q3
U 1 1 59524B85
P 2850 5950
F 0 "Q3" H 2850 5801 40  0000 R CNN
F 1 "S8050" H 2850 6100 40  0000 R CNN
F 2 "TO_SOT_Packages_THT:TO-92_Inline_Narrow_Oval" H 2750 6052 29  0000 C CNN
F 3 "" H 2850 5950 60  0000 C CNN
	1    2850 5950
	-1   0    0    -1  
$EndComp
$Comp
L R-RESCUE-stm32f103-ibus R13
U 1 1 59524C83
P 3300 5750
F 0 "R13" V 3380 5750 40  0000 C CNN
F 1 "330" V 3307 5751 40  0000 C CNN
F 2 "Discret:R3" V 3230 5750 30  0001 C CNN
F 3 "" H 3300 5750 30  0000 C CNN
	1    3300 5750
	0    1    1    0   
$EndComp
$Comp
L R-RESCUE-stm32f103-ibus R14
U 1 1 59524D2E
P 3300 6250
F 0 "R14" V 3380 6250 40  0000 C CNN
F 1 "10K" V 3307 6251 40  0000 C CNN
F 2 "Discret:R3" V 3230 6250 30  0001 C CNN
F 3 "" H 3300 6250 30  0000 C CNN
	1    3300 6250
	0    1    1    0   
$EndComp
Text GLabel 3850 6250 2    60   Input ~ 0
IBUS_TX
$Comp
L GND-RESCUE-stm32f103-ibus #PWR02
U 1 1 59524FF1
P 2750 6300
F 0 "#PWR02" H 2750 6300 30  0001 C CNN
F 1 "GND" H 2750 6230 30  0001 C CNN
F 2 "" H 2750 6300 60  0000 C CNN
F 3 "" H 2750 6300 60  0000 C CNN
	1    2750 6300
	1    0    0    -1  
$EndComp
$Comp
L GND-RESCUE-stm32f103-ibus #PWR03
U 1 1 59525005
P 1700 6100
F 0 "#PWR03" H 1700 6100 30  0001 C CNN
F 1 "GND" H 1700 6030 30  0001 C CNN
F 2 "" H 1700 6100 60  0000 C CNN
F 3 "" H 1700 6100 60  0000 C CNN
	1    1700 6100
	1    0    0    -1  
$EndComp
Text GLabel 15200 2100 2    60   Output ~ 0
IBUS_TX
Text GLabel 15200 2200 2    60   Input ~ 0
IBUS_RX
Text GLabel 8100 1300 0    60   Output ~ 0
USART2_TX
Text GLabel 8100 1400 0    60   Input ~ 0
USART2_RX
$Comp
L GND-RESCUE-stm32f103-ibus #PWR04
U 1 1 596304DE
P 3700 5100
F 0 "#PWR04" H 3700 5100 30  0001 C CNN
F 1 "GND" H 3700 5030 30  0001 C CNN
F 2 "" H 3700 5100 60  0000 C CNN
F 3 "" H 3700 5100 60  0000 C CNN
	1    3700 5100
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X04 P1
U 1 1 59AF8F34
P 750 1050
F 0 "P1" H 750 1300 50  0000 C CNN
F 1 "USART2" V 850 1050 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x04" H 750 1050 60  0001 C CNN
F 3 "" H 750 1050 60  0000 C CNN
	1    750  1050
	-1   0    0    -1  
$EndComp
$Comp
L GND-RESCUE-stm32f103-ibus #PWR05
U 1 1 59AF94BC
P 950 1350
F 0 "#PWR05" H 950 1350 30  0001 C CNN
F 1 "GND" H 950 1280 30  0001 C CNN
F 2 "" H 950 1350 60  0000 C CNN
F 3 "" H 950 1350 60  0000 C CNN
	1    950  1350
	1    0    0    -1  
$EndComp
Text GLabel 1250 1200 2    60   Input ~ 0
USART2_TX
Text GLabel 1100 1000 2    60   Output ~ 0
USART2_RX
$Comp
L CONN_01X02 P2
U 1 1 59AFA389
P 750 1650
F 0 "P2" H 750 1800 50  0000 C CNN
F 1 "POWER" V 850 1650 50  0000 C CNN
F 2 "Socket_Strips:Socket_Strip_Straight_1x02" H 750 1650 60  0001 C CNN
F 3 "" H 750 1650 60  0000 C CNN
	1    750  1650
	-1   0    0    -1  
$EndComp
$Comp
L GND-RESCUE-stm32f103-ibus #PWR06
U 1 1 59AFA473
P 1100 1800
F 0 "#PWR06" H 1100 1800 30  0001 C CNN
F 1 "GND" H 1100 1730 30  0001 C CNN
F 2 "" H 1100 1800 60  0000 C CNN
F 3 "" H 1100 1800 60  0000 C CNN
	1    1100 1800
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X01 P3
U 1 1 59AFA583
P 750 2100
F 0 "P3" H 750 2200 50  0000 C CNN
F 1 "I-Bus" V 850 2100 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x01" H 750 2100 60  0001 C CNN
F 3 "" H 750 2100 60  0000 C CNN
	1    750  2100
	-1   0    0    -1  
$EndComp
Text Label 1400 2100 0    60   ~ 0
I-Bus
Text GLabel 8100 2000 0    60   Output ~ 0
DBUS_TX
Text GLabel 8100 2100 0    60   Input ~ 0
DBUS_RX
Text Label 800  6950 0    60   ~ 0
D-Bus
$Comp
L PC817-RESCUE-stm32f103-ibus IC4
U 1 1 5ABD5859
P 2600 7600
F 0 "IC4" H 2390 7790 40  0000 C CNN
F 1 "PC817" H 2750 7410 40  0000 C CNN
F 2 "DIP4" H 2400 7420 30  0000 C CIN
F 3 "" H 2600 7600 60  0000 C CNN
	1    2600 7600
	1    0    0    -1  
$EndComp
$Comp
L PC817-RESCUE-stm32f103-ibus IC2
U 1 1 5ABD585F
P 2150 8600
F 0 "IC2" H 1940 8790 40  0000 C CNN
F 1 "PC817" H 2300 8410 40  0000 C CNN
F 2 "DIP4" H 1950 8420 30  0000 C CIN
F 3 "" H 2150 8600 60  0000 C CNN
	1    2150 8600
	-1   0    0    -1  
$EndComp
$Comp
L R-RESCUE-stm32f103-ibus R18
U 1 1 5ABD5865
P 3550 7700
F 0 "R18" V 3630 7700 40  0000 C CNN
F 1 "330" V 3557 7701 40  0000 C CNN
F 2 "Discret:R3" V 3480 7700 30  0001 C CNN
F 3 "" H 3550 7700 30  0000 C CNN
	1    3550 7700
	0    1    1    0   
$EndComp
Text GLabel 3950 7500 2    60   Output ~ 0
DBUS_RX
$Comp
L GND-RESCUE-stm32f103-ibus #PWR07
U 1 1 5ABD586C
P 2250 7850
F 0 "#PWR07" H 2250 7850 30  0001 C CNN
F 1 "GND" H 2250 7780 30  0001 C CNN
F 2 "" H 2250 7850 60  0000 C CNN
F 3 "" H 2250 7850 60  0000 C CNN
	1    2250 7850
	1    0    0    -1  
$EndComp
$Comp
L S8050 Q4
U 1 1 5ABD5878
P 2950 8700
F 0 "Q4" H 2950 8551 40  0000 R CNN
F 1 "S8050" H 2950 8850 40  0000 R CNN
F 2 "TO_SOT_Packages_THT:TO-92_Inline_Narrow_Oval" H 2850 8802 29  0000 C CNN
F 3 "" H 2950 8700 60  0000 C CNN
	1    2950 8700
	-1   0    0    -1  
$EndComp
$Comp
L R-RESCUE-stm32f103-ibus R15
U 1 1 5ABD587E
P 3400 8500
F 0 "R15" V 3480 8500 40  0000 C CNN
F 1 "330" V 3407 8501 40  0000 C CNN
F 2 "Discret:R3" V 3330 8500 30  0001 C CNN
F 3 "" H 3400 8500 30  0000 C CNN
	1    3400 8500
	0    1    1    0   
$EndComp
$Comp
L R-RESCUE-stm32f103-ibus R16
U 1 1 5ABD5884
P 3400 9000
F 0 "R16" V 3480 9000 40  0000 C CNN
F 1 "10K" V 3407 9001 40  0000 C CNN
F 2 "Discret:R3" V 3330 9000 30  0001 C CNN
F 3 "" H 3400 9000 30  0000 C CNN
	1    3400 9000
	0    1    1    0   
$EndComp
Text GLabel 3950 9000 2    60   Input ~ 0
DBUS_TX
$Comp
L GND-RESCUE-stm32f103-ibus #PWR08
U 1 1 5ABD5891
P 2850 9050
F 0 "#PWR08" H 2850 9050 30  0001 C CNN
F 1 "GND" H 2850 8980 30  0001 C CNN
F 2 "" H 2850 9050 60  0000 C CNN
F 3 "" H 2850 9050 60  0000 C CNN
	1    2850 9050
	1    0    0    -1  
$EndComp
$Comp
L GND-RESCUE-stm32f103-ibus #PWR09
U 1 1 5ABD5897
P 1800 8850
F 0 "#PWR09" H 1800 8850 30  0001 C CNN
F 1 "GND" H 1800 8780 30  0001 C CNN
F 2 "" H 1800 8850 60  0000 C CNN
F 3 "" H 1800 8850 60  0000 C CNN
	1    1800 8850
	1    0    0    -1  
$EndComp
$Comp
L GND-RESCUE-stm32f103-ibus #PWR010
U 1 1 5ABD589D
P 3800 7850
F 0 "#PWR010" H 3800 7850 30  0001 C CNN
F 1 "GND" H 3800 7780 30  0001 C CNN
F 2 "" H 3800 7850 60  0000 C CNN
F 3 "" H 3800 7850 60  0000 C CNN
	1    3800 7850
	1    0    0    -1  
$EndComp
$Comp
L R-RESCUE-stm32f103-ibus R9
U 1 1 5ABD6AD0
P 1300 7000
F 0 "R9" V 1380 7000 50  0000 C CNN
F 1 "3K" V 1300 7000 50  0000 C CNN
F 2 "Discret:R3" V 1230 7000 50  0001 C CNN
F 3 "" H 1300 7000 50  0000 C CNN
	1    1300 7000
	0    1    1    0   
$EndComp
$Comp
L CONN_01X01 P4
U 1 1 5ABD6F9C
P 750 2550
F 0 "P4" H 750 2650 50  0000 C CNN
F 1 "D-Bus" V 850 2550 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x01" H 750 2550 60  0001 C CNN
F 3 "" H 750 2550 60  0000 C CNN
	1    750  2550
	-1   0    0    -1  
$EndComp
Text Label 1400 2550 0    60   ~ 0
D-Bus
$Comp
L S8050 Q2
U 1 1 5AD06CEC
P 1400 7700
F 0 "Q2" H 1400 7551 40  0000 R CNN
F 1 "S8050" H 1400 7850 40  0000 R CNN
F 2 "TO_SOT_Packages_THT:TO-92_Inline_Narrow_Oval" H 1300 7802 29  0000 C CNN
F 3 "" H 1400 7700 60  0000 C CNN
	1    1400 7700
	1    0    0    -1  
$EndComp
$Comp
L R-RESCUE-stm32f103-ibus R7
U 1 1 5AD06D1F
P 1050 7450
F 0 "R7" V 1130 7450 40  0000 C CNN
F 1 "32.7K" V 1057 7451 40  0000 C CNN
F 2 "Discret:R3" V 980 7450 30  0001 C CNN
F 3 "" H 1050 7450 30  0000 C CNN
	1    1050 7450
	1    0    0    -1  
$EndComp
$Comp
L R-RESCUE-stm32f103-ibus R8
U 1 1 5AD06D80
P 1050 7950
F 0 "R8" V 1130 7950 40  0000 C CNN
F 1 "14.7K" V 1057 7951 40  0000 C CNN
F 2 "Discret:R3" V 980 7950 30  0001 C CNN
F 3 "" H 1050 7950 30  0000 C CNN
	1    1050 7950
	1    0    0    -1  
$EndComp
$Comp
L GND-RESCUE-stm32f103-ibus #PWR011
U 1 1 5AD06EAE
P 1050 8200
F 0 "#PWR011" H 1050 8200 30  0001 C CNN
F 1 "GND" H 1050 8130 30  0001 C CNN
F 2 "" H 1050 8200 60  0000 C CNN
F 3 "" H 1050 8200 60  0000 C CNN
	1    1050 8200
	1    0    0    -1  
$EndComp
$Comp
L R-RESCUE-stm32f103-ibus R11
U 1 1 5AD06F8C
P 1750 7200
F 0 "R11" V 1830 7200 40  0000 C CNN
F 1 "330" V 1757 7201 40  0000 C CNN
F 2 "Discret:R3" V 1680 7200 30  0001 C CNN
F 3 "" H 1750 7200 30  0000 C CNN
	1    1750 7200
	0    1    1    0   
$EndComp
$Comp
L STM32F103C(8-B)Tx U2
U 1 1 5AD4C0AA
P 11650 3200
F 0 "U2" H 11650 3300 50  0000 C CNN
F 1 "STM32F103C(8-B)Tx" H 11650 3100 50  0000 C CNN
F 2 "LQFP48" H 11650 3000 50  0000 C CIN
F 3 "" H 11650 3200 50  0000 C CNN
	1    11650 3200
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR012
U 1 1 5AD4DEAB
P 15350 5300
F 0 "#PWR012" H 15350 5050 50  0001 C CNN
F 1 "GND" H 15350 5150 50  0000 C CNN
F 2 "" H 15350 5300 50  0000 C CNN
F 3 "" H 15350 5300 50  0000 C CNN
	1    15350 5300
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X04 P7
U 1 1 5AD4E4DA
P 7350 2450
F 0 "P7" H 7350 2700 50  0000 C CNN
F 1 "SWD" V 7450 2450 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x04" H 7350 2450 50  0001 C CNN
F 3 "" H 7350 2450 50  0000 C CNN
	1    7350 2450
	-1   0    0    -1  
$EndComp
$Comp
L GND #PWR013
U 1 1 5AD4E81F
P 7700 2600
F 0 "#PWR013" H 7700 2350 50  0001 C CNN
F 1 "GND" H 7700 2450 50  0000 C CNN
F 2 "" H 7700 2600 50  0000 C CNN
F 3 "" H 7700 2600 50  0000 C CNN
	1    7700 2600
	1    0    0    -1  
$EndComp
$Comp
L C C1
U 1 1 5AD4EA74
P 4900 1300
F 0 "C1" H 4925 1400 50  0000 L CNN
F 1 "104" H 4925 1200 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 4938 1150 50  0001 C CNN
F 3 "" H 4900 1300 50  0000 C CNN
	1    4900 1300
	1    0    0    -1  
$EndComp
$Comp
L C C2
U 1 1 5AD4EB05
P 5300 1300
F 0 "C2" H 5325 1400 50  0000 L CNN
F 1 "1UF" H 5325 1200 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 5338 1150 50  0001 C CNN
F 3 "" H 5300 1300 50  0000 C CNN
	1    5300 1300
	1    0    0    -1  
$EndComp
$Comp
L C C3
U 1 1 5AD4EB6E
P 5700 1300
F 0 "C3" H 5725 1400 50  0000 L CNN
F 1 "104" H 5725 1200 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 5738 1150 50  0001 C CNN
F 3 "" H 5700 1300 50  0000 C CNN
	1    5700 1300
	1    0    0    -1  
$EndComp
$Comp
L C C4
U 1 1 5AD4EBC1
P 6100 1300
F 0 "C4" H 6125 1400 50  0000 L CNN
F 1 "104" H 6125 1200 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 6138 1150 50  0001 C CNN
F 3 "" H 6100 1300 50  0000 C CNN
	1    6100 1300
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR014
U 1 1 5AD4F30B
P 5500 1550
F 0 "#PWR014" H 5500 1300 50  0001 C CNN
F 1 "GND" H 5500 1400 50  0000 C CNN
F 2 "" H 5500 1550 50  0000 C CNN
F 3 "" H 5500 1550 50  0000 C CNN
	1    5500 1550
	1    0    0    -1  
$EndComp
$Comp
L Crystal Y1
U 1 1 5AD4F670
P 6300 2300
F 0 "Y1" H 6300 2450 50  0000 C CNN
F 1 "8M" H 6300 2150 50  0000 C CNN
F 2 "Crystals:Crystal_SMD_0603_2Pads_obsolete" H 6300 2300 50  0001 C CNN
F 3 "" H 6300 2300 50  0000 C CNN
	1    6300 2300
	1    0    0    -1  
$EndComp
$Comp
L R R2
U 1 1 5AD4F879
P 6300 2600
F 0 "R2" V 6380 2600 50  0000 C CNN
F 1 "10M" V 6300 2600 50  0000 C CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 6230 2600 50  0001 C CNN
F 3 "" H 6300 2600 50  0000 C CNN
	1    6300 2600
	0    1    1    0   
$EndComp
$Comp
L C C5
U 1 1 5AD4F9D1
P 6000 2750
F 0 "C5" H 6025 2850 50  0000 L CNN
F 1 "22PF" H 6025 2650 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 6038 2600 50  0001 C CNN
F 3 "" H 6000 2750 50  0000 C CNN
	1    6000 2750
	1    0    0    -1  
$EndComp
$Comp
L C C6
U 1 1 5AD4FA40
P 6600 2750
F 0 "C6" H 6625 2850 50  0000 L CNN
F 1 "22PF" H 6625 2650 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 6638 2600 50  0001 C CNN
F 3 "" H 6600 2750 50  0000 C CNN
	1    6600 2750
	1    0    0    -1  
$EndComp
Text Label 5850 2300 2    60   ~ 0
XTALA
Text Label 6750 2300 0    60   ~ 0
XTALB
$Comp
L GND #PWR015
U 1 1 5AD50194
P 6300 3000
F 0 "#PWR015" H 6300 2750 50  0001 C CNN
F 1 "GND" H 6300 2850 50  0000 C CNN
F 2 "" H 6300 3000 50  0000 C CNN
F 3 "" H 6300 3000 50  0000 C CNN
	1    6300 3000
	1    0    0    -1  
$EndComp
$Comp
L Crystal Y2
U 1 1 5AD5055B
P 6300 3550
F 0 "Y2" H 6300 3700 50  0000 C CNN
F 1 "32.768K" H 6300 3400 50  0000 C CNN
F 2 "Crystals:Crystal_HC49-U_Vertical" H 6300 3550 50  0001 C CNN
F 3 "" H 6300 3550 50  0000 C CNN
	1    6300 3550
	1    0    0    -1  
$EndComp
Text Label 5850 3550 2    60   ~ 0
XTALC
Text Label 6750 3550 0    60   ~ 0
XTALD
$Comp
L C C8
U 1 1 5AD5085B
P 6000 3850
F 0 "C8" H 6025 3950 50  0000 L CNN
F 1 "22PF" H 6025 3750 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 6038 3700 50  0001 C CNN
F 3 "" H 6000 3850 50  0000 C CNN
	1    6000 3850
	1    0    0    -1  
$EndComp
$Comp
L C C9
U 1 1 5AD508C2
P 6600 3850
F 0 "C9" H 6625 3950 50  0000 L CNN
F 1 "22PF" H 6625 3750 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 6638 3700 50  0001 C CNN
F 3 "" H 6600 3850 50  0000 C CNN
	1    6600 3850
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR016
U 1 1 5AD50C2C
P 6300 4100
F 0 "#PWR016" H 6300 3850 50  0001 C CNN
F 1 "GND" H 6300 3950 50  0000 C CNN
F 2 "" H 6300 4100 50  0000 C CNN
F 3 "" H 6300 4100 50  0000 C CNN
	1    6300 4100
	1    0    0    -1  
$EndComp
Text Label 15200 2800 0    60   ~ 0
XTALA
Text Label 15200 2900 0    60   ~ 0
XTALB
Text Label 8100 2900 2    60   ~ 0
XTALC
Text Label 8100 3000 2    60   ~ 0
XTALD
$Comp
L R R4
U 1 1 5AD52755
P 7950 4350
F 0 "R4" V 8030 4350 50  0000 C CNN
F 1 "10K" V 7950 4350 50  0000 C CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 7880 4350 50  0001 C CNN
F 3 "" H 7950 4350 50  0000 C CNN
	1    7950 4350
	0    1    1    0   
$EndComp
$Comp
L R R3
U 1 1 5AD52CAA
P 2300 1200
F 0 "R3" V 2380 1200 50  0000 C CNN
F 1 "10K" V 2300 1200 50  0000 C CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 2230 1200 50  0001 C CNN
F 3 "" H 2300 1200 50  0000 C CNN
	1    2300 1200
	1    0    0    -1  
$EndComp
$Comp
L C C7
U 1 1 5AD52D8A
P 2300 1800
F 0 "C7" H 2325 1900 50  0000 L CNN
F 1 "104" H 2325 1700 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 2338 1650 50  0001 C CNN
F 3 "" H 2300 1800 50  0000 C CNN
	1    2300 1800
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR017
U 1 1 5AD52E95
P 2300 2250
F 0 "#PWR017" H 2300 2000 50  0001 C CNN
F 1 "GND" H 2300 2100 50  0000 C CNN
F 2 "" H 2300 2250 50  0000 C CNN
F 3 "" H 2300 2250 50  0000 C CNN
	1    2300 2250
	1    0    0    -1  
$EndComp
Text Label 2800 1500 0    60   ~ 0
RST
$Comp
L SW_PUSH SW1
U 1 1 5AD530DC
P 2650 1800
F 0 "SW1" H 2800 1910 50  0000 C CNN
F 1 "SW_PUSH" H 2650 1720 50  0000 C CNN
F 2 "Buttons_Switches_SMD:SW_SPST_FSMSM" H 2650 1800 50  0001 C CNN
F 3 "" H 2650 1800 50  0000 C CNN
	1    2650 1800
	0    1    1    0   
$EndComp
Text Label 8100 4600 2    60   ~ 0
RST
$Comp
L R R5
U 1 1 5AD551D4
P 3300 1500
F 0 "R5" V 3380 1500 50  0000 C CNN
F 1 "1K" V 3300 1500 50  0000 C CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 3230 1500 50  0001 C CNN
F 3 "" H 3300 1500 50  0000 C CNN
	1    3300 1500
	1    0    0    -1  
$EndComp
$Comp
L LED D1
U 1 1 5AD5530B
P 3300 1050
F 0 "D1" H 3300 1150 50  0000 C CNN
F 1 "LED" H 3300 950 50  0000 C CNN
F 2 "LEDs:LED_1206" H 3300 1050 50  0001 C CNN
F 3 "" H 3300 1050 50  0000 C CNN
	1    3300 1050
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR018
U 1 1 5AD5566A
P 3300 1800
F 0 "#PWR018" H 3300 1550 50  0001 C CNN
F 1 "GND" H 3300 1650 50  0000 C CNN
F 2 "" H 3300 1800 50  0000 C CNN
F 3 "" H 3300 1800 50  0000 C CNN
	1    3300 1800
	1    0    0    -1  
$EndComp
$Comp
L R R20
U 1 1 5AD5582E
P 3950 1500
F 0 "R20" V 4030 1500 50  0000 C CNN
F 1 "1K" V 3950 1500 50  0000 C CNN
F 2 "Resistors_SMD:R_1206" V 3880 1500 50  0001 C CNN
F 3 "" H 3950 1500 50  0000 C CNN
	1    3950 1500
	1    0    0    -1  
$EndComp
$Comp
L LED D2
U 1 1 5AD55834
P 3950 1050
F 0 "D2" H 3950 1150 50  0000 C CNN
F 1 "LED" H 3950 950 50  0000 C CNN
F 2 "LEDs:LED_1206" H 3950 1050 50  0001 C CNN
F 3 "" H 3950 1050 50  0000 C CNN
	1    3950 1050
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR019
U 1 1 5AD57468
P 7650 4350
F 0 "#PWR019" H 7650 4100 50  0001 C CNN
F 1 "GND" H 7650 4200 50  0000 C CNN
F 2 "" H 7650 4350 50  0000 C CNN
F 3 "" H 7650 4350 50  0000 C CNN
	1    7650 4350
	1    0    0    -1  
$EndComp
$Comp
L S8050 Q1
U 1 1 5AD58B9E
P 1300 4950
F 0 "Q1" H 1300 4801 40  0000 R CNN
F 1 "S8050" H 1300 5100 40  0000 R CNN
F 2 "TO_SOT_Packages_THT:TO-92_Inline_Narrow_Oval" H 1200 5052 29  0000 C CNN
F 3 "" H 1300 4950 60  0000 C CNN
	1    1300 4950
	1    0    0    -1  
$EndComp
$Comp
L R-RESCUE-stm32f103-ibus R1
U 1 1 5AD58BA4
P 950 4700
F 0 "R1" V 1030 4700 40  0000 C CNN
F 1 "32.7K" V 957 4701 40  0000 C CNN
F 2 "Discret:R3" V 880 4700 30  0001 C CNN
F 3 "" H 950 4700 30  0000 C CNN
	1    950  4700
	1    0    0    -1  
$EndComp
$Comp
L R-RESCUE-stm32f103-ibus R6
U 1 1 5AD58BAA
P 950 5200
F 0 "R6" V 1030 5200 40  0000 C CNN
F 1 "14.7K" V 957 5201 40  0000 C CNN
F 2 "Discret:R3" V 880 5200 30  0001 C CNN
F 3 "" H 950 5200 30  0000 C CNN
	1    950  5200
	1    0    0    -1  
$EndComp
$Comp
L GND-RESCUE-stm32f103-ibus #PWR020
U 1 1 5AD58BB0
P 950 5450
F 0 "#PWR020" H 950 5450 30  0001 C CNN
F 1 "GND" H 950 5380 30  0001 C CNN
F 2 "" H 950 5450 60  0000 C CNN
F 3 "" H 950 5450 60  0000 C CNN
	1    950  5450
	1    0    0    -1  
$EndComp
$Comp
L R-RESCUE-stm32f103-ibus R10
U 1 1 5AD58BBC
P 1650 4450
F 0 "R10" V 1730 4450 40  0000 C CNN
F 1 "330" V 1657 4451 40  0000 C CNN
F 2 "Discret:R3" V 1580 4450 30  0001 C CNN
F 3 "" H 1650 4450 30  0000 C CNN
	1    1650 4450
	0    1    1    0   
$EndComp
$Comp
L GND #PWR021
U 1 1 5AD768F9
P 6600 6250
F 0 "#PWR021" H 6600 6000 50  0001 C CNN
F 1 "GND" H 6600 6100 50  0000 C CNN
F 2 "" H 6600 6250 50  0000 C CNN
F 3 "" H 6600 6250 50  0000 C CNN
	1    6600 6250
	1    0    0    -1  
$EndComp
$Comp
L R R25
U 1 1 5AD76A37
P 7200 5900
F 0 "R25" V 7280 5900 50  0000 C CNN
F 1 "10K" V 7200 5900 50  0000 C CNN
F 2 "Resistors_SMD:R_1206" V 7130 5900 50  0001 C CNN
F 3 "" H 7200 5900 50  0000 C CNN
	1    7200 5900
	0    1    1    0   
$EndComp
Text GLabel 8100 1200 0    60   Output ~ 0
PA1
Text GLabel 5350 6700 0    60   BiDi ~ 0
HEATER
$Comp
L R R22
U 1 1 5AD78455
P 6250 7250
F 0 "R22" V 6330 7250 50  0000 C CNN
F 1 "0.097/2W" V 6250 7250 50  0000 C CNN
F 2 "Resistors_SMD:R_1206" V 6180 7250 50  0001 C CNN
F 3 "" H 6250 7250 50  0000 C CNN
	1    6250 7250
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR022
U 1 1 5AD7868A
P 6600 7350
F 0 "#PWR022" H 6600 7100 50  0001 C CNN
F 1 "GND" H 6600 7200 50  0000 C CNN
F 2 "" H 6600 7350 50  0000 C CNN
F 3 "" H 6600 7350 50  0000 C CNN
	1    6600 7350
	1    0    0    -1  
$EndComp
$Comp
L R R23
U 1 1 5AD7921A
P 7150 7200
F 0 "R23" V 7230 7200 50  0000 C CNN
F 1 "50K" V 7150 7200 50  0000 C CNN
F 2 "Resistors_SMD:R_1206" V 7080 7200 50  0001 C CNN
F 3 "" H 7150 7200 50  0000 C CNN
	1    7150 7200
	1    0    0    -1  
$EndComp
$Comp
L R R24
U 1 1 5AD7929F
P 7150 7800
F 0 "R24" V 7230 7800 50  0000 C CNN
F 1 "10K" V 7150 7800 50  0000 C CNN
F 2 "Resistors_SMD:R_1206" V 7080 7800 50  0001 C CNN
F 3 "" H 7150 7800 50  0000 C CNN
	1    7150 7800
	1    0    0    -1  
$EndComp
Text GLabel 7300 7050 2    60   Output ~ 0
ADC1_5
$Comp
L GND #PWR023
U 1 1 5AD79DB0
P 6250 7400
F 0 "#PWR023" H 6250 7150 50  0001 C CNN
F 1 "GND" H 6250 7250 50  0000 C CNN
F 2 "" H 6250 7400 50  0000 C CNN
F 3 "" H 6250 7400 50  0000 C CNN
	1    6250 7400
	1    0    0    -1  
$EndComp
Text GLabel 8100 1600 0    60   Input ~ 0
ADC1_5
Text GLabel 2900 3550 0    60   Input ~ 0
PWM1
$Comp
L R R12
U 1 1 5AD7A6EC
P 3200 3550
F 0 "R12" V 3280 3550 50  0000 C CNN
F 1 "10K" V 3200 3550 50  0000 C CNN
F 2 "Resistors_SMD:R_1206" V 3130 3550 50  0001 C CNN
F 3 "" H 3200 3550 50  0000 C CNN
	1    3200 3550
	0    1    1    0   
$EndComp
$Comp
L S8050 Q5
U 1 1 5AD7A7F8
P 3700 3550
F 0 "Q5" H 3900 3625 50  0000 L CNN
F 1 "S8050" H 3900 3550 50  0000 L CNN
F 2 "TO_SOT_Packages_THT:TO-92_Inline_Narrow_Oval" H 3900 3475 50  0000 L CIN
F 3 "" H 3700 3550 50  0000 L CNN
	1    3700 3550
	1    0    0    -1  
$EndComp
$Comp
L S8050 Q6
U 1 1 5AD7AA3A
P 4150 3000
F 0 "Q6" H 4350 3075 50  0000 L CNN
F 1 "S8050" H 4350 3000 50  0000 L CNN
F 2 "TO_SOT_Packages_THT:TO-92_Inline_Narrow_Oval" H 4350 2925 50  0000 L CIN
F 3 "" H 4150 3000 50  0000 L CNN
	1    4150 3000
	1    0    0    -1  
$EndComp
$Comp
L R R19
U 1 1 5AD7ABE6
P 3800 2700
F 0 "R19" V 3880 2700 50  0000 C CNN
F 1 "10K" V 3800 2700 50  0000 C CNN
F 2 "Resistors_SMD:R_1206" V 3730 2700 50  0001 C CNN
F 3 "" H 3800 2700 50  0001 C CNN
	1    3800 2700
	1    0    0    -1  
$EndComp
Text GLabel 4400 3350 2    60   Output ~ 0
PWM_FAN
$Comp
L +12V #PWR024
U 1 1 5AD7B4A9
P 4250 2550
F 0 "#PWR024" H 4250 2400 50  0001 C CNN
F 1 "+12V" H 4250 2690 50  0000 C CNN
F 2 "" H 4250 2550 50  0001 C CNN
F 3 "" H 4250 2550 50  0001 C CNN
	1    4250 2550
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR025
U 1 1 5AD7BAE9
P 3800 3900
F 0 "#PWR025" H 3800 3650 50  0001 C CNN
F 1 "GND" H 3800 3750 50  0000 C CNN
F 2 "" H 3800 3900 50  0001 C CNN
F 3 "" H 3800 3900 50  0001 C CNN
	1    3800 3900
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR026
U 1 1 5AD7CC02
P 7950 5450
F 0 "#PWR026" H 7950 5300 50  0001 C CNN
F 1 "+3.3V" H 7950 5590 50  0000 C CNN
F 2 "" H 7950 5450 50  0001 C CNN
F 3 "" H 7950 5450 50  0001 C CNN
	1    7950 5450
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR027
U 1 1 5AD7D02E
P 2050 4450
F 0 "#PWR027" H 2050 4300 50  0001 C CNN
F 1 "+3.3V" H 2050 4590 50  0000 C CNN
F 2 "" H 2050 4450 50  0001 C CNN
F 3 "" H 2050 4450 50  0001 C CNN
	1    2050 4450
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR028
U 1 1 5AD7D0B6
P 3000 4600
F 0 "#PWR028" H 3000 4450 50  0001 C CNN
F 1 "+3.3V" H 3000 4740 50  0000 C CNN
F 2 "" H 3000 4600 50  0001 C CNN
F 3 "" H 3000 4600 50  0001 C CNN
	1    3000 4600
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR029
U 1 1 5AD7D261
P 3700 5600
F 0 "#PWR029" H 3700 5450 50  0001 C CNN
F 1 "+3.3V" H 3700 5740 50  0000 C CNN
F 2 "" H 3700 5600 50  0001 C CNN
F 3 "" H 3700 5600 50  0001 C CNN
	1    3700 5600
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR030
U 1 1 5AD7D445
P 4800 6300
F 0 "#PWR030" H 4800 6150 50  0001 C CNN
F 1 "+3.3V" H 4800 6440 50  0000 C CNN
F 2 "" H 4800 6300 50  0001 C CNN
F 3 "" H 4800 6300 50  0001 C CNN
	1    4800 6300
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR031
U 1 1 5AD7D99A
P 2000 7050
F 0 "#PWR031" H 2000 6900 50  0001 C CNN
F 1 "+3.3V" H 2000 7190 50  0000 C CNN
F 2 "" H 2000 7050 50  0001 C CNN
F 3 "" H 2000 7050 50  0001 C CNN
	1    2000 7050
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR032
U 1 1 5AD7DA22
P 3100 7350
F 0 "#PWR032" H 3100 7200 50  0001 C CNN
F 1 "+3.3V" H 3100 7490 50  0000 C CNN
F 2 "" H 3100 7350 50  0001 C CNN
F 3 "" H 3100 7350 50  0001 C CNN
	1    3100 7350
	1    0    0    -1  
$EndComp
$Comp
L +12V #PWR033
U 1 1 5AD7DB59
P 1700 7000
F 0 "#PWR033" H 1700 6850 50  0001 C CNN
F 1 "+12V" H 1700 7140 50  0000 C CNN
F 2 "" H 1700 7000 50  0001 C CNN
F 3 "" H 1700 7000 50  0001 C CNN
	1    1700 7000
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR034
U 1 1 5AD7DE9F
P 3800 8350
F 0 "#PWR034" H 3800 8200 50  0001 C CNN
F 1 "+3.3V" H 3800 8490 50  0000 C CNN
F 2 "" H 3800 8350 50  0001 C CNN
F 3 "" H 3800 8350 50  0001 C CNN
	1    3800 8350
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR035
U 1 1 5AD7E0F7
P 2300 900
F 0 "#PWR035" H 2300 750 50  0001 C CNN
F 1 "+3.3V" H 2300 1040 50  0000 C CNN
F 2 "" H 2300 900 50  0001 C CNN
F 3 "" H 2300 900 50  0001 C CNN
	1    2300 900 
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR036
U 1 1 5AD7E267
P 5500 900
F 0 "#PWR036" H 5500 750 50  0001 C CNN
F 1 "+3.3V" H 5500 1040 50  0000 C CNN
F 2 "" H 5500 900 50  0001 C CNN
F 3 "" H 5500 900 50  0001 C CNN
	1    5500 900 
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR037
U 1 1 5AD7EC4D
P 3300 900
F 0 "#PWR037" H 3300 750 50  0001 C CNN
F 1 "+3.3V" H 3300 1040 50  0000 C CNN
F 2 "" H 3300 900 50  0001 C CNN
F 3 "" H 3300 900 50  0001 C CNN
	1    3300 900 
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR038
U 1 1 5AD7ECD5
P 3950 900
F 0 "#PWR038" H 3950 750 50  0001 C CNN
F 1 "+3.3V" H 3950 1040 50  0000 C CNN
F 2 "" H 3950 900 50  0001 C CNN
F 3 "" H 3950 900 50  0001 C CNN
	1    3950 900 
	1    0    0    -1  
$EndComp
Text GLabel 15200 2400 2    60   Output ~ 0
PB12
Text GLabel 4100 1800 2    60   Input ~ 0
PB12
$Comp
L S8050 Q7
U 1 1 5AD80FB5
P 6700 5900
F 0 "Q7" H 6900 5975 50  0000 L CNN
F 1 "S8050" H 6900 5900 50  0000 L CNN
F 2 "TO_SOT_Packages_THT:TO-92_Rugged" H 6900 5825 50  0000 L CIN
F 3 "" H 6700 5900 50  0000 L CNN
	1    6700 5900
	-1   0    0    -1  
$EndComp
Text GLabel 7500 5900 2    60   Input ~ 0
PA1
Text GLabel 8100 1100 0    60   Output ~ 0
PWM1
$Comp
L CONN_01X01 P5
U 1 1 5AD85CB0
P 750 3050
F 0 "P5" H 750 3150 50  0000 C CNN
F 1 "HEATER" V 850 3050 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x01" H 750 3050 50  0001 C CNN
F 3 "" H 750 3050 50  0000 C CNN
	1    750  3050
	-1   0    0    -1  
$EndComp
$Comp
L CONN_01X01 P6
U 1 1 5AD85DD4
P 750 3550
F 0 "P6" H 750 3650 50  0000 C CNN
F 1 "PWM_FAN" V 850 3550 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x01" H 750 3550 50  0001 C CNN
F 3 "" H 750 3550 50  0000 C CNN
	1    750  3550
	-1   0    0    -1  
$EndComp
Text GLabel 950  3050 2    60   BiDi ~ 0
HEATER
Text GLabel 950  3550 2    60   Input ~ 0
PWM_FAN
$Comp
L +3.3V #PWR039
U 1 1 5AD871DF
P 1100 900
F 0 "#PWR039" H 1100 750 50  0001 C CNN
F 1 "+3.3V" H 1100 1040 50  0000 C CNN
F 2 "" H 1100 900 50  0001 C CNN
F 3 "" H 1100 900 50  0001 C CNN
	1    1100 900 
	1    0    0    -1  
$EndComp
$Comp
L +12V #PWR040
U 1 1 5AD87398
P 1100 1600
F 0 "#PWR040" H 1100 1450 50  0001 C CNN
F 1 "+12V" H 1100 1740 50  0000 C CNN
F 2 "" H 1100 1600 50  0001 C CNN
F 3 "" H 1100 1600 50  0001 C CNN
	1    1100 1600
	1    0    0    -1  
$EndComp
NoConn ~ 8250 1500
NoConn ~ 8250 1700
NoConn ~ 8250 1800
NoConn ~ 8250 1900
NoConn ~ 8250 2600
NoConn ~ 8250 2300
NoConn ~ 8250 2200
NoConn ~ 8250 2800
NoConn ~ 15050 1100
NoConn ~ 15050 1200
NoConn ~ 15050 1300
NoConn ~ 15050 1400
NoConn ~ 15050 1500
NoConn ~ 15050 1600
NoConn ~ 15050 1700
NoConn ~ 15050 1800
NoConn ~ 15050 1900
NoConn ~ 15050 2000
NoConn ~ 15050 2400
NoConn ~ 15050 2500
NoConn ~ 15050 2600
NoConn ~ 6100 6600
Wire Bus Line
	700  4300 700  6250
Wire Bus Line
	700  5700 700  6150
Wire Wire Line
	2150 4950 2150 5100
Wire Wire Line
	2850 4750 3000 4750
Wire Wire Line
	3000 4750 3000 4600
Wire Wire Line
	3050 5950 3050 6250
Wire Wire Line
	3550 5750 3700 5750
Wire Wire Line
	3700 5750 3700 5600
Wire Wire Line
	2750 6150 2750 6300
Wire Wire Line
	2400 5950 2400 6150
Wire Wire Line
	2400 6150 2750 6150
Wire Wire Line
	1700 5950 1700 6100
Wire Wire Line
	3700 4950 3700 5100
Wire Wire Line
	2850 4950 3200 4950
Wire Wire Line
	3850 4750 3200 4750
Wire Wire Line
	3200 4750 3200 4950
Wire Wire Line
	3850 6250 3550 6250
Wire Wire Line
	2400 5750 3050 5750
Connection ~ 2750 5750
Wire Wire Line
	950  900  1100 900 
Wire Wire Line
	950  1100 1100 1100
Wire Wire Line
	1100 1100 1100 1200
Wire Wire Line
	950  1000 1100 1000
Wire Wire Line
	1100 1200 1250 1200
Wire Wire Line
	950  1200 950  1350
Wire Wire Line
	950  1600 1100 1600
Wire Wire Line
	950  1700 1100 1700
Wire Wire Line
	1100 1700 1100 1800
Wire Bus Line
	1400 1950 1400 2300
Wire Wire Line
	950  2100 1400 2100
Wire Bus Line
	700  700  700  1150
Wire Wire Line
	2250 7700 2250 7850
Wire Wire Line
	2950 7500 3100 7500
Wire Wire Line
	3100 7500 3100 7350
Wire Wire Line
	3150 8700 3150 9000
Wire Wire Line
	3650 8500 3800 8500
Wire Wire Line
	3800 8500 3800 8350
Wire Wire Line
	2850 8900 2850 9050
Wire Wire Line
	2500 8700 2500 8900
Wire Wire Line
	2500 8900 2850 8900
Wire Wire Line
	1800 8700 1800 8850
Wire Wire Line
	3800 7700 3800 7850
Wire Wire Line
	2950 7700 3300 7700
Wire Wire Line
	3950 7500 3300 7500
Wire Wire Line
	3300 7500 3300 7700
Wire Wire Line
	3950 9000 3650 9000
Wire Wire Line
	2500 8500 3150 8500
Connection ~ 2850 8500
Wire Bus Line
	1400 2400 1400 2750
Wire Wire Line
	950  2550 1400 2550
Wire Wire Line
	1050 7700 1200 7700
Wire Bus Line
	800  8700 800  6950
Wire Wire Line
	2100 7500 2250 7500
Wire Wire Line
	2100 7500 2100 7900
Wire Wire Line
	2100 7900 1500 7900
Wire Wire Line
	1500 7200 1500 7500
Wire Wire Line
	8250 1400 8100 1400
Wire Wire Line
	8250 1300 8100 1300
Wire Wire Line
	15050 2100 15200 2100
Wire Wire Line
	15200 2200 15050 2200
Wire Wire Line
	8100 2000 8250 2000
Wire Wire Line
	8100 2100 8250 2100
Wire Wire Line
	15050 4900 15350 4900
Wire Wire Line
	15350 4900 15350 5300
Wire Wire Line
	15050 5000 15350 5000
Connection ~ 15350 5000
Wire Wire Line
	15050 5100 15350 5100
Connection ~ 15350 5100
Wire Wire Line
	15050 5200 15350 5200
Connection ~ 15350 5200
Wire Wire Line
	7550 2400 8250 2400
Wire Wire Line
	7550 2500 8250 2500
Wire Wire Line
	7550 2600 7700 2600
Wire Wire Line
	4900 1150 6100 1150
Connection ~ 5700 1150
Connection ~ 5300 1150
Wire Wire Line
	5500 900  5500 1150
Connection ~ 5500 1150
Wire Wire Line
	4900 1450 6100 1450
Connection ~ 5300 1450
Connection ~ 5700 1450
Wire Wire Line
	5500 1550 5500 1450
Connection ~ 5500 1450
Wire Wire Line
	5850 2300 6150 2300
Wire Wire Line
	6450 2300 6750 2300
Wire Wire Line
	6000 2600 6150 2600
Wire Wire Line
	6450 2600 6600 2600
Connection ~ 6600 2300
Connection ~ 6000 2300
Wire Wire Line
	6000 2900 6600 2900
Wire Wire Line
	6300 3000 6300 2900
Connection ~ 6300 2900
Wire Wire Line
	6000 2300 6000 2600
Wire Wire Line
	6600 2600 6600 2300
Wire Wire Line
	5850 3550 6150 3550
Wire Wire Line
	6000 3700 6000 3550
Connection ~ 6000 3550
Wire Wire Line
	6450 3550 6750 3550
Wire Wire Line
	6600 3700 6600 3550
Connection ~ 6600 3550
Wire Wire Line
	6000 4000 6600 4000
Wire Wire Line
	6300 4100 6300 4000
Connection ~ 6300 4000
Wire Wire Line
	15050 2800 15200 2800
Wire Wire Line
	15200 2900 15050 2900
Wire Wire Line
	8100 2900 8250 2900
Wire Wire Line
	8250 3000 8100 3000
Wire Wire Line
	2300 1350 2300 1650
Wire Wire Line
	2300 1950 2300 2250
Wire Wire Line
	2650 2100 2300 2100
Connection ~ 2300 2100
Wire Wire Line
	2300 1500 2800 1500
Connection ~ 2300 1500
Connection ~ 2650 1500
Wire Wire Line
	2300 900  2300 1050
Wire Wire Line
	3300 1200 3300 1350
Wire Wire Line
	3300 1800 3300 1650
Wire Wire Line
	3950 1200 3950 1350
Wire Wire Line
	15050 2300 15150 2300
Wire Wire Line
	15150 2300 15150 2400
Wire Wire Line
	15150 2400 15200 2400
Wire Wire Line
	3950 1650 3950 1800
Wire Wire Line
	3950 1800 4100 1800
Wire Wire Line
	8100 4600 8250 4600
Wire Wire Line
	8250 4500 8100 4500
Wire Wire Line
	8100 4500 8100 4350
Wire Wire Line
	7800 4350 7650 4350
Wire Wire Line
	950  4950 1100 4950
Wire Wire Line
	2000 4750 2150 4750
Wire Wire Line
	2000 4750 2000 5150
Wire Wire Line
	2000 5150 1400 5150
Wire Wire Line
	1400 4450 1400 4750
Wire Wire Line
	4800 6300 5500 6300
Wire Wire Line
	6600 6100 6600 6250
Wire Wire Line
	8100 1200 8250 1200
Wire Wire Line
	5500 6700 5350 6700
Wire Wire Line
	6100 6950 6400 6950
Wire Wire Line
	6250 6950 6250 7100
Connection ~ 6250 6950
Wire Wire Line
	7150 7350 7150 7650
Wire Wire Line
	7150 7500 6850 7500
Wire Wire Line
	6850 7500 6850 7650
Wire Wire Line
	6850 7650 6400 7650
Wire Wire Line
	6400 7650 6400 7150
Connection ~ 7150 7500
Wire Wire Line
	7000 7050 7300 7050
Connection ~ 7150 7050
Wire Wire Line
	8250 1600 8100 1600
Wire Wire Line
	3800 2850 3800 3350
Wire Wire Line
	3950 3000 3800 3000
Connection ~ 3800 3000
Wire Wire Line
	3800 3350 3950 3350
Wire Wire Line
	4250 3200 4250 3350
Wire Wire Line
	4250 3350 4400 3350
Wire Wire Line
	3800 2550 4250 2550
Wire Wire Line
	4250 2550 4250 2800
Wire Wire Line
	2900 3550 3050 3550
Wire Wire Line
	3350 3550 3500 3550
Wire Wire Line
	3800 3750 3800 3900
Wire Wire Line
	6900 5900 7050 5900
Wire Wire Line
	7350 5900 7500 5900
Wire Wire Line
	6100 6300 6100 5700
Wire Wire Line
	6100 5700 6600 5700
Wire Wire Line
	6100 6800 6100 6950
Wire Wire Line
	8100 1100 8250 1100
Wire Wire Line
	950  4450 700  4450
Wire Wire Line
	700  4450 700  4300
Wire Wire Line
	1700 5750 800  5750
Wire Wire Line
	800  5750 800  4450
Connection ~ 800  4450
Wire Wire Line
	1050 7000 800  7000
Wire Wire Line
	800  7000 800  6950
Wire Wire Line
	1050 7200 1050 7000
Wire Wire Line
	1800 8500 900  8500
Wire Wire Line
	900  8500 900  7000
Connection ~ 900  7000
Wire Wire Line
	2050 4450 1900 4450
Wire Wire Line
	8250 4700 8100 4700
Wire Wire Line
	8100 4700 8100 5450
Wire Wire Line
	8100 5450 7950 5450
Wire Wire Line
	8250 4900 8100 4900
Connection ~ 8100 4900
Wire Wire Line
	8250 5000 8100 5000
Connection ~ 8100 5000
Wire Wire Line
	8250 5100 8100 5100
Connection ~ 8100 5100
Wire Wire Line
	8250 5200 8100 5200
Connection ~ 8100 5200
Wire Wire Line
	2000 7050 2000 7200
Wire Wire Line
	1550 7000 1700 7000
$Comp
L PWR_FLAG #FLG041
U 1 1 5AD8EE3F
P 2000 3050
F 0 "#FLG041" H 2000 3125 50  0001 C CNN
F 1 "PWR_FLAG" H 2000 3200 50  0000 C CNN
F 2 "" H 2000 3050 50  0001 C CNN
F 3 "" H 2000 3050 50  0001 C CNN
	1    2000 3050
	-1   0    0    1   
$EndComp
$Comp
L +12V #PWR042
U 1 1 5AD8EECD
P 2000 2900
F 0 "#PWR042" H 2000 2750 50  0001 C CNN
F 1 "+12V" H 2000 3040 50  0000 C CNN
F 2 "" H 2000 2900 50  0001 C CNN
F 3 "" H 2000 2900 50  0001 C CNN
	1    2000 2900
	1    0    0    -1  
$EndComp
Wire Wire Line
	2000 2900 2000 3050
$Comp
L PWR_FLAG #FLG043
U 1 1 5AD8F758
P 2450 3050
F 0 "#FLG043" H 2450 3125 50  0001 C CNN
F 1 "PWR_FLAG" H 2450 3200 50  0000 C CNN
F 2 "" H 2450 3050 50  0001 C CNN
F 3 "" H 2450 3050 50  0001 C CNN
	1    2450 3050
	-1   0    0    1   
$EndComp
$Comp
L +3.3V #PWR044
U 1 1 5AD8FB20
P 2450 2900
F 0 "#PWR044" H 2450 2750 50  0001 C CNN
F 1 "+3.3V" H 2450 3040 50  0000 C CNN
F 2 "" H 2450 2900 50  0001 C CNN
F 3 "" H 2450 2900 50  0001 C CNN
	1    2450 2900
	1    0    0    -1  
$EndComp
Wire Wire Line
	2450 3050 2450 2900
$Comp
L GND #PWR045
U 1 1 5AD902E8
P 2900 3050
F 0 "#PWR045" H 2900 2800 50  0001 C CNN
F 1 "GND" H 2900 2900 50  0000 C CNN
F 2 "" H 2900 3050 50  0001 C CNN
F 3 "" H 2900 3050 50  0001 C CNN
	1    2900 3050
	1    0    0    -1  
$EndComp
$Comp
L PWR_FLAG #FLG046
U 1 1 5AD90376
P 2900 2900
F 0 "#FLG046" H 2900 2975 50  0001 C CNN
F 1 "PWR_FLAG" H 2900 3050 50  0000 C CNN
F 2 "" H 2900 2900 50  0001 C CNN
F 3 "" H 2900 2900 50  0001 C CNN
	1    2900 2900
	1    0    0    -1  
$EndComp
Wire Wire Line
	2900 2900 2900 3050
$Comp
L LM358N U1
U 1 1 5AD918A1
P 6700 7050
F 0 "U1" H 6700 7250 50  0000 L CNN
F 1 "LM358N" H 6700 6850 50  0000 L CNN
F 2 "Housings_DIP:DIP-8_W7.62mm" H 6700 7050 50  0001 C CNN
F 3 "" H 6700 7050 50  0001 C CNN
	1    6700 7050
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR047
U 1 1 5AD91ABC
P 6600 6600
F 0 "#PWR047" H 6600 6450 50  0001 C CNN
F 1 "+3.3V" H 6600 6740 50  0000 C CNN
F 2 "" H 6600 6600 50  0001 C CNN
F 3 "" H 6600 6600 50  0001 C CNN
	1    6600 6600
	1    0    0    -1  
$EndComp
Wire Wire Line
	6600 6600 6600 6750
$Comp
L GND #PWR048
U 1 1 5AD9246B
P 7150 8100
F 0 "#PWR048" H 7150 7850 50  0001 C CNN
F 1 "GND" H 7150 7950 50  0000 C CNN
F 2 "" H 7150 8100 50  0001 C CNN
F 3 "" H 7150 8100 50  0001 C CNN
	1    7150 8100
	1    0    0    -1  
$EndComp
Wire Wire Line
	7150 7950 7150 8100
$Comp
L +3.3V #PWR049
U 1 1 5AD92827
P 7550 2150
F 0 "#PWR049" H 7550 2000 50  0001 C CNN
F 1 "+3.3V" H 7550 2290 50  0000 C CNN
F 2 "" H 7550 2150 50  0001 C CNN
F 3 "" H 7550 2150 50  0001 C CNN
	1    7550 2150
	1    0    0    -1  
$EndComp
Wire Wire Line
	7550 2150 7550 2300
$Comp
L 1N4148 D3
U 1 1 5AD7B95A
P 4100 3350
F 0 "D3" H 4100 3450 50  0000 C CNN
F 1 "1N4148" H 4100 3250 50  0000 C CNN
F 2 "Diodes_THT:D_DO-35_SOD27_Horizontal_RM10" H 4100 3175 50  0001 C CNN
F 3 "" H 4100 3350 50  0001 C CNN
	1    4100 3350
	1    0    0    -1  
$EndComp
$Comp
L SANYOU_SRD_Form_C K1
U 1 1 5AD7B75D
P 5800 6500
F 0 "K1" H 6250 6650 50  0000 L CNN
F 1 "SANYOU_SRD_Form_C" H 6250 6550 50  0000 L CNN
F 2 "Relays_THT:Relay_SPDT_SANYOU_SRD_Series_Form_C" H 7250 6450 50  0001 C CNN
F 3 "" H 5800 6500 50  0001 C CNN
	1    5800 6500
	0    1    1    0   
$EndComp
$EndSCHEMATC
