EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:switches
LIBS:relays
LIBS:motors
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
LIBS:UHTS
LIBS:UHTSControl7Thermocouples-cache
EELAYER 25 0
EELAYER END
$Descr A3 16535 11693
encoding utf-8
Sheet 1 8
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L GND #PWR01
U 1 1 5A6C9437
P 1700 2300
F 0 "#PWR01" H 1700 2050 50  0001 C CNN
F 1 "GND" H 1700 2150 50  0000 C CNN
F 2 "" H 1700 2300 50  0001 C CNN
F 3 "" H 1700 2300 50  0001 C CNN
	1    1700 2300
	1    0    0    -1  
$EndComp
$Comp
L Conn_01x12 J1
U 1 1 5A6D1780
P 1100 1700
F 0 "J1" H 1100 2300 50  0000 C CNN
F 1 "Conn_01x12" H 1100 1000 50  0000 C CNN
F 2 "UHTS:RSPro12-wayScrewTerminalPCBMount" H 1100 1700 50  0001 C CNN
F 3 "" H 1100 1700 50  0001 C CNN
	1    1100 1700
	-1   0    0    1   
$EndComp
Text Label 1850 2200 0    60   ~ 0
GND
Text Label 1850 2100 0    60   ~ 0
VIN
Text Label 1400 1600 0    60   ~ 0
CS1
Text Label 1400 1500 0    60   ~ 0
CS2
Text Label 1400 1400 0    60   ~ 0
CS3
Text Label 1400 1300 0    60   ~ 0
CS4
Text Label 1400 1200 0    60   ~ 0
CS5
Text Label 1400 1100 0    60   ~ 0
CS6
$Comp
L LM317L_TO92 U1
U 1 1 5A6DDBDE
P 3500 1300
F 0 "U1" H 3350 1425 50  0000 C CNN
F 1 "LM317L_TO92" H 3500 1425 50  0000 L CNN
F 2 "TO_SOT_Packages_THT:TO-92_Inline_Narrow_Oval" H 3500 1525 50  0001 C CIN
F 3 "" H 3500 1300 50  0001 C CNN
	1    3500 1300
	1    0    0    -1  
$EndComp
Text Label 2450 1300 2    60   ~ 0
VIN
$Comp
L R R2
U 1 1 5A6E0E1E
P 3500 2000
F 0 "R2" V 3580 2000 50  0000 C CNN
F 1 "560" V 3500 2000 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 3430 2000 50  0001 C CNN
F 3 "" H 3500 2000 50  0001 C CNN
	1    3500 2000
	1    0    0    -1  
$EndComp
$Comp
L R R3
U 1 1 5A6E0F44
P 3900 1750
F 0 "R3" V 3980 1750 50  0000 C CNN
F 1 "330" V 3900 1750 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 3830 1750 50  0001 C CNN
F 3 "" H 3900 1750 50  0001 C CNN
	1    3900 1750
	0    1    1    0   
$EndComp
Text Label 4600 1300 0    60   ~ 0
VDD
$Comp
L GND #PWR02
U 1 1 5A6E1C16
P 3500 2300
F 0 "#PWR02" H 3500 2050 50  0001 C CNN
F 1 "GND" H 3500 2150 50  0000 C CNN
F 2 "" H 3500 2300 50  0001 C CNN
F 3 "" H 3500 2300 50  0001 C CNN
	1    3500 2300
	1    0    0    -1  
$EndComp
$Comp
L +VSW #PWR03
U 1 1 5A6EA020
P 1700 2000
F 0 "#PWR03" H 1700 1850 50  0001 C CNN
F 1 "+VSW" H 1700 2140 50  0000 C CNN
F 2 "" H 1700 2000 50  0001 C CNN
F 3 "" H 1700 2000 50  0001 C CNN
	1    1700 2000
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR04
U 1 1 5A6EB1DE
P 4100 1100
F 0 "#PWR04" H 4100 950 50  0001 C CNN
F 1 "+3.3V" H 4100 1240 50  0000 C CNN
F 2 "" H 4100 1100 50  0001 C CNN
F 3 "" H 4100 1100 50  0001 C CNN
	1    4100 1100
	1    0    0    -1  
$EndComp
$Comp
L PWR_FLAG #FLG05
U 1 1 5A6F19C1
P 4000 3000
F 0 "#FLG05" H 4000 3075 50  0001 C CNN
F 1 "PWR_FLAG" H 4000 3150 50  0000 C CNN
F 2 "" H 4000 3000 50  0001 C CNN
F 3 "" H 4000 3000 50  0001 C CNN
	1    4000 3000
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR06
U 1 1 5A6F2334
P 4000 3100
F 0 "#PWR06" H 4000 2850 50  0001 C CNN
F 1 "GND" H 4000 2950 50  0000 C CNN
F 2 "" H 4000 3100 50  0001 C CNN
F 3 "" H 4000 3100 50  0001 C CNN
	1    4000 3100
	1    0    0    -1  
$EndComp
$Comp
L C C1
U 1 1 5A6F3575
P 3100 1600
F 0 "C1" H 3125 1700 50  0000 L CNN
F 1 "0.1uF" H 3125 1500 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 3138 1450 50  0001 C CNN
F 3 "" H 3100 1600 50  0001 C CNN
	1    3100 1600
	1    0    0    -1  
$EndComp
$Comp
L C C2
U 1 1 5A6F426E
P 4300 1600
F 0 "C2" H 4325 1700 50  0000 L CNN
F 1 "1uF" H 4325 1500 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 4338 1450 50  0001 C CNN
F 3 "" H 4300 1600 50  0001 C CNN
	1    4300 1600
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR07
U 1 1 5A6F4562
P 3100 1900
F 0 "#PWR07" H 3100 1650 50  0001 C CNN
F 1 "GND" H 3100 1750 50  0000 C CNN
F 2 "" H 3100 1900 50  0001 C CNN
F 3 "" H 3100 1900 50  0001 C CNN
	1    3100 1900
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR08
U 1 1 5A6F463C
P 4300 1900
F 0 "#PWR08" H 4300 1650 50  0001 C CNN
F 1 "GND" H 4300 1750 50  0000 C CNN
F 2 "" H 4300 1900 50  0001 C CNN
F 3 "" H 4300 1900 50  0001 C CNN
	1    4300 1900
	1    0    0    -1  
$EndComp
$Comp
L PWR_FLAG #FLG09
U 1 1 5A6CDEC2
P 3600 3000
F 0 "#FLG09" H 3600 3075 50  0001 C CNN
F 1 "PWR_FLAG" H 3600 3150 50  0000 C CNN
F 2 "" H 3600 3000 50  0001 C CNN
F 3 "" H 3600 3000 50  0001 C CNN
	1    3600 3000
	1    0    0    -1  
$EndComp
$Comp
L +VSW #PWR010
U 1 1 5A6CDFA3
P 3600 3150
F 0 "#PWR010" H 3600 3000 50  0001 C CNN
F 1 "+VSW" H 3600 3290 50  0000 C CNN
F 2 "" H 3600 3150 50  0001 C CNN
F 3 "" H 3600 3150 50  0001 C CNN
	1    3600 3150
	-1   0    0    1   
$EndComp
Wire Wire Line
	1700 2300 1700 2200
Wire Wire Line
	1300 2100 1850 2100
Wire Wire Line
	1300 2000 1400 2000
Wire Wire Line
	1300 1800 1400 1800
Wire Wire Line
	1300 1700 1400 1700
Wire Wire Line
	1300 1600 1400 1600
Wire Wire Line
	1300 1500 1400 1500
Wire Wire Line
	1300 1400 1400 1400
Wire Wire Line
	1300 1300 1400 1300
Wire Wire Line
	1300 1200 1400 1200
Wire Wire Line
	1300 1100 1400 1100
Wire Wire Line
	2450 1300 3200 1300
Wire Wire Line
	3500 1600 3500 1850
Wire Wire Line
	3750 1750 3500 1750
Connection ~ 3500 1750
Wire Wire Line
	4100 1100 4100 1750
Connection ~ 4100 1300
Wire Wire Line
	3500 2300 3500 2150
Wire Wire Line
	4100 1750 4050 1750
Connection ~ 1700 2100
Wire Wire Line
	1300 2200 1850 2200
Connection ~ 1700 2200
Wire Wire Line
	1700 2100 1700 2000
Wire Wire Line
	4000 3000 4000 3100
Wire Wire Line
	3100 1300 3100 1450
Connection ~ 3100 1300
Wire Wire Line
	3100 1750 3100 1900
Wire Wire Line
	4300 1750 4300 1900
Wire Wire Line
	4300 1300 4300 1450
Connection ~ 4300 1300
Wire Wire Line
	3600 3000 3600 3150
Wire Wire Line
	1300 1900 1400 1900
$Comp
L LED D1
U 1 1 5A6DA4F9
P 2700 2050
F 0 "D1" H 2700 2150 50  0000 C CNN
F 1 "LED" H 2700 1950 50  0000 C CNN
F 2 "LEDs:LED_0805_HandSoldering" H 2700 2050 50  0001 C CNN
F 3 "" H 2700 2050 50  0001 C CNN
	1    2700 2050
	0    -1   -1   0   
$EndComp
$Comp
L R R1
U 1 1 5A6DA938
P 2700 1650
F 0 "R1" V 2600 1650 50  0000 C CNN
F 1 "820" V 2700 1650 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 2630 1650 50  0001 C CNN
F 3 "" H 2700 1650 50  0001 C CNN
	1    2700 1650
	1    0    0    -1  
$EndComp
Wire Wire Line
	2700 1300 2700 1500
Wire Wire Line
	2700 1800 2700 1900
$Comp
L GND #PWR011
U 1 1 5A6DAFCB
P 2700 2300
F 0 "#PWR011" H 2700 2050 50  0001 C CNN
F 1 "GND" H 2700 2150 50  0000 C CNN
F 2 "" H 2700 2300 50  0001 C CNN
F 3 "" H 2700 2300 50  0001 C CNN
	1    2700 2300
	1    0    0    -1  
$EndComp
Wire Wire Line
	2700 2300 2700 2200
Wire Wire Line
	3800 1300 4600 1300
Connection ~ 2700 1300
Text Label 1400 2000 0    60   ~ 0
CS0
Text Label 1400 1700 0    60   ~ 0
SDI
Text Label 1400 1800 0    60   ~ 0
SDO
Text Label 1400 1900 0    60   ~ 0
SCK
$Sheet
S 7150 1200 600  700 
U 5A7CCC06
F0 "MAX31856Aux0" 60
F1 "MAX31856Aux.sch" 60
F2 "GND" I R 7750 1450 60 
F3 "VDD" I R 7750 1550 60 
F4 "SDI" I L 7150 1450 60 
F5 "SDO" O L 7150 1550 60 
F6 "SCK" I L 7150 1650 60 
F7 "CS" I R 7750 1650 60 
$EndSheet
$Sheet
S 9300 1150 600  700 
U 5A7D1AF6
F0 "MAX31856Aux1" 60
F1 "MAX31856Aux.sch" 60
F2 "GND" I R 9900 1400 60 
F3 "VDD" I R 9900 1500 60 
F4 "SDI" I L 9300 1400 60 
F5 "SDO" O L 9300 1500 60 
F6 "SCK" I L 9300 1600 60 
F7 "CS" I R 9900 1600 60 
$EndSheet
$Sheet
S 11200 1150 600  700 
U 5A7D2196
F0 "MAX31856Aux2" 60
F1 "MAX31856Aux.sch" 60
F2 "GND" I R 11800 1400 60 
F3 "VDD" I R 11800 1500 60 
F4 "SDI" I L 11200 1400 60 
F5 "SDO" O L 11200 1500 60 
F6 "SCK" I L 11200 1600 60 
F7 "CS" I R 11800 1600 60 
$EndSheet
$Sheet
S 7150 2600 600  700 
U 5A7D303A
F0 "MAX31856Aux4" 60
F1 "MAX31856Aux.sch" 60
F2 "GND" I R 7750 2850 60 
F3 "VDD" I R 7750 2950 60 
F4 "SDI" I L 7150 2850 60 
F5 "SDO" O L 7150 2950 60 
F6 "SCK" I L 7150 3050 60 
F7 "CS" I R 7750 3050 60 
$EndSheet
$Sheet
S 9300 2550 600  700 
U 5A7D3042
F0 "MAX31856Aux5" 60
F1 "MAX31856Aux.sch" 60
F2 "GND" I R 9900 2800 60 
F3 "VDD" I R 9900 2900 60 
F4 "SDI" I L 9300 2800 60 
F5 "SDO" O L 9300 2900 60 
F6 "SCK" I L 9300 3000 60 
F7 "CS" I R 9900 3000 60 
$EndSheet
$Sheet
S 11200 2550 600  700 
U 5A7D304A
F0 "MAX31856Aux6" 60
F1 "MAX31856Aux.sch" 60
F2 "GND" I R 11800 2800 60 
F3 "VDD" I R 11800 2900 60 
F4 "SDI" I L 11200 2800 60 
F5 "SDO" O L 11200 2900 60 
F6 "SCK" I L 11200 3000 60 
F7 "CS" I R 11800 3000 60 
$EndSheet
$Sheet
S 12850 1100 600  700 
U 5A7D676D
F0 "MAX31856Aux3" 60
F1 "MAX31856Aux.sch" 60
F2 "GND" I R 13450 1350 60 
F3 "VDD" I R 13450 1450 60 
F4 "SDI" I L 12850 1350 60 
F5 "SDO" O L 12850 1450 60 
F6 "SCK" I L 12850 1550 60 
F7 "CS" I R 13450 1550 60 
$EndSheet
Text Label 7000 1450 2    60   ~ 0
SDI
Text Label 9150 1400 2    60   ~ 0
SDI
Text Label 11000 1400 2    60   ~ 0
SDI
Text Label 12650 1350 2    60   ~ 0
SDI
Text Label 7000 2850 2    60   ~ 0
SDI
Text Label 9150 2800 2    60   ~ 0
SDI
Text Label 11050 2800 2    60   ~ 0
SDI
Text Label 7000 1550 2    60   ~ 0
SDO
Text Label 9150 1500 2    60   ~ 0
SDO
Text Label 11000 1500 2    60   ~ 0
SDO
Text Label 12650 1450 2    60   ~ 0
SDO
Text Label 7000 2950 2    60   ~ 0
SDO
Text Label 9150 2900 2    60   ~ 0
SDO
Text Label 11050 2900 2    60   ~ 0
SDO
Text Label 7000 1650 2    60   ~ 0
SCK
Text Label 9150 1600 2    60   ~ 0
SCK
Text Label 11000 1600 2    60   ~ 0
SCK
Text Label 12650 1550 2    60   ~ 0
SCK
Text Label 7000 3050 2    60   ~ 0
SCK
Text Label 9150 3000 2    60   ~ 0
SCK
Text Label 11050 3000 2    60   ~ 0
SCK
Text Label 7900 1450 0    60   ~ 0
GND
Text Label 10100 1400 0    60   ~ 0
GND
Text Label 12000 1400 0    60   ~ 0
GND
Text Label 13650 1350 0    60   ~ 0
GND
Text Label 7900 2850 0    60   ~ 0
GND
Text Label 10100 2800 0    60   ~ 0
GND
Text Label 12000 2800 0    60   ~ 0
GND
Text Label 7900 1550 0    60   ~ 0
VDD
Text Label 10100 1500 0    60   ~ 0
VDD
Text Label 12000 1500 0    60   ~ 0
VDD
Text Label 13650 1450 0    60   ~ 0
VDD
Text Label 7900 2950 0    60   ~ 0
VDD
Text Label 10100 2900 0    60   ~ 0
VDD
Text Label 12000 2900 0    60   ~ 0
VDD
Text Label 7900 1650 0    60   ~ 0
CS0
Text Label 10100 1600 0    60   ~ 0
CS1
Text Label 12000 1600 0    60   ~ 0
CS2
Text Label 13650 1550 0    60   ~ 0
CS3
Text Label 7900 3050 0    60   ~ 0
CS4
Text Label 10100 3000 0    60   ~ 0
CS5
Text Label 12000 3000 0    60   ~ 0
CS6
Wire Wire Line
	7000 1450 7150 1450
Wire Wire Line
	7000 1550 7150 1550
Wire Wire Line
	7000 1650 7150 1650
Wire Wire Line
	7750 1450 7900 1450
Wire Wire Line
	7750 1550 7900 1550
Wire Wire Line
	7750 1650 7900 1650
Wire Wire Line
	11000 1400 11200 1400
Wire Wire Line
	11000 1500 11200 1500
Wire Wire Line
	11000 1600 11200 1600
Wire Wire Line
	11800 1400 12000 1400
Wire Wire Line
	11800 1500 12000 1500
Wire Wire Line
	11800 1600 12000 1600
Wire Wire Line
	12650 1350 12850 1350
Wire Wire Line
	12650 1450 12850 1450
Wire Wire Line
	12650 1550 12850 1550
Wire Wire Line
	13650 1350 13450 1350
Wire Wire Line
	13650 1450 13450 1450
Wire Wire Line
	13650 1550 13450 1550
Wire Wire Line
	7000 2850 7150 2850
Wire Wire Line
	7000 2950 7150 2950
Wire Wire Line
	7000 3050 7150 3050
Wire Wire Line
	7750 2850 7900 2850
Wire Wire Line
	7750 2950 7900 2950
Wire Wire Line
	7750 3050 7900 3050
Wire Wire Line
	9150 2800 9300 2800
Wire Wire Line
	9150 2900 9300 2900
Wire Wire Line
	9150 3000 9300 3000
Wire Wire Line
	10100 2800 9900 2800
Wire Wire Line
	10100 2900 9900 2900
Wire Wire Line
	10100 3000 9900 3000
Wire Wire Line
	11050 2800 11200 2800
Wire Wire Line
	11050 2900 11200 2900
Wire Wire Line
	11050 3000 11200 3000
Wire Wire Line
	11800 2800 12000 2800
Wire Wire Line
	11800 2900 12000 2900
Wire Wire Line
	11800 3000 12000 3000
Wire Wire Line
	9150 1400 9300 1400
Wire Wire Line
	9150 1500 9300 1500
Wire Wire Line
	9150 1600 9300 1600
Wire Wire Line
	9900 1400 10100 1400
Wire Wire Line
	9900 1500 10100 1500
Wire Wire Line
	9900 1600 10100 1600
$EndSCHEMATC
