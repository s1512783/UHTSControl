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
$Descr A4 11693 8268
encoding utf-8
Sheet 2 2
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Connection ~ 7300 4100
Wire Wire Line
	7300 4500 7300 4400
Wire Wire Line
	5700 4100 5700 4200
Wire Wire Line
	7150 4100 7450 4100
Connection ~ 5700 3800
Wire Wire Line
	5600 3800 5850 3800
Wire Wire Line
	5350 3450 5350 3500
Wire Wire Line
	5350 3750 5350 3650
Wire Wire Line
	5350 3650 5850 3650
Wire Wire Line
	5350 3500 5850 3500
Wire Wire Line
	7150 3200 7250 3200
Wire Wire Line
	5850 3200 5750 3200
Wire Wire Line
	3950 3350 5850 3350
Connection ~ 5050 3750
Wire Wire Line
	4800 3750 5350 3750
Connection ~ 5050 3450
Wire Wire Line
	4800 3450 5350 3450
Wire Wire Line
	3950 3350 3950 3550
Connection ~ 4700 3500
Wire Wire Line
	4700 3500 4700 3800
Connection ~ 3950 3500
Wire Wire Line
	4400 4100 4400 4200
Connection ~ 4400 3500
Wire Wire Line
	4800 3500 4800 3450
Connection ~ 4400 3700
Wire Wire Line
	4800 3700 4800 3750
Wire Wire Line
	4700 4100 4700 4200
Wire Wire Line
	4400 3700 4400 3800
Wire Wire Line
	4300 3700 4800 3700
Wire Wire Line
	4300 3500 4800 3500
Wire Wire Line
	3950 3700 4000 3700
Wire Wire Line
	3950 3650 3950 3700
Wire Wire Line
	3850 3650 3950 3650
Wire Wire Line
	3950 3500 4000 3500
Wire Wire Line
	3950 3550 3850 3550
NoConn ~ 5850 4100
NoConn ~ 7150 3350
Text Label 7150 3950 0    60   ~ 0
CS0
Text Label 7150 3800 0    60   ~ 0
SCK
Text Label 7150 3650 0    60   ~ 0
SDO
Text Label 7150 3500 0    60   ~ 0
SDI
Text Label 7450 4100 0    60   ~ 0
VDD
$Comp
L GND #PWR?
U 1 1 5A6F2AE0
P 7300 4500
F 0 "#PWR?" H 7300 4250 50  0001 C CNN
F 1 "GND" H 7300 4350 50  0000 C CNN
F 2 "" H 7300 4500 50  0001 C CNN
F 3 "" H 7300 4500 50  0001 C CNN
	1    7300 4500
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5A6F2AE6
P 5700 4200
F 0 "#PWR?" H 5700 3950 50  0001 C CNN
F 1 "GND" H 5700 4050 50  0000 C CNN
F 2 "" H 5700 4200 50  0001 C CNN
F 3 "" H 5700 4200 50  0001 C CNN
	1    5700 4200
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5A6F2AEC
P 7300 4250
F 0 "C?" H 7325 4350 50  0000 L CNN
F 1 "0.1u" H 7325 4150 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 7338 4100 50  0001 C CNN
F 3 "" H 7300 4250 50  0001 C CNN
	1    7300 4250
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5A6F2AF3
P 5700 3950
F 0 "C?" H 5725 4050 50  0000 L CNN
F 1 "0.1u" H 5725 3850 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 5738 3800 50  0001 C CNN
F 3 "" H 5700 3950 50  0001 C CNN
	1    5700 3950
	1    0    0    -1  
$EndComp
Text Label 5600 3800 2    60   ~ 0
VDD
Text Label 7250 3200 0    60   ~ 0
GND
Text Label 5750 3200 2    60   ~ 0
GND
$Comp
L GND #PWR?
U 1 1 5A6F2AFD
P 4400 4200
F 0 "#PWR?" H 4400 3950 50  0001 C CNN
F 1 "GND" H 4400 4050 50  0000 C CNN
F 2 "" H 4400 4200 50  0001 C CNN
F 3 "" H 4400 4200 50  0001 C CNN
	1    4400 4200
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5A6F2B03
P 4700 4200
F 0 "#PWR?" H 4700 3950 50  0001 C CNN
F 1 "GND" H 4700 4050 50  0000 C CNN
F 2 "" H 4700 4200 50  0001 C CNN
F 3 "" H 4700 4200 50  0001 C CNN
	1    4700 4200
	1    0    0    -1  
$EndComp
$Comp
L Conn_01x02 J?
U 1 1 5A6F2B09
P 3650 3650
F 0 "J?" H 3650 3750 50  0000 C CNN
F 1 "Conn_01x02" H 3650 3450 50  0000 C CNN
F 2 "UHTS:RSPro2-wayScrewTerminalPCBMount" H 3650 3650 50  0001 C CNN
F 3 "" H 3650 3650 50  0001 C CNN
	1    3650 3650
	-1   0    0    1   
$EndComp
$Comp
L R R?
U 1 1 5A6F2B10
P 4150 3700
F 0 "R?" V 4230 3700 50  0000 C CNN
F 1 "100" V 4150 3700 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 4080 3700 50  0001 C CNN
F 3 "" H 4150 3700 50  0001 C CNN
	1    4150 3700
	0    1    1    0   
$EndComp
$Comp
L R R?
U 1 1 5A6F2B17
P 4150 3500
F 0 "R?" V 4230 3500 50  0000 C CNN
F 1 "100" V 4150 3500 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 4080 3500 50  0001 C CNN
F 3 "" H 4150 3500 50  0001 C CNN
	1    4150 3500
	0    1    1    0   
$EndComp
$Comp
L C C?
U 1 1 5A6F2B1E
P 4700 3950
F 0 "C?" H 4725 4050 50  0000 L CNN
F 1 "0.01u" H 4725 3850 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 4738 3800 50  0001 C CNN
F 3 "" H 4700 3950 50  0001 C CNN
	1    4700 3950
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5A6F2B25
P 4400 3950
F 0 "C?" H 4425 4050 50  0000 L CNN
F 1 "0.01u" H 4425 3850 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 4438 3800 50  0001 C CNN
F 3 "" H 4400 3950 50  0001 C CNN
	1    4400 3950
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5A6F2B2C
P 5050 3600
F 0 "C?" H 5075 3700 50  0000 L CNN
F 1 "0.1u" H 5075 3500 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 5088 3450 50  0001 C CNN
F 3 "" H 5050 3600 50  0001 C CNN
	1    5050 3600
	1    0    0    -1  
$EndComp
$Comp
L MAX31856MUD+ U?
U 1 1 5A6F2B33
P 6500 3650
F 0 "U?" H 6500 2950 60  0000 C CNN
F 1 "MAX31856MUD+" V 6500 3650 60  0000 C CNN
F 2 "Housings_SSOP:TSSOP-14_4.4x5mm_Pitch0.65mm" H 6450 3800 60  0001 C CNN
F 3 "" H 6450 3800 60  0001 C CNN
	1    6500 3650
	1    0    0    -1  
$EndComp
$EndSCHEMATC