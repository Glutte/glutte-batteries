EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 2 3
Title "Coulomb meter for Glutte Battery"
Date "2019-10-18"
Rev ""
Comp "HB9EGM"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L glutte-coulombcounter:LT1236 U2
U 1 1 5D86AC0C
P 3750 2250
F 0 "U2" H 3750 2715 50  0000 C CNN
F 1 "LT1236-5" H 3750 2624 50  0000 C CNN
F 2 "Package_SO:SO-8_3.9x4.9mm_P1.27mm" H 3750 2600 50  0001 C CNN
F 3 "" H 3650 2250 50  0001 C CNN
F 4 "LT1236AIS8-5" H 3750 2250 50  0001 C CNN "MPN"
	1    3750 2250
	1    0    0    -1  
$EndComp
$Comp
L power:GNDA #PWR0108
U 1 1 5D86AC19
P 3750 2750
F 0 "#PWR0108" H 3750 2500 50  0001 C CNN
F 1 "GNDA" H 3755 2577 50  0000 C CNN
F 2 "" H 3750 2750 50  0001 C CNN
F 3 "" H 3750 2750 50  0001 C CNN
	1    3750 2750
	1    0    0    -1  
$EndComp
$Comp
L power:GNDA #PWR0112
U 1 1 5D86AC1F
P 2450 2550
F 0 "#PWR0112" H 2450 2300 50  0001 C CNN
F 1 "GNDA" H 2455 2377 50  0000 C CNN
F 2 "" H 2450 2550 50  0001 C CNN
F 3 "" H 2450 2550 50  0001 C CNN
	1    2450 2550
	1    0    0    -1  
$EndComp
Text GLabel 4550 2250 2    50   Input ~ 0
VREF
NoConn ~ 4450 2350
Text Notes 2000 1300 0    100  ~ 0
POWER
Text Notes 4350 2000 0    50   ~ 0
5V Reference ADC
$Comp
L glutte-coulombcounter:LT3433 U3
U 1 1 5D86AC2A
P 3750 4000
F 0 "U3" H 3750 4815 50  0000 C CNN
F 1 "LT3433" H 3750 4724 50  0000 C CNN
F 2 "Package_SO:TSSOP-16-1EP_4.4x5mm_P0.65mm" H 3700 4000 50  0001 C CNN
F 3 "" H 3700 4000 50  0001 C CNN
F 4 "LT3433IFE" H 3750 4000 50  0001 C CNN "MPN"
	1    3750 4000
	1    0    0    -1  
$EndComp
$Comp
L Device:CP1 C1
U 1 1 5D86AC30
P 2200 3150
F 0 "C1" H 2315 3196 50  0000 L CNN
F 1 "2.2u" H 2315 3105 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 2200 3150 50  0001 C CNN
F 3 "~" H 2200 3150 50  0001 C CNN
	1    2200 3150
	1    0    0    -1  
$EndComp
Connection ~ 2650 2250
Wire Wire Line
	2650 2250 2950 2250
Wire Wire Line
	2650 3000 2200 3000
Wire Wire Line
	2650 2250 2650 3000
Wire Wire Line
	2650 3000 2650 3450
Wire Wire Line
	2650 3450 3100 3450
Connection ~ 2650 3000
$Comp
L power:GNDA #PWR0121
U 1 1 5D86AC3D
P 2200 3300
F 0 "#PWR0121" H 2200 3050 50  0001 C CNN
F 1 "GNDA" H 2205 3127 50  0000 C CNN
F 2 "" H 2200 3300 50  0001 C CNN
F 3 "" H 2200 3300 50  0001 C CNN
	1    2200 3300
	1    0    0    -1  
$EndComp
Connection ~ 3650 4950
Wire Wire Line
	3650 4950 3700 4950
Connection ~ 3750 4950
Wire Wire Line
	3750 4950 3850 4950
Connection ~ 3850 4950
Wire Wire Line
	3850 4950 3950 4950
$Comp
L power:GNDA #PWR0122
U 1 1 5D86AC49
P 3700 5050
F 0 "#PWR0122" H 3700 4800 50  0001 C CNN
F 1 "GNDA" H 3705 4877 50  0000 C CNN
F 2 "" H 3700 5050 50  0001 C CNN
F 3 "" H 3700 5050 50  0001 C CNN
	1    3700 5050
	1    0    0    -1  
$EndComp
Wire Wire Line
	3700 5050 3700 4950
$Comp
L Device:C C2
U 1 1 5D86AC50
P 2200 4050
F 0 "C2" H 2315 4096 50  0000 L CNN
F 1 "330p" H 2315 4005 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 2238 3900 50  0001 C CNN
F 3 "~" H 2200 4050 50  0001 C CNN
	1    2200 4050
	1    0    0    -1  
$EndComp
$Comp
L Device:R R1
U 1 1 5D86AC56
P 2550 4050
F 0 "R1" H 2480 4004 50  0000 R CNN
F 1 "68k" H 2480 4095 50  0000 R CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 2480 4050 50  0001 C CNN
F 3 "~" H 2550 4050 50  0001 C CNN
	1    2550 4050
	-1   0    0    1   
$EndComp
$Comp
L power:GNDA #PWR0123
U 1 1 5D86AC5C
P 2200 4500
F 0 "#PWR0123" H 2200 4250 50  0001 C CNN
F 1 "GNDA" H 2205 4327 50  0000 C CNN
F 2 "" H 2200 4500 50  0001 C CNN
F 3 "" H 2200 4500 50  0001 C CNN
	1    2200 4500
	1    0    0    -1  
$EndComp
$Comp
L Device:C C3
U 1 1 5D86AC62
P 2550 4350
F 0 "C3" H 2665 4396 50  0000 L CNN
F 1 "1000p" H 2665 4305 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 2588 4200 50  0001 C CNN
F 3 "~" H 2550 4350 50  0001 C CNN
	1    2550 4350
	1    0    0    -1  
$EndComp
Wire Wire Line
	2550 4500 2200 4500
Wire Wire Line
	2200 4200 2200 4500
Connection ~ 2200 4500
Wire Wire Line
	2200 3900 2550 3900
Connection ~ 2550 3900
Wire Wire Line
	2550 3900 3100 3900
$Comp
L Device:R R4
U 1 1 5D86AC6E
P 5500 4500
F 0 "R4" H 5430 4454 50  0000 R CNN
F 1 "97.6k" H 5430 4545 50  0000 R CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 5430 4500 50  0001 C CNN
F 3 "~" H 5500 4500 50  0001 C CNN
	1    5500 4500
	-1   0    0    1   
$EndComp
$Comp
L power:GNDA #PWR0124
U 1 1 5D86AC74
P 5500 4650
F 0 "#PWR0124" H 5500 4400 50  0001 C CNN
F 1 "GNDA" H 5505 4477 50  0000 C CNN
F 2 "" H 5500 4650 50  0001 C CNN
F 3 "" H 5500 4650 50  0001 C CNN
	1    5500 4650
	1    0    0    -1  
$EndComp
$Comp
L Device:R R3
U 1 1 5D86AC7A
P 5500 4200
F 0 "R3" H 5430 4154 50  0000 R CNN
F 1 "300k" H 5430 4245 50  0000 R CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 5430 4200 50  0001 C CNN
F 3 "~" H 5500 4200 50  0001 C CNN
	1    5500 4200
	-1   0    0    1   
$EndComp
Wire Wire Line
	5500 4050 5500 4000
$Comp
L Device:L L1
U 1 1 5D86AC81
P 4700 3850
F 0 "L1" V 4519 3850 50  0000 C CNN
F 1 "100uH" V 4610 3850 50  0000 C CNN
F 2 "glutte-coulombcounter:Sumida_CDRH6D28" H 4700 3850 50  0001 C CNN
F 3 "~" H 4700 3850 50  0001 C CNN
F 4 "CDRH6D28-101" V 4700 3850 50  0001 C CNN "MPN"
	1    4700 3850
	0    1    1    0   
$EndComp
Wire Wire Line
	4400 3850 4450 3850
Wire Wire Line
	4400 3750 4450 3750
Wire Wire Line
	4850 3750 4850 3850
$Comp
L Device:C C9
U 1 1 5D86AC8A
P 5000 3750
F 0 "C9" V 4748 3750 50  0000 C CNN
F 1 "0.1u" V 4839 3750 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 5038 3600 50  0001 C CNN
F 3 "~" H 5000 3750 50  0001 C CNN
	1    5000 3750
	0    1    1    0   
$EndComp
Connection ~ 4850 3750
Wire Wire Line
	5150 3750 5150 3600
Wire Wire Line
	5150 3600 4400 3600
$Comp
L Device:D_ALT D2
U 1 1 5D86AC93
P 5050 4150
F 0 "D2" H 5050 4366 50  0000 C CNN
F 1 "1N4148" H 5050 4275 50  0000 C CNN
F 2 "Diode_SMD:D_SOD-123" H 5050 4150 50  0001 C CNN
F 3 "~" H 5050 4150 50  0001 C CNN
F 4 "1N4148W-7-F" H 5050 4150 50  0001 C CNN "MPN"
	1    5050 4150
	1    0    0    -1  
$EndComp
$Comp
L Device:C C8
U 1 1 5D86AC99
P 4800 4300
F 0 "C8" H 4685 4254 50  0000 R CNN
F 1 "0.1u" H 4685 4345 50  0000 R CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 4838 4150 50  0001 C CNN
F 3 "~" H 4800 4300 50  0001 C CNN
	1    4800 4300
	-1   0    0    1   
$EndComp
$Comp
L power:GNDA #PWR0125
U 1 1 5D86AC9F
P 4800 4450
F 0 "#PWR0125" H 4800 4200 50  0001 C CNN
F 1 "GNDA" H 4805 4277 50  0000 C CNN
F 2 "" H 4800 4450 50  0001 C CNN
F 3 "" H 4800 4450 50  0001 C CNN
	1    4800 4450
	1    0    0    -1  
$EndComp
Wire Wire Line
	4400 4000 5300 4000
Wire Wire Line
	4400 4150 4800 4150
Connection ~ 4800 4150
Wire Wire Line
	4800 4150 4900 4150
Wire Wire Line
	5200 4150 5300 4150
Wire Wire Line
	5300 4150 5300 4000
Connection ~ 5300 4000
Wire Wire Line
	5300 4000 5500 4000
Wire Wire Line
	5500 4350 5250 4350
Wire Wire Line
	5250 4350 5250 4750
Wire Wire Line
	5250 4750 4400 4750
Wire Wire Line
	4400 4750 4400 4350
Connection ~ 5500 4350
Wire Wire Line
	4450 3850 4450 3900
Wire Wire Line
	4450 3900 5450 3900
Connection ~ 4450 3850
Wire Wire Line
	4450 3850 4550 3850
Wire Wire Line
	5500 4000 5750 4000
Wire Wire Line
	5750 4000 5750 3900
Connection ~ 5500 4000
$Comp
L Device:D_ALT D4
U 1 1 5D86ACB9
P 5950 3750
F 0 "D4" V 5904 3829 50  0000 L CNN
F 1 "1N4148" V 5995 3829 50  0000 L CNN
F 2 "Diode_SMD:D_SOD-123" H 5950 3750 50  0001 C CNN
F 3 "~" H 5950 3750 50  0001 C CNN
F 4 "1N4148W-7-F" H 5950 3750 50  0001 C CNN "MPN"
	1    5950 3750
	0    1    1    0   
$EndComp
Wire Wire Line
	5750 4000 5950 4000
Wire Wire Line
	5950 4000 5950 3900
Connection ~ 5750 4000
Wire Wire Line
	5950 3600 5150 3600
Connection ~ 5150 3600
Wire Wire Line
	4450 3350 4450 3750
Connection ~ 4450 3750
Wire Wire Line
	4450 3750 4850 3750
$Comp
L power:GNDA #PWR0126
U 1 1 5D86ACC7
P 4750 3350
F 0 "#PWR0126" H 4750 3100 50  0001 C CNN
F 1 "GNDA" H 4755 3177 50  0000 C CNN
F 2 "" H 4750 3350 50  0001 C CNN
F 3 "" H 4750 3350 50  0001 C CNN
	1    4750 3350
	1    0    0    -1  
$EndComp
Wire Wire Line
	3100 3600 3100 3450
Connection ~ 3100 3450
$Comp
L power:GNDA #PWR0127
U 1 1 5D86ACCF
P 3100 4350
F 0 "#PWR0127" H 3100 4100 50  0001 C CNN
F 1 "GNDA" H 3105 4177 50  0000 C CNN
F 2 "" H 3100 4350 50  0001 C CNN
F 3 "" H 3100 4350 50  0001 C CNN
	1    3100 4350
	1    0    0    -1  
$EndComp
$Comp
L Device:C C4
U 1 1 5D86ACD5
P 2800 3750
F 0 "C4" V 2548 3750 50  0000 C CNN
F 1 "0.01u" V 2639 3750 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 2838 3600 50  0001 C CNN
F 3 "~" H 2800 3750 50  0001 C CNN
	1    2800 3750
	0    1    1    0   
$EndComp
Wire Wire Line
	2950 3750 3100 3750
$Comp
L power:GNDA #PWR0128
U 1 1 5D86ACDC
P 2650 3750
F 0 "#PWR0128" H 2650 3500 50  0001 C CNN
F 1 "GNDA" V 2655 3623 50  0000 R CNN
F 2 "" H 2650 3750 50  0001 C CNN
F 3 "" H 2650 3750 50  0001 C CNN
	1    2650 3750
	0    1    1    0   
$EndComp
$Comp
L power:+5VA #PWR0129
U 1 1 5D86ACE2
P 6450 4000
F 0 "#PWR0129" H 6450 3850 50  0001 C CNN
F 1 "+5VA" H 6465 4173 50  0000 C CNN
F 2 "" H 6450 4000 50  0001 C CNN
F 3 "" H 6450 4000 50  0001 C CNN
	1    6450 4000
	1    0    0    -1  
$EndComp
Wire Wire Line
	5950 4000 6150 4000
Connection ~ 5950 4000
Text Notes 4350 3050 0    50   ~ 0
5V Alimentation
$Comp
L Device:CP1 C5
U 1 1 5D86AD01
P 2950 2400
F 0 "C5" H 3065 2446 50  0000 L CNN
F 1 "0.1u" H 3065 2355 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 2950 2400 50  0001 C CNN
F 3 "~" H 2950 2400 50  0001 C CNN
	1    2950 2400
	1    0    0    -1  
$EndComp
Connection ~ 2950 2250
Wire Wire Line
	2950 2250 3050 2250
$Comp
L power:GNDA #PWR0130
U 1 1 5D86AD09
P 2950 2550
F 0 "#PWR0130" H 2950 2300 50  0001 C CNN
F 1 "GNDA" H 2955 2377 50  0000 C CNN
F 2 "" H 2950 2550 50  0001 C CNN
F 3 "" H 2950 2550 50  0001 C CNN
	1    2950 2550
	1    0    0    -1  
$EndComp
Wire Wire Line
	3550 4950 3650 4950
Wire Wire Line
	3450 4950 3550 4950
Connection ~ 3550 4950
Connection ~ 3700 4950
Wire Wire Line
	3700 4950 3750 4950
$Comp
L Device:LED_ALT D6
U 1 1 5D86AD14
P 6150 4350
F 0 "D6" V 6189 4232 50  0000 R CNN
F 1 "LED_PWR" V 6098 4232 50  0000 R CNN
F 2 "LED_SMD:LED_1206_3216Metric" H 6150 4350 50  0001 C CNN
F 3 "~" H 6150 4350 50  0001 C CNN
	1    6150 4350
	0    -1   -1   0   
$EndComp
Wire Wire Line
	6150 4200 6150 4000
Connection ~ 6150 4000
Wire Wire Line
	6150 4000 6450 4000
$Comp
L Device:R R12
U 1 1 5D86AD1D
P 6150 4700
F 0 "R12" H 6080 4654 50  0000 R CNN
F 1 "330" H 6080 4745 50  0000 R CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 6080 4700 50  0001 C CNN
F 3 "~" H 6150 4700 50  0001 C CNN
	1    6150 4700
	-1   0    0    1   
$EndComp
$Comp
L power:GNDA #PWR0131
U 1 1 5D86AD23
P 6150 4900
F 0 "#PWR0131" H 6150 4650 50  0001 C CNN
F 1 "GNDA" H 6155 4727 50  0000 C CNN
F 2 "" H 6150 4900 50  0001 C CNN
F 3 "" H 6150 4900 50  0001 C CNN
	1    6150 4900
	1    0    0    -1  
$EndComp
Wire Wire Line
	6150 4500 6150 4550
Wire Wire Line
	6150 4850 6150 4900
$Comp
L power:+12V #PWR0132
U 1 1 5D86AD2B
P 2950 2250
F 0 "#PWR0132" H 2950 2100 50  0001 C CNN
F 1 "+12V" H 2965 2423 50  0000 C CNN
F 2 "" H 2950 2250 50  0001 C CNN
F 3 "" H 2950 2250 50  0001 C CNN
	1    2950 2250
	1    0    0    -1  
$EndComp
$Comp
L Diode:B160-E3 D1
U 1 1 5D86AD31
P 4600 3350
F 0 "D1" H 4600 3566 50  0000 C CNN
F 1 "B160-E3" H 4600 3475 50  0000 C CNN
F 2 "Diode_SMD:D_SMA" H 4600 3175 50  0001 C CNN
F 3 "http://www.vishay.com/docs/88946/b120.pdf" H 4600 3350 50  0001 C CNN
F 4 "B160-E3/5AT" H 4600 3350 50  0001 C CNN "MPN"
	1    4600 3350
	1    0    0    -1  
$EndComp
$Comp
L Diode:B120-E3 D3
U 1 1 5D86AD37
P 5600 3900
F 0 "D3" H 5600 3684 50  0000 C CNN
F 1 "B120-E3" H 5600 3775 50  0000 C CNN
F 2 "Diode_SMD:D_SMA" H 5600 3725 50  0001 C CNN
F 3 "http://www.vishay.com/docs/88946/b120.pdf" H 5600 3900 50  0001 C CNN
F 4 "B120-E3/5AT " H 5600 3900 50  0001 C CNN "MPN"
	1    5600 3900
	-1   0    0    1   
$EndComp
Text HLabel 2300 2250 0    50   Input ~ 0
IN_+12V
Wire Wire Line
	2300 2250 2650 2250
Text HLabel 2300 2550 0    50   Input ~ 0
IN_GND
Wire Wire Line
	2300 2550 2450 2550
Text Notes 2000 1500 0    50   ~ 0
Provides VREF and +5VA
Wire Wire Line
	4450 2250 4550 2250
$EndSCHEMATC
