EESchema Schematic File Version 4
LIBS:glutte-coulombcounter-cache
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 3 3
Title ""
Date "2019-10-18"
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Connector_Generic:Conn_01x06 J?
U 1 1 5D969296
P 6950 2800
AR Path="/5D969296" Ref="J?"  Part="1" 
AR Path="/5D96189F/5D969296" Ref="J3"  Part="1" 
F 0 "J3" H 7030 2792 50  0000 L CNN
F 1 "K1" H 7030 2701 50  0000 L CNN
F 2 "TerminalBlock_Phoenix:TerminalBlock_Phoenix_MPT-0,5-6-2.54_1x06_P2.54mm_Horizontal" H 6950 2800 50  0001 C CNN
F 3 "~" H 6950 2800 50  0001 C CNN
	1    6950 2800
	1    0    0    -1  
$EndComp
$Comp
L Transistor_Array:ULN2003A U?
U 1 1 5D96929C
P 3200 2400
AR Path="/5D96929C" Ref="U?"  Part="1" 
AR Path="/5D96189F/5D96929C" Ref="U9"  Part="1" 
F 0 "U9" H 3200 3067 50  0000 C CNN
F 1 "ULN2003A" H 3200 2976 50  0000 C CNN
F 2 "Package_DIP:DIP-16_W7.62mm_Socket" H 3250 1850 50  0001 L CNN
F 3 "http://www.ti.com/lit/ds/symlink/uln2003a.pdf" H 3300 2200 50  0001 C CNN
	1    3200 2400
	1    0    0    -1  
$EndComp
$Comp
L power:GNDA #PWR?
U 1 1 5D9692A2
P 3200 3000
AR Path="/5D9692A2" Ref="#PWR?"  Part="1" 
AR Path="/5D96189F/5D9692A2" Ref="#PWR015"  Part="1" 
F 0 "#PWR015" H 3200 2750 50  0001 C CNN
F 1 "GNDA" H 3205 2827 50  0000 C CNN
F 2 "" H 3200 3000 50  0001 C CNN
F 3 "" H 3200 3000 50  0001 C CNN
	1    3200 3000
	1    0    0    -1  
$EndComp
$Comp
L power:+5VA #PWR?
U 1 1 5D9692A8
P 3600 2000
AR Path="/5D9692A8" Ref="#PWR?"  Part="1" 
AR Path="/5D96189F/5D9692A8" Ref="#PWR016"  Part="1" 
F 0 "#PWR016" H 3600 1850 50  0001 C CNN
F 1 "+5VA" V 3615 2127 50  0000 L CNN
F 2 "" H 3600 2000 50  0001 C CNN
F 3 "" H 3600 2000 50  0001 C CNN
	1    3600 2000
	0    1    1    0   
$EndComp
$Comp
L Relay:G6SK-2 K?
U 1 1 5D9692AE
P 4900 2850
AR Path="/5D9692AE" Ref="K?"  Part="1" 
AR Path="/5D96189F/5D9692AE" Ref="K1"  Part="1" 
F 0 "K1" H 5730 2896 50  0000 L CNN
F 1 "G6SK-2F-DC5" H 5730 2805 50  0000 L CNN
F 2 "Relay_SMD:Relay_DPDT_Omron_G6SK-2F" H 5450 2800 50  0001 L CNN
F 3 "http://omronfs.omron.com/en_US/ecb/products/pdf/en-g6s.pdf" H 4600 2850 50  0001 C CNN
	1    4900 2850
	1    0    0    -1  
$EndComp
$Comp
L Relay:G6SK-2 K?
U 1 1 5D9692C4
P 4900 4200
AR Path="/5D9692C4" Ref="K?"  Part="1" 
AR Path="/5D96189F/5D9692C4" Ref="K2"  Part="1" 
F 0 "K2" H 5730 4246 50  0000 L CNN
F 1 "G6SK-2F-DC5" H 5730 4155 50  0000 L CNN
F 2 "Relay_SMD:Relay_DPDT_Omron_G6SK-2F" H 5450 4150 50  0001 L CNN
F 3 "http://omronfs.omron.com/en_US/ecb/products/pdf/en-g6s.pdf" H 4600 4200 50  0001 C CNN
	1    4900 4200
	1    0    0    -1  
$EndComp
$Comp
L Relay:G6SK-2 K?
U 1 1 5D9692DE
P 4900 5450
AR Path="/5D9692DE" Ref="K?"  Part="1" 
AR Path="/5D96189F/5D9692DE" Ref="K3"  Part="1" 
F 0 "K3" H 5730 5496 50  0000 L CNN
F 1 "G6SK-2F-DC5" H 5730 5405 50  0000 L CNN
F 2 "Relay_SMD:Relay_DPDT_Omron_G6SK-2F" H 5450 5400 50  0001 L CNN
F 3 "http://omronfs.omron.com/en_US/ecb/products/pdf/en-g6s.pdf" H 4600 5450 50  0001 C CNN
	1    4900 5450
	1    0    0    -1  
$EndComp
Text HLabel 2800 2300 0    50   Input ~ 0
K1_SET
Text HLabel 2800 2200 0    50   Input ~ 0
K1_RESET
Text HLabel 2800 2400 0    50   Input ~ 0
K2_RESET
Text HLabel 2800 2500 0    50   Input ~ 0
K2_SET
Text HLabel 2800 2600 0    50   Input ~ 0
K3_RESET
Text HLabel 2800 2700 0    50   Input ~ 0
K3_SET
$Comp
L power:GNDA #PWR?
U 1 1 5D96A87D
P 2650 3000
AR Path="/5D96A87D" Ref="#PWR?"  Part="1" 
AR Path="/5D96189F/5D96A87D" Ref="#PWR05"  Part="1" 
F 0 "#PWR05" H 2650 2750 50  0001 C CNN
F 1 "GNDA" H 2655 2827 50  0000 C CNN
F 2 "" H 2650 3000 50  0001 C CNN
F 3 "" H 2650 3000 50  0001 C CNN
	1    2650 3000
	1    0    0    -1  
$EndComp
Wire Wire Line
	2800 2800 2650 2800
Wire Wire Line
	2650 2800 2650 3000
NoConn ~ 3600 2800
Text Notes 4500 1900 0    50   ~ 0
min 10ms SET/RESET pulse width\nCan switch 2A
Wire Wire Line
	4800 3150 4800 3350
Wire Wire Line
	4800 3350 6750 3350
Wire Wire Line
	6750 3350 6750 3100
Wire Wire Line
	6750 2800 6400 2800
Wire Wire Line
	6400 2800 6400 3300
Wire Wire Line
	6400 3300 5200 3300
Wire Wire Line
	5200 3300 5200 3150
Wire Wire Line
	5300 2550 5300 2500
Wire Wire Line
	5300 2500 6400 2500
Wire Wire Line
	6400 2500 6400 2700
Wire Wire Line
	6400 2700 6750 2700
Wire Wire Line
	6750 2600 6500 2600
Wire Wire Line
	6500 2600 6500 2400
Wire Wire Line
	6500 2400 5100 2400
Wire Wire Line
	5100 2400 5100 2550
Wire Wire Line
	4900 2550 4900 2300
Wire Wire Line
	4900 2300 6600 2300
Wire Wire Line
	6600 2300 6600 3000
Wire Wire Line
	6600 3000 6750 3000
Wire Wire Line
	6750 2900 6700 2900
Wire Wire Line
	6700 2900 6700 2250
Wire Wire Line
	6700 2250 4700 2250
Wire Wire Line
	4700 2250 4700 2550
$Comp
L Connector_Generic:Conn_01x06 J?
U 1 1 5D9845EF
P 6950 4150
AR Path="/5D9845EF" Ref="J?"  Part="1" 
AR Path="/5D96189F/5D9845EF" Ref="J6"  Part="1" 
F 0 "J6" H 7030 4142 50  0000 L CNN
F 1 "K2" H 7030 4051 50  0000 L CNN
F 2 "TerminalBlock_Phoenix:TerminalBlock_Phoenix_MPT-0,5-6-2.54_1x06_P2.54mm_Horizontal" H 6950 4150 50  0001 C CNN
F 3 "~" H 6950 4150 50  0001 C CNN
	1    6950 4150
	1    0    0    -1  
$EndComp
Wire Wire Line
	4800 4700 6750 4700
Wire Wire Line
	6750 4700 6750 4450
Wire Wire Line
	6750 4150 6400 4150
Wire Wire Line
	6400 4150 6400 4650
Wire Wire Line
	6400 4650 5200 4650
Wire Wire Line
	5300 3850 6400 3850
Wire Wire Line
	6400 3850 6400 4050
Wire Wire Line
	6400 4050 6750 4050
Wire Wire Line
	6750 3950 6500 3950
Wire Wire Line
	6500 3950 6500 3750
Wire Wire Line
	6500 3750 5100 3750
Wire Wire Line
	4900 3650 6600 3650
Wire Wire Line
	6600 3650 6600 4350
Wire Wire Line
	6600 4350 6750 4350
Wire Wire Line
	6750 4250 6700 4250
Wire Wire Line
	6700 4250 6700 3600
Wire Wire Line
	6700 3600 4700 3600
Wire Wire Line
	4800 4700 4800 4500
Wire Wire Line
	5200 4500 5200 4650
Wire Wire Line
	5300 3900 5300 3850
Wire Wire Line
	5100 3750 5100 3900
Wire Wire Line
	4900 3900 4900 3650
Wire Wire Line
	4700 3600 4700 3900
$Comp
L Connector_Generic:Conn_01x06 J?
U 1 1 5D991D53
P 6950 5400
AR Path="/5D991D53" Ref="J?"  Part="1" 
AR Path="/5D96189F/5D991D53" Ref="J7"  Part="1" 
F 0 "J7" H 7030 5392 50  0000 L CNN
F 1 "K3" H 7030 5301 50  0000 L CNN
F 2 "TerminalBlock_Phoenix:TerminalBlock_Phoenix_MPT-0,5-6-2.54_1x06_P2.54mm_Horizontal" H 6950 5400 50  0001 C CNN
F 3 "~" H 6950 5400 50  0001 C CNN
	1    6950 5400
	1    0    0    -1  
$EndComp
Wire Wire Line
	4800 5950 6750 5950
Wire Wire Line
	6750 5950 6750 5700
Wire Wire Line
	6750 5400 6400 5400
Wire Wire Line
	6400 5400 6400 5900
Wire Wire Line
	6400 5900 5200 5900
Wire Wire Line
	5300 5100 6400 5100
Wire Wire Line
	6400 5100 6400 5300
Wire Wire Line
	6400 5300 6750 5300
Wire Wire Line
	6750 5200 6500 5200
Wire Wire Line
	6500 5200 6500 5000
Wire Wire Line
	6500 5000 5100 5000
Wire Wire Line
	4900 4900 6600 4900
Wire Wire Line
	6600 4900 6600 5600
Wire Wire Line
	6600 5600 6750 5600
Wire Wire Line
	6750 5500 6700 5500
Wire Wire Line
	6700 5500 6700 4850
Wire Wire Line
	6700 4850 4700 4850
Wire Wire Line
	5300 5100 5300 5150
Wire Wire Line
	5100 5000 5100 5150
Wire Wire Line
	4900 4900 4900 5150
Wire Wire Line
	4700 4850 4700 5150
Wire Wire Line
	4800 5950 4800 5750
Wire Wire Line
	5200 5900 5200 5750
$Comp
L power:+5VA #PWR?
U 1 1 5D9AE450
P 4400 2150
AR Path="/5D9AE450" Ref="#PWR?"  Part="1" 
AR Path="/5D96189F/5D9AE450" Ref="#PWR022"  Part="1" 
F 0 "#PWR022" H 4400 2000 50  0001 C CNN
F 1 "+5VA" H 4415 2277 50  0000 L CNN
F 2 "" H 4400 2150 50  0001 C CNN
F 3 "" H 4400 2150 50  0001 C CNN
	1    4400 2150
	1    0    0    -1  
$EndComp
Wire Wire Line
	4400 2150 4400 2550
Wire Wire Line
	4400 2150 5500 2150
Wire Wire Line
	5500 2150 5500 2550
Connection ~ 4400 2150
$Comp
L power:+5VA #PWR?
U 1 1 5D9B73EF
P 4400 3550
AR Path="/5D9B73EF" Ref="#PWR?"  Part="1" 
AR Path="/5D96189F/5D9B73EF" Ref="#PWR023"  Part="1" 
F 0 "#PWR023" H 4400 3400 50  0001 C CNN
F 1 "+5VA" H 4415 3677 50  0000 L CNN
F 2 "" H 4400 3550 50  0001 C CNN
F 3 "" H 4400 3550 50  0001 C CNN
	1    4400 3550
	1    0    0    -1  
$EndComp
Wire Wire Line
	4400 3550 4400 3900
Wire Wire Line
	4400 3550 5500 3550
Wire Wire Line
	5500 3550 5500 3900
Connection ~ 4400 3550
$Comp
L power:+5VA #PWR?
U 1 1 5D9BB07F
P 4400 4800
AR Path="/5D9BB07F" Ref="#PWR?"  Part="1" 
AR Path="/5D96189F/5D9BB07F" Ref="#PWR025"  Part="1" 
F 0 "#PWR025" H 4400 4650 50  0001 C CNN
F 1 "+5VA" H 4415 4927 50  0000 L CNN
F 2 "" H 4400 4800 50  0001 C CNN
F 3 "" H 4400 4800 50  0001 C CNN
	1    4400 4800
	1    0    0    -1  
$EndComp
Wire Wire Line
	4400 4800 5500 4800
Wire Wire Line
	5500 4800 5500 5150
Wire Wire Line
	4400 5150 4400 4800
Connection ~ 4400 4800
Wire Wire Line
	5500 3150 5500 3250
Wire Wire Line
	5500 3250 4000 3250
Wire Wire Line
	4000 3250 4000 2200
Wire Wire Line
	4000 2200 3600 2200
Wire Wire Line
	4400 3150 4400 3200
Wire Wire Line
	4400 3200 3950 3200
Wire Wire Line
	3950 3200 3950 2300
Wire Wire Line
	3950 2300 3600 2300
Wire Wire Line
	4400 4500 3850 4500
Wire Wire Line
	3850 4500 3850 2500
Wire Wire Line
	3850 2500 3600 2500
Wire Wire Line
	3600 2400 3900 2400
Wire Wire Line
	3900 2400 3900 4550
Wire Wire Line
	3900 4550 5500 4550
Wire Wire Line
	5500 4550 5500 4500
Wire Wire Line
	4400 5750 3750 5750
Wire Wire Line
	3750 5750 3750 2700
Wire Wire Line
	3750 2700 3600 2700
Wire Wire Line
	3600 2600 3800 2600
Wire Wire Line
	3800 2600 3800 5800
Wire Wire Line
	3800 5800 5500 5800
Wire Wire Line
	5500 5800 5500 5750
$EndSCHEMATC
