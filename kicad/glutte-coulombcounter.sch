EESchema Schematic File Version 4
LIBS:glutte-coulombcounter-cache
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 3
Title "Coulomb meter for Glutte Battery"
Date "2019-09-09"
Rev ""
Comp "HB9EGM"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Connector:Screw_Terminal_01x04 J2
U 1 1 5D38101D
P 1350 2000
F 0 "J2" H 1268 1575 50  0000 C CNN
F 1 "Measure" H 1268 1666 50  0000 C CNN
F 2 "TerminalBlock_Phoenix:TerminalBlock_Phoenix_MPT-0,5-4-2.54_1x04_P2.54mm_Horizontal" H 1350 2000 50  0001 C CNN
F 3 "~" H 1350 2000 50  0001 C CNN
	1    1350 2000
	-1   0    0    1   
$EndComp
Wire Bus Line
	1150 1650 800  1650
Wire Bus Line
	850  1700 850  1900
Wire Bus Line
	750  1900 750  1700
Wire Bus Line
	750  1700 850  1700
Wire Bus Line
	800  1950 1150 1950
Wire Bus Line
	750  1900 850  1900
Wire Bus Line
	900  2300 850  2350
Wire Bus Line
	850  2300 800  2350
Wire Bus Line
	800  2300 750  2350
Wire Bus Line
	750  2300 700  2350
Wire Bus Line
	700  2300 650  2350
Wire Bus Line
	700  2300 900  2300
Wire Bus Line
	700  2150 900  2150
Wire Bus Line
	650  2100 950  2100
Wire Bus Line
	800  1350 750  1400
Wire Bus Line
	850  1400 800  1350
Text Notes 700  1900 1    50   ~ 0
Rshunt
Wire Bus Line
	800  1350 800  1700
Text Notes 1950 1500 0    50   ~ 0
No input filtering because\nload is not inductive\n(AD8210 data sheet p.13)
Wire Wire Line
	3100 2150 3100 2450
Wire Wire Line
	3100 2450 2950 2450
$Comp
L power:GNDA #PWR0101
U 1 1 5D387D27
P 2950 2450
F 0 "#PWR0101" H 2950 2200 50  0001 C CNN
F 1 "GNDA" H 2955 2277 50  0000 C CNN
F 2 "" H 2950 2450 50  0001 C CNN
F 3 "" H 2950 2450 50  0001 C CNN
	1    2950 2450
	1    0    0    -1  
$EndComp
Connection ~ 2950 2450
Wire Wire Line
	2950 2450 2700 2450
$Comp
L power:+5VA #PWR0102
U 1 1 5D38821F
P 2150 2250
F 0 "#PWR0102" H 2150 2100 50  0001 C CNN
F 1 "+5VA" V 2165 2377 50  0000 L CNN
F 2 "" H 2150 2250 50  0001 C CNN
F 3 "" H 2150 2250 50  0001 C CNN
	1    2150 2250
	0    -1   -1   0   
$EndComp
$Comp
L Device:CP1 C6
U 1 1 5D3888B2
P 2150 2400
F 0 "C6" H 2265 2446 50  0000 L CNN
F 1 "0.1u" H 2265 2355 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric_Pad1.42x1.75mm_HandSolder" H 2150 2400 50  0001 C CNN
F 3 "~" H 2150 2400 50  0001 C CNN
	1    2150 2400
	1    0    0    -1  
$EndComp
$Comp
L power:GNDA #PWR0103
U 1 1 5D388F4A
P 2150 2550
F 0 "#PWR0103" H 2150 2300 50  0001 C CNN
F 1 "GNDA" H 2155 2377 50  0000 C CNN
F 2 "" H 2150 2550 50  0001 C CNN
F 3 "" H 2150 2550 50  0001 C CNN
	1    2150 2550
	1    0    0    -1  
$EndComp
Wire Wire Line
	2250 1650 2500 1650
$Comp
L Amplifier_Current:AD8210 U4
U 1 1 5D38683F
P 2700 2050
F 0 "U4" H 2700 1561 50  0000 C CNN
F 1 "AD8210" H 2700 2150 50  0000 C CNN
F 2 "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm" H 2700 2050 50  0001 C CNN
F 3 "https://www.analog.com/media/en/technical-documentation/data-sheets/AD8210.pdf" H 3350 1350 50  0001 C CNN
	1    2700 2050
	1    0    0    -1  
$EndComp
Wire Wire Line
	2250 1800 2250 1650
Wire Wire Line
	2150 1900 2150 1550
Wire Wire Line
	2150 1550 2900 1550
Wire Wire Line
	2900 1550 2900 1650
$Comp
L glutte-coulombcounter:LTC2400 U5
U 1 1 5D3923A7
P 5000 2050
F 0 "U5" H 5250 1650 50  0000 R CNN
F 1 "LTC2400" H 5400 1750 50  0000 R CNN
F 2 "Package_SO:SO-8_3.9x4.9mm_P1.27mm" H 5000 2300 50  0001 C CNN
F 3 "" H 4900 2050 50  0001 C CNN
	1    5000 2050
	1    0    0    -1  
$EndComp
$Comp
L Device:R R2
U 1 1 5D392B33
P 3750 2050
F 0 "R2" V 3543 2050 50  0000 C CNN
F 1 "51" V 3634 2050 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric_Pad1.42x1.75mm_HandSolder" V 3680 2050 50  0001 C CNN
F 3 "~" H 3750 2050 50  0001 C CNN
	1    3750 2050
	0    1    1    0   
$EndComp
$Comp
L power:GNDA #PWR0104
U 1 1 5D39388D
P 5000 2450
F 0 "#PWR0104" H 5000 2200 50  0001 C CNN
F 1 "GNDA" H 5005 2277 50  0000 C CNN
F 2 "" H 5000 2450 50  0001 C CNN
F 3 "" H 5000 2450 50  0001 C CNN
	1    5000 2450
	1    0    0    -1  
$EndComp
$Comp
L Device:CP1 C11
U 1 1 5D395258
P 5250 1300
F 0 "C11" H 5365 1346 50  0000 L CNN
F 1 "4.7u" H 5365 1255 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric_Pad1.42x1.75mm_HandSolder" H 5250 1300 50  0001 C CNN
F 3 "~" H 5250 1300 50  0001 C CNN
	1    5250 1300
	1    0    0    -1  
$EndComp
$Comp
L power:GNDA #PWR0105
U 1 1 5D395262
P 5250 1450
F 0 "#PWR0105" H 5250 1200 50  0001 C CNN
F 1 "GNDA" H 5255 1277 50  0000 C CNN
F 2 "" H 5250 1450 50  0001 C CNN
F 3 "" H 5250 1450 50  0001 C CNN
	1    5250 1450
	1    0    0    -1  
$EndComp
Wire Wire Line
	5000 1650 5000 1150
Wire Wire Line
	5000 1150 5250 1150
Wire Wire Line
	4900 1500 4900 1650
Connection ~ 5000 1150
$Comp
L power:+5VA #PWR0106
U 1 1 5D398C29
P 5000 1150
F 0 "#PWR0106" H 5000 1000 50  0001 C CNN
F 1 "+5VA" H 5015 1323 50  0000 C CNN
F 2 "" H 5000 1150 50  0001 C CNN
F 3 "" H 5000 1150 50  0001 C CNN
	1    5000 1150
	1    0    0    -1  
$EndComp
Wire Wire Line
	5700 1150 5250 1150
Connection ~ 5250 1150
$Comp
L MCU_Microchip_ATmega:ATmega328P-PU U7
U 1 1 5D39FAEE
P 8300 2350
F 0 "U7" H 8800 900 50  0000 R CNN
F 1 "ATmega328P-PU" H 8550 2350 50  0000 R CNN
F 2 "Package_DIP:DIP-28_W7.62mm" H 8300 2350 50  0001 C CIN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/ATmega328_P%20AVR%20MCU%20with%20picoPower%20Technology%20Data%20Sheet%2040001984A.pdf" H 8300 2350 50  0001 C CNN
	1    8300 2350
	-1   0    0    -1  
$EndComp
$Comp
L power:+5VA #PWR0107
U 1 1 5D3A1385
P 8200 700
F 0 "#PWR0107" H 8200 550 50  0001 C CNN
F 1 "+5VA" H 8215 873 50  0000 C CNN
F 2 "" H 8200 700 50  0001 C CNN
F 3 "" H 8200 700 50  0001 C CNN
	1    8200 700 
	1    0    0    -1  
$EndComp
Wire Wire Line
	8200 850  8200 700 
Wire Wire Line
	8300 850  8300 700 
Wire Wire Line
	8300 700  8200 700 
Connection ~ 8200 700 
$Comp
L Device:CP1 C13
U 1 1 5D3A2749
P 8450 700
F 0 "C13" V 8702 700 50  0000 C CNN
F 1 "0.1u" V 8611 700 50  0000 C CNN
F 2 "Capacitor_SMD:C_1206_3216Metric_Pad1.42x1.75mm_HandSolder" H 8450 700 50  0001 C CNN
F 3 "~" H 8450 700 50  0001 C CNN
	1    8450 700 
	0    -1   -1   0   
$EndComp
$Comp
L power:GNDA #PWR0109
U 1 1 5D3A378A
P 8600 700
F 0 "#PWR0109" H 8600 450 50  0001 C CNN
F 1 "GNDA" V 8605 572 50  0000 R CNN
F 2 "" H 8600 700 50  0001 C CNN
F 3 "" H 8600 700 50  0001 C CNN
	1    8600 700 
	0    -1   -1   0   
$EndComp
$Comp
L glutte-coulombcounter:CSTNE16M0V530000R0 Y1
U 1 1 5D3A4871
P 6950 1800
F 0 "Y1" V 7150 1800 50  0000 C CNN
F 1 "CSTNE16M0V530000R0" V 7150 1550 50  0001 C CNN
F 2 "Crystal:Resonator_SMD_muRata_CSTxExxV-3Pin_3.0x1.1mm_HandSoldering" H 6925 1800 50  0001 C CNN
F 3 "~" H 6925 1800 50  0001 C CNN
	1    6950 1800
	0    1    1    0   
$EndComp
$Comp
L Device:R R10
U 1 1 5D3A5A16
P 7250 1800
F 0 "R10" V 7350 1800 50  0000 C CNN
F 1 "1M" V 7450 1800 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric_Pad1.42x1.75mm_HandSolder" V 7180 1800 50  0001 C CNN
F 3 "~" H 7250 1800 50  0001 C CNN
	1    7250 1800
	0    1    1    0   
$EndComp
$Comp
L power:GNDA #PWR0111
U 1 1 5D3A6049
P 6750 1800
F 0 "#PWR0111" H 6750 1550 50  0001 C CNN
F 1 "GNDA" V 6755 1673 50  0000 R CNN
F 2 "" H 6750 1800 50  0001 C CNN
F 3 "" H 6750 1800 50  0001 C CNN
	1    6750 1800
	0    1    1    0   
$EndComp
Wire Wire Line
	7700 1750 7550 1750
Wire Wire Line
	7550 1750 7550 1700
Wire Wire Line
	7550 1700 7100 1700
Wire Wire Line
	7050 1900 7400 1900
Wire Wire Line
	7550 1900 7550 1850
Wire Wire Line
	7550 1850 7700 1850
Wire Wire Line
	7100 1800 7100 1700
Connection ~ 7100 1700
Wire Wire Line
	7100 1700 7050 1700
Wire Wire Line
	7400 1800 7400 1900
Connection ~ 7400 1900
Wire Wire Line
	7400 1900 7550 1900
Wire Wire Line
	8900 1150 9200 1150
$Comp
L Device:C C15
U 1 1 5D3AAD90
P 9200 1350
F 0 "C15" H 9315 1396 50  0000 L CNN
F 1 "0.1u" H 9315 1305 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric_Pad1.42x1.75mm_HandSolder" H 9238 1200 50  0001 C CNN
F 3 "~" H 9200 1350 50  0001 C CNN
	1    9200 1350
	1    0    0    -1  
$EndComp
$Comp
L power:GNDA #PWR0113
U 1 1 5D3AB383
P 9200 1500
F 0 "#PWR0113" H 9200 1250 50  0001 C CNN
F 1 "GNDA" H 9205 1327 50  0000 C CNN
F 2 "" H 9200 1500 50  0001 C CNN
F 3 "" H 9200 1500 50  0001 C CNN
	1    9200 1500
	1    0    0    -1  
$EndComp
$Comp
L Device:R R9
U 1 1 5D3AB834
P 6200 2500
F 0 "R9" H 6270 2454 50  0000 L CNN
F 1 "10k" H 6270 2545 50  0000 L CNN
F 2 "Resistor_SMD:R_1206_3216Metric_Pad1.42x1.75mm_HandSolder" V 6130 2500 50  0001 C CNN
F 3 "~" H 6200 2500 50  0001 C CNN
	1    6200 2500
	-1   0    0    1   
$EndComp
$Comp
L Device:D_ALT D5
U 1 1 5D3AC266
P 6350 2500
F 0 "D5" V 6304 2579 50  0000 L CNN
F 1 "1N4148" V 6395 2579 50  0000 L CNN
F 2 "Diode_SMD:D_1210_3225Metric_Pad1.42x2.65mm_HandSolder" H 6350 2500 50  0001 C CNN
F 3 "~" H 6350 2500 50  0001 C CNN
	1    6350 2500
	0    1    1    0   
$EndComp
Wire Wire Line
	6200 2650 6350 2650
Connection ~ 6350 2650
Wire Wire Line
	6350 2650 7700 2650
$Comp
L power:+5VA #PWR0114
U 1 1 5D3AE3B4
P 6200 2350
F 0 "#PWR0114" H 6200 2200 50  0001 C CNN
F 1 "+5VA" V 6200 2550 50  0000 C CNN
F 2 "" H 6200 2350 50  0001 C CNN
F 3 "" H 6200 2350 50  0001 C CNN
	1    6200 2350
	0    -1   -1   0   
$EndComp
$Comp
L Connector:AVR-ISP-6 J4
U 1 1 5D3AEC0C
P 10500 1200
F 0 "J4" H 10221 1296 50  0000 R CNN
F 1 "AVR-ISP-6" H 10221 1205 50  0000 R CNN
F 2 "Connector_IDC:IDC-Header_2x03_P2.54mm_Vertical" V 10250 1250 50  0001 C CNN
F 3 " ~" H 9225 650 50  0001 C CNN
	1    10500 1200
	1    0    0    -1  
$EndComp
$Comp
L power:GNDA #PWR0115
U 1 1 5D3AF5B2
P 10400 1600
F 0 "#PWR0115" H 10400 1350 50  0001 C CNN
F 1 "GNDA" H 10405 1427 50  0000 C CNN
F 2 "" H 10400 1600 50  0001 C CNN
F 3 "" H 10400 1600 50  0001 C CNN
	1    10400 1600
	1    0    0    -1  
$EndComp
$Comp
L power:+5VA #PWR0116
U 1 1 5D3AF8CA
P 10400 700
F 0 "#PWR0116" H 10400 550 50  0001 C CNN
F 1 "+5VA" H 10415 873 50  0000 C CNN
F 2 "" H 10400 700 50  0001 C CNN
F 3 "" H 10400 700 50  0001 C CNN
	1    10400 700 
	1    0    0    -1  
$EndComp
Text Label 10900 1000 0    50   ~ 0
MISO
Text Label 10900 1100 0    50   ~ 0
MOSI
Text Label 10900 1200 0    50   ~ 0
SCK
Text Label 10900 1300 0    50   ~ 0
RESETn
Text Label 7250 2650 0    50   ~ 0
RESETn
Text Label 7700 1650 2    50   ~ 0
SCK
Text Label 7700 1550 2    50   ~ 0
MISO
Text Label 7700 1450 2    50   ~ 0
MOSI
Wire Wire Line
	3900 2050 4100 2050
$Comp
L Device:C C7
U 1 1 5D3B4AD2
P 3450 2300
F 0 "C7" H 3565 2346 50  0000 L CNN
F 1 "DNF" H 3565 2255 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric_Pad1.42x1.75mm_HandSolder" H 3488 2150 50  0001 C CNN
F 3 "~" H 3450 2300 50  0001 C CNN
	1    3450 2300
	1    0    0    -1  
$EndComp
$Comp
L power:GNDA #PWR0117
U 1 1 5D3B4ADC
P 3450 2450
F 0 "#PWR0117" H 3450 2200 50  0001 C CNN
F 1 "GNDA" H 3455 2277 50  0000 C CNN
F 2 "" H 3450 2450 50  0001 C CNN
F 3 "" H 3450 2450 50  0001 C CNN
	1    3450 2450
	1    0    0    -1  
$EndComp
Wire Wire Line
	3100 2050 3450 2050
Wire Wire Line
	3450 2150 3450 2050
Connection ~ 3450 2050
Wire Wire Line
	3450 2050 3600 2050
$Comp
L Device:C C10
U 1 1 5D3B7CC3
P 4100 2300
F 0 "C10" H 4215 2346 50  0000 L CNN
F 1 "DNF" H 4215 2255 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric_Pad1.42x1.75mm_HandSolder" H 4138 2150 50  0001 C CNN
F 3 "~" H 4100 2300 50  0001 C CNN
	1    4100 2300
	1    0    0    -1  
$EndComp
$Comp
L power:GNDA #PWR0118
U 1 1 5D3B8160
P 4100 2450
F 0 "#PWR0118" H 4100 2200 50  0001 C CNN
F 1 "GNDA" H 4105 2277 50  0000 C CNN
F 2 "" H 4100 2450 50  0001 C CNN
F 3 "" H 4100 2450 50  0001 C CNN
	1    4100 2450
	1    0    0    -1  
$EndComp
Wire Wire Line
	4100 2150 4100 2050
Connection ~ 4100 2050
Wire Wire Line
	4100 2050 4300 2050
$Comp
L Device:C C12
U 1 1 5D3B9BA6
P 5700 1300
F 0 "C12" H 5815 1346 50  0000 L CNN
F 1 "0.1uF" H 5815 1255 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric_Pad1.42x1.75mm_HandSolder" H 5738 1150 50  0001 C CNN
F 3 "~" H 5700 1300 50  0001 C CNN
	1    5700 1300
	1    0    0    -1  
$EndComp
$Comp
L power:GNDA #PWR0119
U 1 1 5D3BA0A0
P 5700 1450
F 0 "#PWR0119" H 5700 1200 50  0001 C CNN
F 1 "GNDA" H 5705 1277 50  0000 C CNN
F 2 "" H 5700 1450 50  0001 C CNN
F 3 "" H 5700 1450 50  0001 C CNN
	1    5700 1450
	1    0    0    -1  
$EndComp
$Comp
L power:+5VA #PWR0120
U 1 1 5D3BA646
P 5700 1900
F 0 "#PWR0120" H 5700 1750 50  0001 C CNN
F 1 "+5VA" H 5715 2073 50  0000 C CNN
F 2 "" H 5700 1900 50  0001 C CNN
F 3 "" H 5700 1900 50  0001 C CNN
	1    5700 1900
	1    0    0    -1  
$EndComp
Wire Wire Line
	5600 1900 5700 1900
Text Notes 3000 1150 0    50   ~ 0
TODO:\n* Filtre entre AD8210 et LTC2400\n* Decider combien de sorties relais et les ajouter\n* Check DB9 DATA connections\n* Decider DB9 GND connection\n* Check all footprints\n* Check LTC2400 bootstrap (serial clock mode)
Text Label 7700 1350 2    50   ~ 0
SSn
Wire Wire Line
	6400 2050 5600 2050
Wire Wire Line
	6300 2150 5600 2150
Wire Wire Line
	6200 2250 5600 2250
Text Label 7700 2850 2    50   ~ 0
RXD
Text Label 7700 2950 2    50   ~ 0
TXD
Text GLabel 4900 1500 1    50   Input ~ 0
VREF
Wire Wire Line
	2300 2250 2150 2250
Text GLabel 2300 2150 0    50   Input ~ 0
VREF
Text GLabel 9200 1100 1    50   Input ~ 0
VREF
Wire Wire Line
	9200 1100 9200 1150
Wire Wire Line
	9200 1200 9200 1150
Connection ~ 9200 1150
Wire Bus Line
	800  2050 1200 2050
Wire Bus Line
	1200 2050 1200 2000
Wire Bus Line
	800  1900 800  2100
$Comp
L power:+BATT #PWR03
U 1 1 5D474ED9
P 1650 2000
F 0 "#PWR03" H 1650 1850 50  0001 C CNN
F 1 "+BATT" V 1665 2128 50  0000 L CNN
F 2 "" H 1650 2000 50  0001 C CNN
F 3 "" H 1650 2000 50  0001 C CNN
	1    1650 2000
	0    1    1    0   
$EndComp
Wire Bus Line
	1250 2100 1250 2250
Wire Bus Line
	1250 2250 800  2250
Wire Bus Line
	800  2150 800  2300
Wire Wire Line
	1550 1800 2250 1800
Wire Wire Line
	1550 1900 2150 1900
Wire Bus Line
	1150 1950 1150 1900
Wire Bus Line
	1150 1800 1150 1650
Wire Bus Line
	1150 1800 1300 1800
Wire Bus Line
	1300 1900 1150 1900
Wire Bus Line
	1300 2000 1200 2000
Wire Bus Line
	1300 2100 1250 2100
$Comp
L power:-BATT #PWR04
U 1 1 5D49B910
P 1650 2100
F 0 "#PWR04" H 1650 1950 50  0001 C CNN
F 1 "-BATT" V 1665 2228 50  0000 L CNN
F 2 "" H 1650 2100 50  0001 C CNN
F 3 "" H 1650 2100 50  0001 C CNN
	1    1650 2100
	0    1    1    0   
$EndComp
Text Label 7700 2050 2    50   ~ 0
ADC0
Text Label 7700 2150 2    50   ~ 0
ADC1
$Comp
L power:+BATT #PWR06
U 1 1 5D4A0255
P 750 4650
F 0 "#PWR06" H 750 4500 50  0001 C CNN
F 1 "+BATT" H 765 4823 50  0000 C CNN
F 2 "" H 750 4650 50  0001 C CNN
F 3 "" H 750 4650 50  0001 C CNN
	1    750  4650
	1    0    0    -1  
$EndComp
$Comp
L Device:R R5
U 1 1 5D4A04D6
P 750 4850
F 0 "R5" H 680 4804 50  0000 R CNN
F 1 "300k/1%" H 680 4895 50  0000 R CNN
F 2 "Resistor_SMD:R_1206_3216Metric_Pad1.42x1.75mm_HandSolder" V 680 4850 50  0001 C CNN
F 3 "~" H 750 4850 50  0001 C CNN
	1    750  4850
	-1   0    0    1   
$EndComp
$Comp
L Device:R R6
U 1 1 5D4A0AFD
P 750 5200
F 0 "R6" H 680 5154 50  0000 R CNN
F 1 "100k/1%" H 680 5245 50  0000 R CNN
F 2 "Resistor_SMD:R_1206_3216Metric_Pad1.42x1.75mm_HandSolder" V 680 5200 50  0001 C CNN
F 3 "~" H 750 5200 50  0001 C CNN
	1    750  5200
	-1   0    0    1   
$EndComp
$Comp
L power:GNDA #PWR07
U 1 1 5D4A107C
P 750 5400
F 0 "#PWR07" H 750 5150 50  0001 C CNN
F 1 "GNDA" H 755 5227 50  0000 C CNN
F 2 "" H 750 5400 50  0001 C CNN
F 3 "" H 750 5400 50  0001 C CNN
	1    750  5400
	1    0    0    -1  
$EndComp
Wire Wire Line
	750  5400 750  5350
Wire Wire Line
	750  4650 750  4700
Wire Wire Line
	750  5000 750  5050
$Comp
L Device:R R7
U 1 1 5D4AD561
P 750 6100
F 0 "R7" H 680 6054 50  0000 R CNN
F 1 "300k/1%" H 680 6145 50  0000 R CNN
F 2 "Resistor_SMD:R_1206_3216Metric_Pad1.42x1.75mm_HandSolder" V 680 6100 50  0001 C CNN
F 3 "~" H 750 6100 50  0001 C CNN
	1    750  6100
	-1   0    0    1   
$EndComp
$Comp
L Device:R R8
U 1 1 5D4AD56B
P 750 6450
F 0 "R8" H 680 6404 50  0000 R CNN
F 1 "100k/1%" H 680 6495 50  0000 R CNN
F 2 "Resistor_SMD:R_1206_3216Metric_Pad1.42x1.75mm_HandSolder" V 680 6450 50  0001 C CNN
F 3 "~" H 750 6450 50  0001 C CNN
	1    750  6450
	-1   0    0    1   
$EndComp
$Comp
L power:GNDA #PWR09
U 1 1 5D4AD575
P 750 6650
F 0 "#PWR09" H 750 6400 50  0001 C CNN
F 1 "GNDA" H 755 6477 50  0000 C CNN
F 2 "" H 750 6650 50  0001 C CNN
F 3 "" H 750 6650 50  0001 C CNN
	1    750  6650
	1    0    0    -1  
$EndComp
Wire Wire Line
	750  6650 750  6600
Wire Wire Line
	750  5900 750  5950
Wire Wire Line
	750  6250 750  6300
$Comp
L power:-BATT #PWR08
U 1 1 5D4B2D35
P 750 5900
F 0 "#PWR08" H 750 5750 50  0001 C CNN
F 1 "-BATT" H 765 6073 50  0000 C CNN
F 2 "" H 750 5900 50  0001 C CNN
F 3 "" H 750 5900 50  0001 C CNN
	1    750  5900
	1    0    0    -1  
$EndComp
$Comp
L Amplifier_Operational:LM324 U6
U 1 1 5D4B3FB4
P 1500 5150
F 0 "U6" H 1500 5517 50  0000 C CNN
F 1 "LM324" H 1500 5426 50  0000 C CNN
F 2 "Package_SO:SOIC-14_3.9x8.7mm_P1.27mm" H 1450 5250 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/lm2902-n.pdf" H 1550 5350 50  0001 C CNN
	1    1500 5150
	1    0    0    -1  
$EndComp
$Comp
L Amplifier_Operational:LM324 U6
U 2 1 5D4B52FF
P 1500 6400
F 0 "U6" H 1500 6767 50  0000 C CNN
F 1 "LM324" H 1500 6676 50  0000 C CNN
F 2 "Package_SO:SOIC-14_3.9x8.7mm_P1.27mm" H 1450 6500 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/lm2902-n.pdf" H 1550 6600 50  0001 C CNN
	2    1500 6400
	1    0    0    -1  
$EndComp
$Comp
L Amplifier_Operational:LM324 U6
U 3 1 5D4B6B38
P 1150 7400
F 0 "U6" H 1150 7767 50  0000 C CNN
F 1 "LM324" H 1150 7676 50  0000 C CNN
F 2 "Package_SO:SOIC-14_3.9x8.7mm_P1.27mm" H 1100 7500 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/lm2902-n.pdf" H 1200 7600 50  0001 C CNN
	3    1150 7400
	1    0    0    -1  
$EndComp
$Comp
L Amplifier_Operational:LM324 U6
U 4 1 5D4B7D78
P 2100 7400
F 0 "U6" H 2100 7767 50  0000 C CNN
F 1 "LM324" H 2100 7676 50  0000 C CNN
F 2 "Package_SO:SOIC-14_3.9x8.7mm_P1.27mm" H 2050 7500 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/lm2902-n.pdf" H 2150 7600 50  0001 C CNN
	4    2100 7400
	1    0    0    -1  
$EndComp
$Comp
L Amplifier_Operational:LM324 U6
U 5 1 5D4B87AB
P 2150 5750
F 0 "U6" H 2108 5796 50  0000 L CNN
F 1 "LM324" H 2108 5705 50  0000 L CNN
F 2 "Package_SO:SOIC-14_3.9x8.7mm_P1.27mm" H 2100 5850 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/lm2902-n.pdf" H 2200 5950 50  0001 C CNN
	5    2150 5750
	1    0    0    -1  
$EndComp
Wire Wire Line
	1200 5250 1200 5400
Wire Wire Line
	1200 5400 1800 5400
Wire Wire Line
	1800 5400 1800 5150
Wire Wire Line
	1200 6500 1200 6650
Wire Wire Line
	1200 6650 1800 6650
Wire Wire Line
	1800 6650 1800 6400
Wire Wire Line
	850  7500 850  7650
Wire Wire Line
	850  7650 1450 7650
Wire Wire Line
	1450 7650 1450 7400
Wire Wire Line
	1800 7500 1800 7650
Wire Wire Line
	1800 7650 2400 7650
Wire Wire Line
	2400 7650 2400 7400
$Comp
L power:GNDA #PWR010
U 1 1 5D4E0EB3
P 850 7300
F 0 "#PWR010" H 850 7050 50  0001 C CNN
F 1 "GNDA" V 855 7173 50  0000 R CNN
F 2 "" H 850 7300 50  0001 C CNN
F 3 "" H 850 7300 50  0001 C CNN
	1    850  7300
	0    1    1    0   
$EndComp
$Comp
L power:GNDA #PWR011
U 1 1 5D4E14F6
P 1800 7300
F 0 "#PWR011" H 1800 7050 50  0001 C CNN
F 1 "GNDA" V 1805 7173 50  0000 R CNN
F 2 "" H 1800 7300 50  0001 C CNN
F 3 "" H 1800 7300 50  0001 C CNN
	1    1800 7300
	0    1    1    0   
$EndComp
$Comp
L power:GNDA #PWR013
U 1 1 5D4E1769
P 2050 6050
F 0 "#PWR013" H 2050 5800 50  0001 C CNN
F 1 "GNDA" H 2055 5877 50  0000 C CNN
F 2 "" H 2050 6050 50  0001 C CNN
F 3 "" H 2050 6050 50  0001 C CNN
	1    2050 6050
	1    0    0    -1  
$EndComp
$Comp
L power:+5VA #PWR012
U 1 1 5D4E19B3
P 2050 5450
F 0 "#PWR012" H 2050 5300 50  0001 C CNN
F 1 "+5VA" H 2065 5623 50  0000 C CNN
F 2 "" H 2050 5450 50  0001 C CNN
F 3 "" H 2050 5450 50  0001 C CNN
	1    2050 5450
	1    0    0    -1  
$EndComp
Text Label 1800 5150 0    50   ~ 0
ADC0
Text Label 1800 6400 0    50   ~ 0
ADC1
Wire Wire Line
	750  5050 1200 5050
Connection ~ 750  5050
Wire Wire Line
	750  6300 1200 6300
Connection ~ 750  6300
Text Notes 650  4200 0    100  ~ 0
V BAT MEASURE
Text Notes 650  4400 0    50   ~ 0
Diviseur tension facteur=4\nLM324 limite max 5V protection ATMega
Wire Notes Line
	2400 4050 2400 6900
Wire Notes Line
	2400 6900 550  6900
Wire Notes Line
	550  6900 550  4050
Wire Notes Line
	550  4050 2400 4050
$Comp
L Interface_UART:MAX232 U8
U 1 1 5D549680
P 9050 5150
F 0 "U8" H 8500 4050 50  0000 C CNN
F 1 "MAX232" H 9050 5250 50  0000 C CNN
F 2 "Package_SO:SOIC-16_3.9x9.9mm_P1.27mm" H 9100 4100 50  0001 L CNN
F 3 "http://www.ti.com/lit/ds/symlink/max232.pdf" H 9050 5250 50  0001 C CNN
	1    9050 5150
	1    0    0    -1  
$EndComp
Wire Wire Line
	6400 1650 7700 1650
Wire Wire Line
	6400 1650 6400 2050
Wire Wire Line
	7700 1550 6300 1550
Wire Wire Line
	6300 1550 6300 2150
Wire Wire Line
	7700 1350 6200 1350
Wire Wire Line
	6200 1350 6200 2250
$Comp
L Device:CP1 C14
U 1 1 5D5A3729
P 8250 4400
F 0 "C14" H 8136 4446 50  0000 R CNN
F 1 "1u" H 8136 4355 50  0000 R CNN
F 2 "Capacitor_SMD:C_1206_3216Metric_Pad1.42x1.75mm_HandSolder" H 8250 4400 50  0001 C CNN
F 3 "~" H 8250 4400 50  0001 C CNN
	1    8250 4400
	1    0    0    -1  
$EndComp
$Comp
L Device:CP1 C17
U 1 1 5D5A42B3
P 9850 4400
F 0 "C17" H 9965 4446 50  0000 L CNN
F 1 "1u" H 9965 4355 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric_Pad1.42x1.75mm_HandSolder" H 9850 4400 50  0001 C CNN
F 3 "~" H 9850 4400 50  0001 C CNN
	1    9850 4400
	1    0    0    -1  
$EndComp
$Comp
L Device:CP1 C16
U 1 1 5D5A596C
P 9200 3950
F 0 "C16" V 9452 3950 50  0000 C CNN
F 1 "1u" V 9361 3950 50  0000 C CNN
F 2 "Capacitor_SMD:C_1206_3216Metric_Pad1.42x1.75mm_HandSolder" H 9200 3950 50  0001 C CNN
F 3 "~" H 9200 3950 50  0001 C CNN
	1    9200 3950
	0    -1   -1   0   
$EndComp
$Comp
L power:GNDA #PWR019
U 1 1 5D5A6A8B
P 9350 3950
F 0 "#PWR019" H 9350 3700 50  0001 C CNN
F 1 "GNDA" V 9355 3822 50  0000 R CNN
F 2 "" H 9350 3950 50  0001 C CNN
F 3 "" H 9350 3950 50  0001 C CNN
	1    9350 3950
	0    -1   -1   0   
$EndComp
$Comp
L power:+5VA #PWR017
U 1 1 5D5A6D91
P 9050 3950
F 0 "#PWR017" H 9050 3800 50  0001 C CNN
F 1 "+5VA" V 9065 4077 50  0000 L CNN
F 2 "" H 9050 3950 50  0001 C CNN
F 3 "" H 9050 3950 50  0001 C CNN
	1    9050 3950
	0    -1   -1   0   
$EndComp
Connection ~ 9050 3950
$Comp
L power:GNDA #PWR018
U 1 1 5D5A7987
P 9050 6350
F 0 "#PWR018" H 9050 6100 50  0001 C CNN
F 1 "GNDA" H 9055 6177 50  0000 C CNN
F 2 "" H 9050 6350 50  0001 C CNN
F 3 "" H 9050 6350 50  0001 C CNN
	1    9050 6350
	1    0    0    -1  
$EndComp
Text Label 8250 5650 2    50   ~ 0
RXD
Text Label 8250 5250 2    50   ~ 0
TXD
$Comp
L Device:CP1 C18
U 1 1 5D5C51E2
P 10000 4750
F 0 "C18" V 10050 4900 50  0000 C CNN
F 1 "1u" V 10161 4750 50  0000 C CNN
F 2 "Capacitor_SMD:C_1206_3216Metric_Pad1.42x1.75mm_HandSolder" H 10000 4750 50  0001 C CNN
F 3 "~" H 10000 4750 50  0001 C CNN
	1    10000 4750
	0    -1   -1   0   
$EndComp
$Comp
L Device:CP1 C19
U 1 1 5D5C5B10
P 10000 5050
F 0 "C19" V 10160 5050 50  0000 C CNN
F 1 "1u" V 10251 5050 50  0000 C CNN
F 2 "Capacitor_SMD:C_1206_3216Metric_Pad1.42x1.75mm_HandSolder" H 10000 5050 50  0001 C CNN
F 3 "~" H 10000 5050 50  0001 C CNN
	1    10000 5050
	0    1    1    0   
$EndComp
$Comp
L power:GNDA #PWR020
U 1 1 5D5C60F6
P 10150 4900
F 0 "#PWR020" H 10150 4650 50  0001 C CNN
F 1 "GNDA" V 10155 4772 50  0000 R CNN
F 2 "" H 10150 4900 50  0001 C CNN
F 3 "" H 10150 4900 50  0001 C CNN
	1    10150 4900
	0    -1   -1   0   
$EndComp
Wire Wire Line
	10150 4750 10150 4900
Connection ~ 10150 4900
Wire Wire Line
	10150 4900 10150 5050
$Comp
L Connector:DB9_Female J5
U 1 1 5D5CBD57
P 10900 5650
F 0 "J5" H 10900 5150 50  0000 L CNN
F 1 "DB9_Female" H 10700 5000 50  0000 L CNN
F 2 "Connector_Dsub:DSUB-9_Female_EdgeMount_P2.77mm" H 10900 5650 50  0001 C CNN
F 3 " ~" H 10900 5650 50  0001 C CNN
	1    10900 5650
	1    0    0    -1  
$EndComp
Wire Wire Line
	9900 5250 9850 5250
Wire Wire Line
	10600 5650 9850 5650
Wire Wire Line
	9900 5250 9900 5350
Wire Wire Line
	9900 5350 10450 5350
Wire Wire Line
	10450 5350 10450 5450
Wire Wire Line
	10450 5450 10600 5450
Wire Wire Line
	10600 5850 10350 5850
Wire Wire Line
	10350 5850 10350 5450
Wire Wire Line
	10350 5450 9850 5450
Text Label 8250 5450 2    50   ~ 0
DTR
Text Label 7700 3050 2    50   ~ 0
DTR
$Comp
L power:GNDA #PWR021
U 1 1 5D5EDAB4
P 10300 6050
F 0 "#PWR021" H 10300 5800 50  0001 C CNN
F 1 "GNDA" V 10305 5923 50  0000 R CNN
F 2 "" H 10300 6050 50  0001 C CNN
F 3 "" H 10300 6050 50  0001 C CNN
	1    10300 6050
	0    1    1    0   
$EndComp
$Comp
L Device:Ferrite_Bead FB1
U 1 1 5D5EE95A
P 10450 6050
F 0 "FB1" V 10268 6050 50  0000 C CNN
F 1 "DNF" V 10177 6050 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric_Pad1.42x1.75mm_HandSolder" V 10380 6050 50  0001 C CNN
F 3 "~" H 10450 6050 50  0001 C CNN
	1    10450 6050
	0    -1   -1   0   
$EndComp
Wire Wire Line
	10600 5350 10500 5350
Wire Wire Line
	10500 5350 10500 5750
Wire Wire Line
	10500 5750 9850 5750
Wire Wire Line
	9850 5750 9850 5850
Text Label 8250 5850 2    50   ~ 0
DSR
Text Label 7700 3150 2    50   ~ 0
DSR
NoConn ~ 10600 5250
NoConn ~ 10600 5550
NoConn ~ 10600 5750
NoConn ~ 10600 5950
Text Notes 9950 4100 0    50   ~ 0
TTL to RS232 level conversion\n
$Comp
L Sensor_Temperature:DS18B20 U1
U 1 1 5D629894
P 1250 3400
F 0 "U1" H 1020 3446 50  0000 R CNN
F 1 "DS18B20" H 1020 3355 50  0000 R CNN
F 2 "Package_TO_SOT_THT:TO-92_Inline" H 250 3150 50  0001 C CNN
F 3 "http://datasheets.maximintegrated.com/en/ds/DS18B20.pdf" H 1100 3650 50  0001 C CNN
	1    1250 3400
	1    0    0    -1  
$EndComp
$Comp
L power:GNDA #PWR02
U 1 1 5D62AC3D
P 1250 3700
F 0 "#PWR02" H 1250 3450 50  0001 C CNN
F 1 "GNDA" H 1255 3527 50  0000 C CNN
F 2 "" H 1250 3700 50  0001 C CNN
F 3 "" H 1250 3700 50  0001 C CNN
	1    1250 3700
	1    0    0    -1  
$EndComp
$Comp
L power:+5VA #PWR01
U 1 1 5D62AEDD
P 1250 3100
F 0 "#PWR01" H 1250 2950 50  0001 C CNN
F 1 "+5VA" H 1265 3273 50  0000 C CNN
F 2 "" H 1250 3100 50  0001 C CNN
F 3 "" H 1250 3100 50  0001 C CNN
	1    1250 3100
	1    0    0    -1  
$EndComp
Text Label 1550 3400 0    50   ~ 0
1WIRE
$Comp
L Device:R R11
U 1 1 5D62BF0C
P 7400 3100
F 0 "R11" H 7470 3054 50  0000 L CNN
F 1 "4k7" H 7470 3145 50  0000 L CNN
F 2 "Resistor_SMD:R_1206_3216Metric_Pad1.42x1.75mm_HandSolder" V 7330 3100 50  0001 C CNN
F 3 "~" H 7400 3100 50  0001 C CNN
	1    7400 3100
	-1   0    0    1   
$EndComp
$Comp
L power:+5VA #PWR014
U 1 1 5D62C601
P 7400 2950
F 0 "#PWR014" H 7400 2800 50  0001 C CNN
F 1 "+5VA" H 7415 3123 50  0000 C CNN
F 2 "" H 7400 2950 50  0001 C CNN
F 3 "" H 7400 2950 50  0001 C CNN
	1    7400 2950
	1    0    0    -1  
$EndComp
Text Label 7700 3250 2    50   ~ 0
1WIRE
Wire Wire Line
	7400 3250 7700 3250
NoConn ~ 7700 1250
NoConn ~ 7700 3550
$Comp
L Device:LED_ALT D7
U 1 1 5D6DA7D7
P 7050 950
F 0 "D7" V 7089 1029 50  0000 L CNN
F 1 "LED_STATUS" V 6998 1029 50  0000 L CNN
F 2 "LED_SMD:LED_1210_3225Metric_Pad1.42x2.65mm_HandSolder" H 7050 950 50  0001 C CNN
F 3 "~" H 7050 950 50  0001 C CNN
	1    7050 950 
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R13
U 1 1 5D6DA7E1
P 7250 1150
F 0 "R13" V 7457 1150 50  0000 C CNN
F 1 "330" V 7366 1150 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric_Pad1.42x1.75mm_HandSolder" V 7180 1150 50  0001 C CNN
F 3 "~" H 7250 1150 50  0001 C CNN
	1    7250 1150
	0    -1   -1   0   
$EndComp
Text Label 7700 1150 2    50   ~ 0
STATUS
$Comp
L power:+5VA #PWR024
U 1 1 5D6E217A
P 7050 800
F 0 "#PWR024" H 7050 650 50  0001 C CNN
F 1 "+5VA" H 7065 973 50  0000 C CNN
F 2 "" H 7050 800 50  0001 C CNN
F 3 "" H 7050 800 50  0001 C CNN
	1    7050 800 
	1    0    0    -1  
$EndComp
Connection ~ 8300 700 
Wire Wire Line
	7050 1100 7050 1150
Wire Wire Line
	7050 1150 7100 1150
Wire Wire Line
	7400 1150 7700 1150
$Comp
L Mechanical:MountingHole H2
U 1 1 5D74A21D
P 600 800
F 0 "H2" H 700 846 50  0000 L CNN
F 1 "M4" H 700 755 50  0000 L CNN
F 2 "MountingHole:MountingHole_4.3mm_M4" H 600 800 50  0001 C CNN
F 3 "~" H 600 800 50  0001 C CNN
	1    600  800 
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H1
U 1 1 5D748F1E
P 600 600
F 0 "H1" H 700 646 50  0000 L CNN
F 1 "M4" H 700 555 50  0000 L CNN
F 2 "MountingHole:MountingHole_4.3mm_M4" H 600 600 50  0001 C CNN
F 3 "~" H 600 600 50  0001 C CNN
	1    600  600 
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H3
U 1 1 5D752FEA
P 600 1000
F 0 "H3" H 700 1046 50  0000 L CNN
F 1 "M4" H 700 955 50  0000 L CNN
F 2 "MountingHole:MountingHole_4.3mm_M4" H 600 1000 50  0001 C CNN
F 3 "~" H 600 1000 50  0001 C CNN
	1    600  1000
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H4
U 1 1 5D753140
P 600 1200
F 0 "H4" H 700 1246 50  0000 L CNN
F 1 "M4" H 700 1155 50  0000 L CNN
F 2 "MountingHole:MountingHole_4.3mm_M4" H 600 1200 50  0001 C CNN
F 3 "~" H 600 1200 50  0001 C CNN
	1    600  1200
	1    0    0    -1  
$EndComp
Text Label 1600 1800 0    50   ~ 0
SHUNT+
Text Label 1600 1900 0    50   ~ 0
SHUNT-
Text Notes 1750 3250 0    100  ~ 0
MESURE TEMPERATURE
Text Notes 1850 3350 0    50   ~ 0
Dans le bac des batteries\n
Wire Notes Line
	550  3950 3500 3950
Wire Notes Line
	3500 3950 3500 2850
Wire Notes Line
	3500 2850 550  2850
Wire Notes Line
	550  2850 550  3950
Wire Bus Line
	600  2000 600  2100
Wire Bus Line
	550  2050 650  2050
Text Notes 10500 800  0    50   ~ 0
Programmation\nATMega
$Sheet
S 1550 600  1200 550 
U 5D85D9F3
F0 "power" 50
F1 "power.sch" 50
F2 "IN_+12V" I L 1550 850 50 
F3 "IN_GND" I L 1550 950 50 
$EndSheet
Text Notes 1700 750  0    50   ~ 0
Provides VREF and +5VA
$Comp
L power:PWR_FLAG #FLG0101
U 1 1 5D8C58BF
P 1350 850
F 0 "#FLG0101" H 1350 925 50  0001 C CNN
F 1 "PWR_FLAG" H 1350 1023 50  0000 C CNN
F 2 "" H 1350 850 50  0001 C CNN
F 3 "~" H 1350 850 50  0001 C CNN
	1    1350 850 
	1    0    0    -1  
$EndComp
$Comp
L power:PWR_FLAG #FLG0102
U 1 1 5D8C5A5E
P 1350 950
F 0 "#FLG0102" H 1350 1025 50  0001 C CNN
F 1 "PWR_FLAG" H 1350 1123 50  0000 C CNN
F 2 "" H 1350 950 50  0001 C CNN
F 3 "~" H 1350 950 50  0001 C CNN
	1    1350 950 
	-1   0    0    1   
$EndComp
Wire Wire Line
	1200 850  1350 850 
Connection ~ 1350 850 
Wire Wire Line
	1350 850  1550 850 
Wire Wire Line
	1200 950  1350 950 
Connection ~ 1350 950 
Wire Wire Line
	1350 950  1550 950 
$Comp
L power:PWR_FLAG #FLG0103
U 1 1 5D8CC20A
P 1550 2000
F 0 "#FLG0103" H 1550 2075 50  0001 C CNN
F 1 "PWR_FLAG" H 1550 2173 50  0001 C CNN
F 2 "" H 1550 2000 50  0001 C CNN
F 3 "~" H 1550 2000 50  0001 C CNN
	1    1550 2000
	-1   0    0    1   
$EndComp
$Comp
L power:PWR_FLAG #FLG0104
U 1 1 5D8D7858
P 1600 2100
F 0 "#FLG0104" H 1600 2175 50  0001 C CNN
F 1 "PWR_FLAG" H 1600 2273 50  0001 C CNN
F 2 "" H 1600 2100 50  0001 C CNN
F 3 "~" H 1600 2100 50  0001 C CNN
	1    1600 2100
	-1   0    0    1   
$EndComp
Connection ~ 2150 2250
Wire Wire Line
	1650 2100 1600 2100
Wire Wire Line
	1550 2000 1650 2000
Connection ~ 1550 2000
Connection ~ 1600 2100
Wire Wire Line
	1600 2100 1550 2100
$Comp
L Connector:Screw_Terminal_01x02 J1
U 1 1 5D89BD64
P 1000 850
F 0 "J1" H 918 1067 50  0000 C CNN
F 1 "12V" H 918 976 50  0000 C CNN
F 2 "TerminalBlock_Phoenix:TerminalBlock_Phoenix_MPT-0,5-2-2.54_1x02_P2.54mm_Horizontal" H 1000 850 50  0001 C CNN
F 3 "~" H 1000 850 50  0001 C CNN
	1    1000 850 
	-1   0    0    -1  
$EndComp
$Comp
L power:GNDA #PWR0110
U 1 1 5D3A3B91
P 8300 3850
F 0 "#PWR0110" H 8300 3600 50  0001 C CNN
F 1 "GNDA" H 8305 3677 50  0000 C CNN
F 2 "" H 8300 3850 50  0001 C CNN
F 3 "" H 8300 3850 50  0001 C CNN
	1    8300 3850
	1    0    0    -1  
$EndComp
$Sheet
S 5900 2850 800  1000
U 5D96189F
F0 "Relays" 50
F1 "relays.sch" 50
F2 "K1_SET" I R 6700 3650 50 
F3 "K1_RESET" I R 6700 3550 50 
F4 "K2_RESET" I R 6700 3300 50 
F5 "K2_SET" I R 6700 3400 50 
F6 "K3_RESET" I R 6700 3050 50 
F7 "K3_SET" I R 6700 3150 50 
$EndSheet
Wire Wire Line
	7700 2450 7100 2450
Wire Wire Line
	7150 2550 7700 2550
Wire Wire Line
	6350 2350 6200 2350
Connection ~ 6200 2350
Wire Wire Line
	7700 2250 6800 2250
Wire Wire Line
	6800 2250 6800 3050
Wire Wire Line
	6800 3050 6700 3050
Wire Wire Line
	6700 3150 6850 3150
Wire Wire Line
	6850 3150 6850 2350
Wire Wire Line
	6850 2350 7700 2350
Wire Wire Line
	7100 3300 6700 3300
Wire Wire Line
	7100 2450 7100 3300
Wire Wire Line
	7150 3400 6700 3400
Wire Wire Line
	7150 2550 7150 3400
Wire Wire Line
	7700 3350 7200 3350
Wire Wire Line
	7200 3350 7200 3550
Wire Wire Line
	7200 3550 6700 3550
Wire Wire Line
	7700 3450 7250 3450
Wire Wire Line
	7250 3450 7250 3650
Wire Wire Line
	7250 3650 6700 3650
$EndSCHEMATC
