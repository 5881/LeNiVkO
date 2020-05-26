EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
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
$Comp
L RF:NRF24L01_Breakout U3
U 1 1 5EC9526B
P 4050 3125
F 0 "U3" H 4430 3171 50  0000 L CNN
F 1 "NRF24L01_Breakout" H 4430 3080 50  0000 L CNN
F 2 "RF_Module:nRF24L01_Breakout" H 4200 3725 50  0001 L CIN
F 3 "http://www.nordicsemi.com/eng/content/download/2730/34105/file/nRF24L01_Product_Specification_v2_0.pdf" H 4050 3025 50  0001 C CNN
	1    4050 3125
	1    0    0    -1  
$EndComp
$Comp
L Interface_Expansion:PCF8574 U2
U 1 1 5EC959F2
P 2300 5625
F 0 "U2" V 2075 6300 50  0000 C CNN
F 1 "PCF8574" V 2275 5725 50  0000 C CNN
F 2 "" H 2300 5625 50  0001 C CNN
F 3 "http://www.nxp.com/documents/data_sheet/PCF8574_PCF8574A.pdf" H 2300 5625 50  0001 C CNN
	1    2300 5625
	0    -1   1    0   
$EndComp
$Comp
L MCU_ST_STM32F0:STM32F030C8Tx U5
U 1 1 5EC99AC5
P 7325 3150
F 0 "U5" H 6800 1475 50  0000 C CNN
F 1 "STM32F030C8Tx" H 6800 1575 50  0000 C CNN
F 2 "Package_QFP:LQFP-48_7x7mm_P0.5mm" H 6825 1650 50  0001 R CNN
F 3 "http://www.st.com/st-web-ui/static/active/en/resource/technical/document/datasheet/DM00088500.pdf" H 7325 3150 50  0001 C CNN
	1    7325 3150
	1    0    0    -1  
$EndComp
$Comp
L Connector:USB_A J1
U 1 1 5ECA6579
P 9475 4250
F 0 "J1" H 9245 4239 50  0000 R CNN
F 1 "USB_A" H 9245 4148 50  0000 R CNN
F 2 "" H 9625 4200 50  0001 C CNN
F 3 " ~" H 9625 4200 50  0001 C CNN
	1    9475 4250
	-1   0    0    -1  
$EndComp
$Comp
L RF:NRF24L01_Breakout U6
U 1 1 5ECB0D47
P 9225 1750
F 0 "U6" H 9605 1796 50  0000 L CNN
F 1 "NRF24L01_Breakout" H 9605 1705 50  0000 L CNN
F 2 "RF_Module:nRF24L01_Breakout" H 9375 2350 50  0001 L CIN
F 3 "http://www.nordicsemi.com/eng/content/download/2730/34105/file/nRF24L01_Product_Specification_v2_0.pdf" H 9225 1650 50  0001 C CNN
	1    9225 1750
	1    0    0    -1  
$EndComp
Wire Wire Line
	9175 4250 8750 4250
Wire Wire Line
	7925 4150 8375 4150
Wire Wire Line
	8375 4150 8375 4350
Wire Wire Line
	8375 4350 9175 4350
$Comp
L power:+5V #PWR0101
U 1 1 5ECBBFA8
P 9175 4050
F 0 "#PWR0101" H 9175 3900 50  0001 C CNN
F 1 "+5V" H 9190 4223 50  0000 C CNN
F 2 "" H 9175 4050 50  0001 C CNN
F 3 "" H 9175 4050 50  0001 C CNN
	1    9175 4050
	1    0    0    -1  
$EndComp
$Comp
L Device:R R6
U 1 1 5ECBF4D9
P 8750 4100
F 0 "R6" H 8820 4146 50  0000 L CNN
F 1 "1K5" H 8820 4055 50  0000 L CNN
F 2 "" V 8680 4100 50  0001 C CNN
F 3 "~" H 8750 4100 50  0001 C CNN
	1    8750 4100
	1    0    0    -1  
$EndComp
Connection ~ 8750 4250
Wire Wire Line
	8750 4250 7925 4250
$Comp
L power:+3.3V #PWR0102
U 1 1 5ECC145E
P 8750 3950
F 0 "#PWR0102" H 8750 3800 50  0001 C CNN
F 1 "+3.3V" H 8765 4123 50  0000 C CNN
F 2 "" H 8750 3950 50  0001 C CNN
F 3 "" H 8750 3950 50  0001 C CNN
	1    8750 3950
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0103
U 1 1 5ECC2B04
P 10125 3200
F 0 "#PWR0103" H 10125 2950 50  0001 C CNN
F 1 "GND" H 10130 3027 50  0000 C CNN
F 2 "" H 10125 3200 50  0001 C CNN
F 3 "" H 10125 3200 50  0001 C CNN
	1    10125 3200
	1    0    0    -1  
$EndComp
Wire Wire Line
	9825 2900 9825 2800
Wire Wire Line
	9825 2800 9500 2800
$Comp
L power:+3.3V #PWR0104
U 1 1 5ECC41C0
P 9500 2800
F 0 "#PWR0104" H 9500 2650 50  0001 C CNN
F 1 "+3.3V" H 9515 2973 50  0000 C CNN
F 2 "" H 9500 2800 50  0001 C CNN
F 3 "" H 9500 2800 50  0001 C CNN
	1    9500 2800
	1    0    0    -1  
$EndComp
$Comp
L newlib:C C17
U 1 1 5ECC46FB
P 9500 2950
F 0 "C17" H 9582 3003 60  0000 L CNN
F 1 "10uF" H 9582 2897 60  0000 L CNN
F 2 "" V 9500 2950 60  0000 C CNN
F 3 "" V 9500 2950 60  0000 C CNN
	1    9500 2950
	1    0    0    -1  
$EndComp
Connection ~ 9500 2800
$Comp
L power:GND #PWR0105
U 1 1 5ECC57CF
P 9500 3100
F 0 "#PWR0105" H 9500 2850 50  0001 C CNN
F 1 "GND" H 9505 2927 50  0000 C CNN
F 2 "" H 9500 3100 50  0001 C CNN
F 3 "" H 9500 3100 50  0001 C CNN
	1    9500 3100
	1    0    0    -1  
$EndComp
Connection ~ 9825 2800
$Comp
L Regulator_Linear:AP131-33 U7
U 1 1 5ECBD2F6
P 10125 2900
F 0 "U7" H 10125 3242 50  0000 C CNN
F 1 "RT9193-33" H 10125 3151 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23-5" H 10125 3225 50  0001 C CNN
F 3 "http://www.diodes.com/_files/datasheets/AP131.pdf" H 10125 2900 50  0001 C CNN
	1    10125 2900
	-1   0    0    -1  
$EndComp
Wire Wire Line
	10425 2900 10425 2800
$Comp
L power:+5V #PWR0106
U 1 1 5ECC6F81
P 10425 2800
F 0 "#PWR0106" H 10425 2650 50  0001 C CNN
F 1 "+5V" H 10440 2973 50  0000 C CNN
F 2 "" H 10425 2800 50  0001 C CNN
F 3 "" H 10425 2800 50  0001 C CNN
	1    10425 2800
	1    0    0    -1  
$EndComp
Connection ~ 10425 2800
Wire Wire Line
	9225 1150 9850 1150
$Comp
L newlib:C C18
U 1 1 5ECC9E81
P 9850 1300
F 0 "C18" H 9932 1353 60  0000 L CNN
F 1 "10uF" H 9932 1247 60  0000 L CNN
F 2 "" V 9850 1300 60  0000 C CNN
F 3 "" V 9850 1300 60  0000 C CNN
	1    9850 1300
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0107
U 1 1 5ECC9E87
P 9850 1450
F 0 "#PWR0107" H 9850 1200 50  0001 C CNN
F 1 "GND" H 9855 1277 50  0000 C CNN
F 2 "" H 9850 1450 50  0001 C CNN
F 3 "" H 9850 1450 50  0001 C CNN
	1    9850 1450
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0108
U 1 1 5ECCAF0D
P 9850 1150
F 0 "#PWR0108" H 9850 1000 50  0001 C CNN
F 1 "+3.3V" H 9865 1323 50  0000 C CNN
F 2 "" H 9850 1150 50  0001 C CNN
F 3 "" H 9850 1150 50  0001 C CNN
	1    9850 1150
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0109
U 1 1 5ECCDB14
P 9225 2350
F 0 "#PWR0109" H 9225 2100 50  0001 C CNN
F 1 "GND" H 9230 2177 50  0000 C CNN
F 2 "" H 9225 2350 50  0001 C CNN
F 3 "" H 9225 2350 50  0001 C CNN
	1    9225 2350
	1    0    0    -1  
$EndComp
$Comp
L newlib:C C19
U 1 1 5ECD0125
P 10275 1300
F 0 "C19" H 10357 1353 60  0000 L CNN
F 1 "10uF" H 10357 1247 60  0000 L CNN
F 2 "" V 10275 1300 60  0000 C CNN
F 3 "" V 10275 1300 60  0000 C CNN
	1    10275 1300
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0110
U 1 1 5ECD012B
P 10275 1450
F 0 "#PWR0110" H 10275 1200 50  0001 C CNN
F 1 "GND" H 10280 1277 50  0000 C CNN
F 2 "" H 10275 1450 50  0001 C CNN
F 3 "" H 10275 1450 50  0001 C CNN
	1    10275 1450
	1    0    0    -1  
$EndComp
Wire Wire Line
	9850 1150 10275 1150
Connection ~ 9850 1150
Wire Wire Line
	7225 1550 7325 1550
Connection ~ 7325 1550
Wire Wire Line
	7325 1550 7425 1550
Connection ~ 7425 1550
Wire Wire Line
	7425 1550 7525 1550
Connection ~ 7525 1550
Wire Wire Line
	7525 1550 8050 1550
$Comp
L newlib:C C16
U 1 1 5ECD2BD9
P 8050 1700
F 0 "C16" H 7900 1575 60  0000 L CNN
F 1 "100n" H 7925 2000 60  0000 L CNN
F 2 "" V 8050 1700 60  0000 C CNN
F 3 "" V 8050 1700 60  0000 C CNN
	1    8050 1700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0111
U 1 1 5ECD2BDF
P 8050 1850
F 0 "#PWR0111" H 8050 1600 50  0001 C CNN
F 1 "GND" H 8055 1677 50  0000 C CNN
F 2 "" H 8050 1850 50  0001 C CNN
F 3 "" H 8050 1850 50  0001 C CNN
	1    8050 1850
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0112
U 1 1 5ECD3A39
P 7425 1550
F 0 "#PWR0112" H 7425 1400 50  0001 C CNN
F 1 "+3.3V" H 7440 1723 50  0000 C CNN
F 2 "" H 7425 1550 50  0001 C CNN
F 3 "" H 7425 1550 50  0001 C CNN
	1    7425 1550
	1    0    0    -1  
$EndComp
Wire Wire Line
	7225 4750 7325 4750
Connection ~ 7325 4750
Wire Wire Line
	7325 4750 7425 4750
Wire Wire Line
	9475 4650 9575 4650
$Comp
L power:GND #PWR0113
U 1 1 5ECD6173
P 9475 4650
F 0 "#PWR0113" H 9475 4400 50  0001 C CNN
F 1 "GND" H 9480 4477 50  0000 C CNN
F 2 "" H 9475 4650 50  0001 C CNN
F 3 "" H 9475 4650 50  0001 C CNN
	1    9475 4650
	1    0    0    -1  
$EndComp
Connection ~ 9475 4650
$Comp
L power:GND #PWR0114
U 1 1 5ECD6FF1
P 7325 4750
F 0 "#PWR0114" H 7325 4500 50  0001 C CNN
F 1 "GND" H 7330 4577 50  0000 C CNN
F 2 "" H 7325 4750 50  0001 C CNN
F 3 "" H 7325 4750 50  0001 C CNN
	1    7325 4750
	1    0    0    -1  
$EndComp
Wire Wire Line
	8725 1650 8425 1650
Wire Wire Line
	8425 1650 8425 3550
Wire Wire Line
	8425 3550 7925 3550
Wire Wire Line
	8725 1550 8325 1550
Wire Wire Line
	8325 1550 8325 3650
Wire Wire Line
	8325 3650 7925 3650
Wire Wire Line
	8725 1450 8225 1450
Wire Wire Line
	8225 1450 8225 3750
Wire Wire Line
	8225 3750 7925 3750
$Comp
L power:GND #PWR0115
U 1 1 5ECDDBBF
P 6475 1975
F 0 "#PWR0115" H 6475 1725 50  0001 C CNN
F 1 "GND" H 6480 1802 50  0000 C CNN
F 2 "" H 6475 1975 50  0001 C CNN
F 3 "" H 6475 1975 50  0001 C CNN
	1    6475 1975
	1    0    0    -1  
$EndComp
Wire Wire Line
	6725 1950 6475 1950
Wire Wire Line
	6475 1950 6475 1975
Wire Wire Line
	8725 1750 8475 1750
Wire Wire Line
	8475 1750 8475 3350
Wire Wire Line
	8475 3350 7925 3350
Wire Wire Line
	8725 1950 8550 1950
Wire Wire Line
	8550 1950 8550 3450
Wire Wire Line
	8550 3450 7925 3450
Wire Wire Line
	8725 2050 8725 3050
Wire Wire Line
	8725 3050 7925 3050
$Comp
L Switch:SW_Push SW2
U 1 1 5ECE5E4D
P 800 7050
F 0 "SW2" H 1200 7050 50  0000 C CNN
F 1 "SW_Push" H 800 7244 50  0001 C CNN
F 2 "" H 800 7250 50  0001 C CNN
F 3 "~" H 800 7250 50  0001 C CNN
	1    800  7050
	0    -1   1    0   
$EndComp
$Comp
L newlib:C C2
U 1 1 5ECE9C6C
P 925 7100
F 0 "C2" V 925 6550 60  0000 L CNN
F 1 "10n" V 925 6700 60  0000 L CNN
F 2 "" V 925 7100 60  0000 C CNN
F 3 "" V 925 7100 60  0000 C CNN
	1    925  7100
	-1   0    0    -1  
$EndComp
Wire Wire Line
	925  6950 925  6850
Wire Wire Line
	925  6850 800  6850
$Comp
L Switch:SW_Push SW3
U 1 1 5ED038DB
P 1175 7050
F 0 "SW3" H 1575 7050 50  0000 C CNN
F 1 "SW_Push" H 1175 7244 50  0001 C CNN
F 2 "" H 1175 7250 50  0001 C CNN
F 3 "~" H 1175 7250 50  0001 C CNN
	1    1175 7050
	0    -1   1    0   
$EndComp
$Comp
L newlib:C C3
U 1 1 5ED038E1
P 1300 7100
F 0 "C3" V 1300 6550 60  0000 L CNN
F 1 "10n" V 1300 6700 60  0000 L CNN
F 2 "" V 1300 7100 60  0000 C CNN
F 3 "" V 1300 7100 60  0000 C CNN
	1    1300 7100
	-1   0    0    -1  
$EndComp
Wire Wire Line
	1300 6850 1175 6850
$Comp
L Switch:SW_Push SW4
U 1 1 5ED0B475
P 1525 7050
F 0 "SW4" H 1925 7050 50  0000 C CNN
F 1 "SW_Push" H 1525 7244 50  0001 C CNN
F 2 "" H 1525 7250 50  0001 C CNN
F 3 "~" H 1525 7250 50  0001 C CNN
	1    1525 7050
	0    -1   1    0   
$EndComp
$Comp
L newlib:C C5
U 1 1 5ED0B47B
P 1650 7100
F 0 "C5" V 1650 6550 60  0000 L CNN
F 1 "10n" V 1650 6700 60  0000 L CNN
F 2 "" V 1650 7100 60  0000 C CNN
F 3 "" V 1650 7100 60  0000 C CNN
	1    1650 7100
	-1   0    0    -1  
$EndComp
Wire Wire Line
	1650 6950 1650 6850
Wire Wire Line
	1650 6850 1525 6850
$Comp
L Switch:SW_Push SW5
U 1 1 5ED0B484
P 1900 7050
F 0 "SW5" H 2300 7050 50  0000 C CNN
F 1 "SW_Push" H 1900 7244 50  0001 C CNN
F 2 "" H 1900 7250 50  0001 C CNN
F 3 "~" H 1900 7250 50  0001 C CNN
	1    1900 7050
	0    -1   1    0   
$EndComp
$Comp
L newlib:C C6
U 1 1 5ED0B48A
P 2025 7100
F 0 "C6" V 2025 6550 60  0000 L CNN
F 1 "10n" V 2025 6700 60  0000 L CNN
F 2 "" V 2025 7100 60  0000 C CNN
F 3 "" V 2025 7100 60  0000 C CNN
	1    2025 7100
	-1   0    0    -1  
$EndComp
Wire Wire Line
	2025 6850 1900 6850
$Comp
L Switch:SW_Push SW6
U 1 1 5ED10342
P 2275 7050
F 0 "SW6" H 2675 7050 50  0000 C CNN
F 1 "SW_Push" H 2275 7244 50  0001 C CNN
F 2 "" H 2275 7250 50  0001 C CNN
F 3 "~" H 2275 7250 50  0001 C CNN
	1    2275 7050
	0    -1   1    0   
$EndComp
$Comp
L newlib:C C7
U 1 1 5ED10348
P 2400 7100
F 0 "C7" V 2400 6550 60  0000 L CNN
F 1 "10n" V 2400 6700 60  0000 L CNN
F 2 "" V 2400 7100 60  0000 C CNN
F 3 "" V 2400 7100 60  0000 C CNN
	1    2400 7100
	-1   0    0    -1  
$EndComp
Wire Wire Line
	2400 6950 2400 6850
Wire Wire Line
	2400 6850 2275 6850
$Comp
L Switch:SW_Push SW7
U 1 1 5ED10351
P 2650 7050
F 0 "SW7" H 3050 7050 50  0000 C CNN
F 1 "SW_Push" H 2650 7244 50  0001 C CNN
F 2 "" H 2650 7250 50  0001 C CNN
F 3 "~" H 2650 7250 50  0001 C CNN
	1    2650 7050
	0    -1   1    0   
$EndComp
$Comp
L newlib:C C8
U 1 1 5ED10357
P 2775 7100
F 0 "C8" V 2775 6550 60  0000 L CNN
F 1 "10n" V 2775 6700 60  0000 L CNN
F 2 "" V 2775 7100 60  0000 C CNN
F 3 "" V 2775 7100 60  0000 C CNN
	1    2775 7100
	-1   0    0    -1  
$EndComp
Wire Wire Line
	2775 6950 2775 6850
Wire Wire Line
	2775 6850 2650 6850
$Comp
L Switch:SW_Push SW8
U 1 1 5ED10360
P 3000 7050
F 0 "SW8" H 3400 7050 50  0000 C CNN
F 1 "SW_Push" H 3000 7244 50  0001 C CNN
F 2 "" H 3000 7250 50  0001 C CNN
F 3 "~" H 3000 7250 50  0001 C CNN
	1    3000 7050
	0    -1   1    0   
$EndComp
$Comp
L newlib:C C11
U 1 1 5ED10366
P 3125 7100
F 0 "C11" V 3125 6550 60  0000 L CNN
F 1 "10n" V 3125 6700 60  0000 L CNN
F 2 "" V 3125 7100 60  0000 C CNN
F 3 "" V 3125 7100 60  0000 C CNN
	1    3125 7100
	-1   0    0    -1  
$EndComp
Wire Wire Line
	3125 6950 3125 6850
Wire Wire Line
	3125 6850 3000 6850
$Comp
L Switch:SW_Push SW9
U 1 1 5ED1036F
P 3375 7050
F 0 "SW9" H 3775 7050 50  0000 C CNN
F 1 "SW_Push" H 3375 7244 50  0001 C CNN
F 2 "" H 3375 7250 50  0001 C CNN
F 3 "~" H 3375 7250 50  0001 C CNN
	1    3375 7050
	0    -1   1    0   
$EndComp
$Comp
L newlib:C C12
U 1 1 5ED10375
P 3500 7100
F 0 "C12" V 3500 6550 60  0000 L CNN
F 1 "10n" V 3500 6700 60  0000 L CNN
F 2 "" V 3500 7100 60  0000 C CNN
F 3 "" V 3500 7100 60  0000 C CNN
	1    3500 7100
	-1   0    0    -1  
$EndComp
Wire Wire Line
	3500 6950 3500 6850
Wire Wire Line
	3500 6850 3375 6850
Wire Wire Line
	800  7250 925  7250
Connection ~ 925  7250
Wire Wire Line
	925  7250 1175 7250
Connection ~ 1175 7250
Wire Wire Line
	1175 7250 1300 7250
Connection ~ 1300 7250
Wire Wire Line
	1300 7250 1525 7250
Connection ~ 1525 7250
Wire Wire Line
	1525 7250 1650 7250
Connection ~ 1650 7250
Wire Wire Line
	1650 7250 1900 7250
Connection ~ 1900 7250
Wire Wire Line
	1900 7250 2025 7250
Connection ~ 2025 7250
Wire Wire Line
	2025 7250 2275 7250
Connection ~ 2275 7250
Wire Wire Line
	2275 7250 2400 7250
Connection ~ 2400 7250
Wire Wire Line
	2400 7250 2650 7250
Connection ~ 2650 7250
Wire Wire Line
	2650 7250 2775 7250
Connection ~ 2775 7250
Wire Wire Line
	2775 7250 3000 7250
Connection ~ 3000 7250
Wire Wire Line
	3000 7250 3125 7250
Connection ~ 3125 7250
Wire Wire Line
	3125 7250 3375 7250
Connection ~ 3375 7250
Wire Wire Line
	3375 7250 3500 7250
$Comp
L power:GND #PWR0116
U 1 1 5ED28004
P 3700 7250
F 0 "#PWR0116" H 3700 7000 50  0001 C CNN
F 1 "GND" H 3705 7077 50  0000 C CNN
F 2 "" H 3700 7250 50  0001 C CNN
F 3 "" H 3700 7250 50  0001 C CNN
	1    3700 7250
	1    0    0    -1  
$EndComp
Wire Wire Line
	3500 7250 3700 7250
Connection ~ 3500 7250
Wire Wire Line
	1900 6125 925  6125
Wire Wire Line
	2000 6125 2000 6225
Wire Wire Line
	2000 6225 1300 6225
Wire Wire Line
	1300 6225 1300 6850
Connection ~ 1300 6850
Wire Wire Line
	1300 6850 1300 6950
Wire Wire Line
	2100 6125 2100 6325
Wire Wire Line
	2100 6325 1650 6325
Wire Wire Line
	1650 6325 1650 6850
Connection ~ 1650 6850
Wire Wire Line
	2200 6125 2200 6475
Wire Wire Line
	2200 6475 2025 6475
Wire Wire Line
	2025 6475 2025 6850
Connection ~ 2025 6850
Wire Wire Line
	2025 6850 2025 6950
Wire Wire Line
	2300 6125 2300 6475
Wire Wire Line
	2300 6475 2400 6475
Wire Wire Line
	2400 6475 2400 6850
Connection ~ 2400 6850
Wire Wire Line
	2400 6125 2400 6400
Wire Wire Line
	2400 6400 2775 6400
Wire Wire Line
	2775 6400 2775 6850
Connection ~ 2775 6850
Wire Wire Line
	2500 6125 2500 6300
Wire Wire Line
	2500 6300 3125 6300
Wire Wire Line
	3125 6300 3125 6850
Connection ~ 3125 6850
Wire Wire Line
	2600 6125 3500 6125
Wire Wire Line
	3500 6125 3500 6850
Connection ~ 3500 6850
Wire Wire Line
	925  6850 925  6125
Connection ~ 925  6850
$Comp
L power:GND #PWR0117
U 1 1 5EDAE32D
P 3000 5625
F 0 "#PWR0117" H 3000 5375 50  0001 C CNN
F 1 "GND" H 3005 5452 50  0000 C CNN
F 2 "" H 3000 5625 50  0001 C CNN
F 3 "" H 3000 5625 50  0001 C CNN
	1    3000 5625
	1    0    0    -1  
$EndComp
$Comp
L newlib:C C4
U 1 1 5EDB5989
P 1475 5775
F 0 "C4" H 975 5775 60  0000 L CNN
F 1 "100n" H 1125 5775 60  0000 L CNN
F 2 "" V 1475 5775 60  0000 C CNN
F 3 "" V 1475 5775 60  0000 C CNN
	1    1475 5775
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0118
U 1 1 5EDB598F
P 1475 5925
F 0 "#PWR0118" H 1475 5675 50  0001 C CNN
F 1 "GND" H 1275 5875 50  0000 C CNN
F 2 "" H 1475 5925 50  0001 C CNN
F 3 "" H 1475 5925 50  0001 C CNN
	1    1475 5925
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0119
U 1 1 5EDC32BD
P 1475 5625
F 0 "#PWR0119" H 1475 5475 50  0001 C CNN
F 1 "+3.3V" H 1490 5798 50  0000 C CNN
F 2 "" H 1475 5625 50  0001 C CNN
F 3 "" H 1475 5625 50  0001 C CNN
	1    1475 5625
	1    0    0    -1  
$EndComp
Wire Wire Line
	1600 5625 1475 5625
Connection ~ 1475 5625
$Comp
L newlib:HT7750A U4
U 1 1 5EDCB66A
P 4400 5125
F 0 "U4" H 4075 4875 60  0000 C CNN
F 1 "ME2108A33" H 4400 4775 60  0000 C CNN
F 2 "" H 4400 5125 60  0000 C CNN
F 3 "" H 4400 5125 60  0000 C CNN
	1    4400 5125
	-1   0    0    -1  
$EndComp
$Comp
L Device:L L1
U 1 1 5EDD0303
P 5050 5025
F 0 "L1" V 5240 5025 50  0000 C CNN
F 1 "33uH" V 5149 5025 50  0000 C CNN
F 2 "" H 5050 5025 50  0001 C CNN
F 3 "~" H 5050 5025 50  0001 C CNN
	1    5050 5025
	0    -1   -1   0   
$EndComp
$Comp
L newlib:C C15
U 1 1 5EDDE8B1
P 5200 5175
F 0 "C15" H 5282 5228 60  0000 L CNN
F 1 "10uF" H 5282 5122 60  0000 L CNN
F 2 "" V 5200 5175 60  0000 C CNN
F 3 "" V 5200 5175 60  0000 C CNN
	1    5200 5175
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0120
U 1 1 5EDDE8B7
P 5200 5325
F 0 "#PWR0120" H 5200 5075 50  0001 C CNN
F 1 "GND" H 5205 5152 50  0000 C CNN
F 2 "" H 5200 5325 50  0001 C CNN
F 3 "" H 5200 5325 50  0001 C CNN
	1    5200 5325
	1    0    0    -1  
$EndComp
$Comp
L Device:D_Schottky D2
U 1 1 5EDE27B5
P 4425 4775
F 0 "D2" H 4425 4992 50  0000 C CNN
F 1 "SS14" H 4425 4901 50  0000 C CNN
F 2 "" H 4425 4775 50  0001 C CNN
F 3 "~" H 4425 4775 50  0001 C CNN
	1    4425 4775
	1    0    0    -1  
$EndComp
Wire Wire Line
	4900 5025 4900 4775
Wire Wire Line
	4900 4775 4575 4775
Connection ~ 4900 5025
Wire Wire Line
	3900 5025 3900 4775
Wire Wire Line
	3900 4775 4275 4775
$Comp
L newlib:CP C13
U 1 1 5EE009D0
P 3650 5175
F 0 "C13" H 3475 5300 60  0000 L CNN
F 1 "100uF" H 3350 5075 60  0000 L CNN
F 2 "" V 3650 5175 60  0000 C CNN
F 3 "" V 3650 5175 60  0000 C CNN
	1    3650 5175
	1    0    0    -1  
$EndComp
Wire Wire Line
	3900 5025 3650 5025
Connection ~ 3900 5025
$Comp
L power:GND #PWR0121
U 1 1 5EE055A9
P 3900 5225
F 0 "#PWR0121" H 3900 4975 50  0001 C CNN
F 1 "GND" H 3905 5052 50  0000 C CNN
F 2 "" H 3900 5225 50  0001 C CNN
F 3 "" H 3900 5225 50  0001 C CNN
	1    3900 5225
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0122
U 1 1 5EE08179
P 3650 5325
F 0 "#PWR0122" H 3650 5075 50  0001 C CNN
F 1 "GND" H 3655 5152 50  0000 C CNN
F 2 "" H 3650 5325 50  0001 C CNN
F 3 "" H 3650 5325 50  0001 C CNN
	1    3650 5325
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0123
U 1 1 5EE0B62D
P 3650 5025
F 0 "#PWR0123" H 3650 4875 50  0001 C CNN
F 1 "+3.3V" H 3665 5198 50  0000 C CNN
F 2 "" H 3650 5025 50  0001 C CNN
F 3 "" H 3650 5025 50  0001 C CNN
	1    3650 5025
	1    0    0    -1  
$EndComp
Connection ~ 3650 5025
$Comp
L power:+1V5 #PWR0124
U 1 1 5EE0BE32
P 5200 5025
F 0 "#PWR0124" H 5200 4875 50  0001 C CNN
F 1 "+1V5" H 5325 5175 50  0000 C CNN
F 2 "" H 5200 5025 50  0001 C CNN
F 3 "" H 5200 5025 50  0001 C CNN
	1    5200 5025
	1    0    0    -1  
$EndComp
Connection ~ 5200 5025
Wire Wire Line
	2100 4225 2200 4225
Connection ~ 2200 4225
Wire Wire Line
	2200 4225 2300 4225
$Comp
L power:GND #PWR0125
U 1 1 5EE20640
P 2200 4225
F 0 "#PWR0125" H 2200 3975 50  0001 C CNN
F 1 "GND" H 2205 4052 50  0000 C CNN
F 2 "" H 2200 4225 50  0001 C CNN
F 3 "" H 2200 4225 50  0001 C CNN
	1    2200 4225
	1    0    0    -1  
$EndComp
Wire Wire Line
	2100 1025 2200 1025
Connection ~ 2200 1025
$Comp
L power:+3.3V #PWR0126
U 1 1 5EE26198
P 2200 1025
F 0 "#PWR0126" H 2200 875 50  0001 C CNN
F 1 "+3.3V" H 2215 1198 50  0000 C CNN
F 2 "" H 2200 1025 50  0001 C CNN
F 3 "" H 2200 1025 50  0001 C CNN
	1    2200 1025
	1    0    0    -1  
$EndComp
Connection ~ 2300 1025
Wire Wire Line
	2300 1025 2400 1025
Wire Wire Line
	2200 1025 2300 1025
$Comp
L power:GND #PWR0127
U 1 1 5EE2B781
P 2850 1325
F 0 "#PWR0127" H 2850 1075 50  0001 C CNN
F 1 "GND" H 2855 1152 50  0000 C CNN
F 2 "" H 2850 1325 50  0001 C CNN
F 3 "" H 2850 1325 50  0001 C CNN
	1    2850 1325
	1    0    0    -1  
$EndComp
$Comp
L newlib:C C9
U 1 1 5EE2B77B
P 2850 1175
F 0 "C9" H 2700 1050 60  0000 L CNN
F 1 "100n" H 2725 1475 60  0000 L CNN
F 2 "" V 2850 1175 60  0000 C CNN
F 3 "" V 2850 1175 60  0000 C CNN
	1    2850 1175
	1    0    0    -1  
$EndComp
$Comp
L MCU_ST_STM32F0:STM32F030C8Tx U1
U 1 1 5EC948CC
P 2200 2625
F 0 "U1" H 1600 1025 50  0000 C CNN
F 1 "STM32F030C8Tx" H 1775 925 50  0000 C CNN
F 2 "Package_QFP:LQFP-48_7x7mm_P0.5mm" H 1700 1125 50  0001 R CNN
F 3 "http://www.st.com/st-web-ui/static/active/en/resource/technical/document/datasheet/DM00088500.pdf" H 2200 2625 50  0001 C CNN
	1    2200 2625
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0128
U 1 1 5EE3A985
P 3075 1325
F 0 "#PWR0128" H 3075 1075 50  0001 C CNN
F 1 "GND" H 3080 1152 50  0000 C CNN
F 2 "" H 3075 1325 50  0001 C CNN
F 3 "" H 3075 1325 50  0001 C CNN
	1    3075 1325
	1    0    0    -1  
$EndComp
$Comp
L newlib:C C10
U 1 1 5EE3A98B
P 3075 1175
F 0 "C10" H 3200 1050 60  0000 L CNN
F 1 "100n" H 3075 1475 60  0000 L CNN
F 2 "" V 3075 1175 60  0000 C CNN
F 3 "" V 3075 1175 60  0000 C CNN
	1    3075 1175
	1    0    0    -1  
$EndComp
Wire Wire Line
	3075 1025 2850 1025
Connection ~ 2850 1025
Connection ~ 2400 1025
Wire Wire Line
	2400 1025 2850 1025
$Comp
L power:GND #PWR0129
U 1 1 5EE8D4E8
P 1350 1450
F 0 "#PWR0129" H 1350 1200 50  0001 C CNN
F 1 "GND" H 1355 1277 50  0000 C CNN
F 2 "" H 1350 1450 50  0001 C CNN
F 3 "" H 1350 1450 50  0001 C CNN
	1    1350 1450
	1    0    0    -1  
$EndComp
Wire Wire Line
	1600 1425 1350 1425
Wire Wire Line
	1350 1425 1350 1450
$Comp
L Switch:SW_Push SW1
U 1 1 5EE95A85
P 725 1425
F 0 "SW1" H 1000 1550 50  0000 C CNN
F 1 "SW_Push" H 725 1619 50  0001 C CNN
F 2 "" H 725 1625 50  0001 C CNN
F 3 "~" H 725 1625 50  0001 C CNN
	1    725  1425
	0    -1   1    0   
$EndComp
$Comp
L newlib:C C1
U 1 1 5EE95A8B
P 850 1475
F 0 "C1" V 700 1300 60  0000 L CNN
F 1 "10n" V 700 1500 60  0000 L CNN
F 2 "" V 850 1475 60  0000 C CNN
F 3 "" V 850 1475 60  0000 C CNN
	1    850  1475
	-1   0    0    -1  
$EndComp
Wire Wire Line
	850  1325 850  1225
Wire Wire Line
	725  1625 850  1625
$Comp
L power:GND #PWR0130
U 1 1 5EEA7F70
P 850 1625
F 0 "#PWR0130" H 850 1375 50  0001 C CNN
F 1 "GND" H 855 1452 50  0000 C CNN
F 2 "" H 850 1625 50  0001 C CNN
F 3 "" H 850 1625 50  0001 C CNN
	1    850  1625
	1    0    0    -1  
$EndComp
Connection ~ 850  1625
Wire Wire Line
	725  1225 850  1225
Connection ~ 850  1225
Wire Wire Line
	850  1225 1600 1225
Wire Wire Line
	2200 5125 2300 5125
Connection ~ 2300 5125
Wire Wire Line
	2300 5125 2400 5125
Wire Wire Line
	2400 5125 2400 4900
Wire Wire Line
	2400 4900 3000 4900
Wire Wire Line
	3000 4900 3000 5625
Connection ~ 2400 5125
Connection ~ 3000 5625
Wire Wire Line
	1900 5125 1475 5125
Wire Wire Line
	1200 5125 1200 3125
Wire Wire Line
	1200 3125 1600 3125
Wire Wire Line
	2000 5125 2000 5025
Wire Wire Line
	2000 5025 1750 5025
Wire Wire Line
	1325 5025 1325 3225
Wire Wire Line
	1325 3225 1600 3225
$Comp
L Device:LED D1
U 1 1 5EECF6E3
P 800 4225
F 0 "D1" V 839 4107 50  0000 R CNN
F 1 "LED" V 748 4107 50  0000 R CNN
F 2 "" H 800 4225 50  0001 C CNN
F 3 "~" H 800 4225 50  0001 C CNN
	1    800  4225
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0131
U 1 1 5EED532F
P 800 4375
F 0 "#PWR0131" H 800 4125 50  0001 C CNN
F 1 "GND" H 805 4202 50  0000 C CNN
F 2 "" H 800 4375 50  0001 C CNN
F 3 "" H 800 4375 50  0001 C CNN
	1    800  4375
	1    0    0    -1  
$EndComp
$Comp
L Device:R R1
U 1 1 5EED9D58
P 800 3925
F 0 "R1" H 870 3971 50  0000 L CNN
F 1 "3K3" H 870 3880 50  0000 L CNN
F 2 "" V 730 3925 50  0001 C CNN
F 3 "~" H 800 3925 50  0001 C CNN
	1    800  3925
	1    0    0    -1  
$EndComp
Wire Wire Line
	1600 3725 800  3725
Wire Wire Line
	800  3725 800  3775
$Comp
L Device:LED D3
U 1 1 5EEF690F
P 6250 4750
F 0 "D3" V 6289 4632 50  0000 R CNN
F 1 "LED" V 6198 4632 50  0000 R CNN
F 2 "" H 6250 4750 50  0001 C CNN
F 3 "~" H 6250 4750 50  0001 C CNN
	1    6250 4750
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0132
U 1 1 5EEF6915
P 6250 4900
F 0 "#PWR0132" H 6250 4650 50  0001 C CNN
F 1 "GND" H 6255 4727 50  0000 C CNN
F 2 "" H 6250 4900 50  0001 C CNN
F 3 "" H 6250 4900 50  0001 C CNN
	1    6250 4900
	1    0    0    -1  
$EndComp
$Comp
L Device:R R5
U 1 1 5EEF691B
P 6250 4450
F 0 "R5" H 6320 4496 50  0000 L CNN
F 1 "3K3" H 6320 4405 50  0000 L CNN
F 2 "" V 6180 4450 50  0001 C CNN
F 3 "~" H 6250 4450 50  0001 C CNN
	1    6250 4450
	1    0    0    -1  
$EndComp
Wire Wire Line
	6250 4250 6250 4300
Wire Wire Line
	6725 4250 6250 4250
$Comp
L Device:R R2
U 1 1 5EF15CCF
P 1475 4800
F 0 "R2" H 1545 4846 50  0000 L CNN
F 1 "3K3" H 1545 4755 50  0000 L CNN
F 2 "" V 1405 4800 50  0001 C CNN
F 3 "~" H 1475 4800 50  0001 C CNN
	1    1475 4800
	1    0    0    -1  
$EndComp
Wire Wire Line
	1475 4600 1475 4650
$Comp
L Device:R R3
U 1 1 5EF1B653
P 1750 4800
F 0 "R3" H 1820 4846 50  0000 L CNN
F 1 "3K3" H 1820 4755 50  0000 L CNN
F 2 "" V 1680 4800 50  0001 C CNN
F 3 "~" H 1750 4800 50  0001 C CNN
	1    1750 4800
	1    0    0    -1  
$EndComp
Wire Wire Line
	1750 4600 1750 4650
Wire Wire Line
	1750 4950 1750 5025
Connection ~ 1750 5025
Wire Wire Line
	1750 5025 1325 5025
Wire Wire Line
	1475 4950 1475 5125
Connection ~ 1475 5125
Wire Wire Line
	1475 5125 1200 5125
Wire Wire Line
	1475 4600 1575 4600
$Comp
L power:+3.3V #PWR0133
U 1 1 5EF390B4
P 1575 4600
F 0 "#PWR0133" H 1575 4450 50  0001 C CNN
F 1 "+3.3V" H 1590 4773 50  0000 C CNN
F 2 "" H 1575 4600 50  0001 C CNN
F 3 "" H 1575 4600 50  0001 C CNN
	1    1575 4600
	1    0    0    -1  
$EndComp
Connection ~ 1575 4600
Wire Wire Line
	1575 4600 1750 4600
Wire Wire Line
	2800 3025 3550 3025
Wire Wire Line
	2800 3125 3375 3125
Wire Wire Line
	3375 3125 3375 2925
Wire Wire Line
	3375 2925 3550 2925
Wire Wire Line
	2800 3225 3300 3225
Wire Wire Line
	3300 3225 3300 2825
Wire Wire Line
	3300 2825 3550 2825
Wire Wire Line
	2800 2825 3200 2825
Wire Wire Line
	3200 2825 3200 3325
Wire Wire Line
	3200 3325 3550 3325
Wire Wire Line
	2800 2925 3050 2925
Wire Wire Line
	3050 2925 3050 2700
Wire Wire Line
	3050 2700 3475 2700
Wire Wire Line
	3475 2700 3475 3125
Wire Wire Line
	3475 3125 3550 3125
Wire Wire Line
	3550 3425 2975 3425
Wire Wire Line
	2975 3425 2975 2525
Wire Wire Line
	2975 2525 2800 2525
Wire Wire Line
	2800 2625 2875 2625
Wire Wire Line
	2875 2625 2875 4500
Wire Wire Line
	2875 5125 2700 5125
$Comp
L Device:R R4
U 1 1 5EFC58B2
P 3025 4500
F 0 "R4" V 3232 4500 50  0000 C CNN
F 1 "10K" V 3141 4500 50  0000 C CNN
F 2 "" V 2955 4500 50  0001 C CNN
F 3 "~" H 3025 4500 50  0001 C CNN
	1    3025 4500
	0    -1   -1   0   
$EndComp
Connection ~ 2875 4500
Wire Wire Line
	2875 4500 2875 5125
$Comp
L power:+3V3 #PWR0134
U 1 1 5EFFA326
P 3175 4500
F 0 "#PWR0134" H 3175 4350 50  0001 C CNN
F 1 "+3V3" H 3190 4673 50  0000 C CNN
F 2 "" H 3175 4500 50  0001 C CNN
F 3 "" H 3175 4500 50  0001 C CNN
	1    3175 4500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0135
U 1 1 5F00F1BE
P 4050 3725
F 0 "#PWR0135" H 4050 3475 50  0001 C CNN
F 1 "GND" H 4055 3552 50  0000 C CNN
F 2 "" H 4050 3725 50  0001 C CNN
F 3 "" H 4050 3725 50  0001 C CNN
	1    4050 3725
	1    0    0    -1  
$EndComp
$Comp
L newlib:C C14
U 1 1 5F01DF88
P 4550 2675
F 0 "C14" H 4632 2728 60  0000 L CNN
F 1 "10uF" H 4632 2622 60  0000 L CNN
F 2 "" V 4550 2675 60  0000 C CNN
F 3 "" V 4550 2675 60  0000 C CNN
	1    4550 2675
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0136
U 1 1 5F01DF8E
P 4550 2825
F 0 "#PWR0136" H 4550 2575 50  0001 C CNN
F 1 "GND" H 4555 2652 50  0000 C CNN
F 2 "" H 4550 2825 50  0001 C CNN
F 3 "" H 4550 2825 50  0001 C CNN
	1    4550 2825
	1    0    0    -1  
$EndComp
Wire Wire Line
	4050 2525 4550 2525
$Comp
L power:+3V3 #PWR0137
U 1 1 5F02E2B9
P 4050 2525
F 0 "#PWR0137" H 4050 2375 50  0001 C CNN
F 1 "+3V3" H 4065 2698 50  0000 C CNN
F 2 "" H 4050 2525 50  0001 C CNN
F 3 "" H 4050 2525 50  0001 C CNN
	1    4050 2525
	1    0    0    -1  
$EndComp
Connection ~ 4050 2525
$EndSCHEMATC
