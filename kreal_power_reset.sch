EESchema Schematic File Version 2
LIBS:KReal-rescue
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
LIBS:cnc
LIBS:TiZed
LIBS:KReal-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 2 3
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
L C_Small C207
U 1 1 58F15AA5
P 6465 2525
F 0 "C207" H 6475 2595 50  0000 L CNN
F 1 "10nF" H 6475 2445 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 6465 2525 50  0001 C CNN
F 3 "" H 6465 2525 50  0001 C CNN
	1    6465 2525
	1    0    0    -1  
$EndComp
$Comp
L C_Small C208
U 1 1 58F15AAC
P 6685 2445
F 0 "C208" H 6695 2515 50  0000 L CNN
F 1 "10nF" H 6695 2365 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 6685 2445 50  0001 C CNN
F 3 "" H 6685 2445 50  0001 C CNN
	1    6685 2445
	1    0    0    -1  
$EndComp
$Comp
L C_Small C209
U 1 1 58F15AB3
P 6905 2365
F 0 "C209" H 6915 2435 50  0000 L CNN
F 1 "10nF" H 6915 2285 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 6905 2365 50  0001 C CNN
F 3 "" H 6905 2365 50  0001 C CNN
	1    6905 2365
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR033
U 1 1 58F15ABA
P 6795 2665
F 0 "#PWR033" H 6795 2415 50  0001 C CNN
F 1 "GND" H 6795 2515 50  0000 C CNN
F 2 "" H 6795 2665 50  0001 C CNN
F 3 "" H 6795 2665 50  0001 C CNN
	1    6795 2665
	1    0    0    -1  
$EndComp
$Comp
L PIC32MX775F-RESCUE-KReal U?
U 1 1 58F15AC0
P 5505 2745
AR Path="/58F15AC0" Ref="U?"  Part="1" 
AR Path="/58F14E7A/58F15AC0" Ref="U101"  Part="1" 
F 0 "U101" H 5275 2015 55  0000 C CNN
F 1 "PIC32MX775F" H 5505 3455 63  0000 C CNN
F 2 "Housings_QFP:TQFP-64_10x10mm_Pitch0.5mm" H 6015 2635 60  0001 C CNN
F 3 "" H 6015 2635 60  0001 C CNN
	1    5505 2745
	1    0    0    -1  
$EndComp
$Comp
L C_Small C210
U 1 1 58F15AC7
P 7125 2285
F 0 "C210" H 7135 2355 50  0000 L CNN
F 1 "100nF" H 7135 2205 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 7125 2285 50  0001 C CNN
F 3 "" H 7125 2285 50  0001 C CNN
	1    7125 2285
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR034
U 1 1 58F15ACE
P 5925 3425
F 0 "#PWR034" H 5925 3175 50  0001 C CNN
F 1 "GND" H 5925 3275 50  0000 C CNN
F 2 "" H 5925 3425 50  0001 C CNN
F 3 "" H 5925 3425 50  0001 C CNN
	1    5925 3425
	1    0    0    -1  
$EndComp
$Comp
L +3V3 #PWR035
U 1 1 58F15AD4
P 6005 2025
F 0 "#PWR035" H 6005 1875 50  0001 C CNN
F 1 "+3V3" H 6005 2165 50  0000 C CNN
F 2 "" H 6005 2025 50  0001 C CNN
F 3 "" H 6005 2025 50  0001 C CNN
	1    6005 2025
	1    0    0    -1  
$EndComp
$Comp
L C_Small C206
U 1 1 58F15B03
P 6185 3205
F 0 "C206" H 6195 3275 50  0000 L CNN
F 1 "10uF" H 6195 3125 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 6185 3205 50  0001 C CNN
F 3 "" H 6185 3205 50  0001 C CNN
	1    6185 3205
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR036
U 1 1 58F15B0A
P 6185 3425
F 0 "#PWR036" H 6185 3175 50  0001 C CNN
F 1 "GND" H 6185 3275 50  0000 C CNN
F 2 "" H 6185 3425 50  0001 C CNN
F 3 "" H 6185 3425 50  0001 C CNN
	1    6185 3425
	1    0    0    -1  
$EndComp
Text Label 5925 2865 0    60   ~ 0
VPP
Text GLabel 6450 3070 2    60   Input ~ 0
VBUS
$Comp
L BSS138-RESCUE-KReal Q201
U 1 1 58F16BCD
P 8980 1645
F 0 "Q201" H 9180 1720 50  0000 L CNN
F 1 "BSS138" H 9180 1645 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:SOT-23" H 9180 1570 50  0001 L CIN
F 3 "" H 8980 1645 50  0001 L CNN
	1    8980 1645
	-1   0    0    -1  
$EndComp
$Comp
L CONN_01X06 P201
U 1 1 58F16BD4
P 10680 1305
F 0 "P201" H 10680 1655 50  0000 C CNN
F 1 "ICSP" V 10780 1305 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x06_Pitch2.54mm" H 10680 1305 50  0001 C CNN
F 3 "" H 10680 1305 50  0001 C CNN
	1    10680 1305
	1    0    0    -1  
$EndComp
$Comp
L +3V3 #PWR037
U 1 1 58F16BDB
P 10280 865
F 0 "#PWR037" H 10280 715 50  0001 C CNN
F 1 "+3V3" H 10280 1005 50  0000 C CNN
F 2 "" H 10280 865 50  0001 C CNN
F 3 "" H 10280 865 50  0001 C CNN
	1    10280 865 
	1    0    0    -1  
$EndComp
$Comp
L R_Small R213
U 1 1 58F16BE3
P 10505 2035
F 0 "R213" V 10360 1935 50  0000 L CNN
F 1 "51" V 10430 2015 50  0000 L CNN
F 2 "Resistors_SMD:R_0603" H 10505 2035 50  0001 C CNN
F 3 "" H 10505 2035 50  0001 C CNN
	1    10505 2035
	0    1    1    0   
$EndComp
$Comp
L R_Small R212
U 1 1 58F16BEA
P 10505 2205
F 0 "R212" V 10435 2115 50  0000 L CNN
F 1 "51" V 10590 2165 50  0000 L CNN
F 2 "Resistors_SMD:R_0603" H 10505 2205 50  0001 C CNN
F 3 "" H 10505 2205 50  0001 C CNN
	1    10505 2205
	0    1    1    0   
$EndComp
$Comp
L GND #PWR038
U 1 1 58F16BF9
P 9980 1325
F 0 "#PWR038" H 9980 1075 50  0001 C CNN
F 1 "GND" H 9980 1175 50  0000 C CNN
F 2 "" H 9980 1325 50  0001 C CNN
F 3 "" H 9980 1325 50  0001 C CNN
	1    9980 1325
	1    0    0    -1  
$EndComp
$Comp
L R_Small R209
U 1 1 58F16C01
P 8880 2025
F 0 "R209" H 8910 2045 50  0000 L CNN
F 1 "220" H 8910 1985 50  0000 L CNN
F 2 "Resistors_SMD:R_0603" H 8880 2025 50  0001 C CNN
F 3 "" H 8880 2025 50  0001 C CNN
	1    8880 2025
	1    0    0    -1  
$EndComp
$Comp
L R_Small R210
U 1 1 58F16C08
P 9215 2025
F 0 "R210" H 9245 2045 50  0000 L CNN
F 1 "20K" H 9245 1985 50  0000 L CNN
F 2 "Resistors_SMD:R_0603" H 9215 2025 50  0001 C CNN
F 3 "" H 9215 2025 50  0001 C CNN
	1    9215 2025
	1    0    0    -1  
$EndComp
$Comp
L R_Small R207
U 1 1 58F16C0F
P 8620 1345
F 0 "R207" V 8680 1355 50  0000 L CNN
F 1 "51" V 8540 1325 50  0000 L CNN
F 2 "Resistors_SMD:R_0603" H 8620 1345 50  0001 C CNN
F 3 "" H 8620 1345 50  0001 C CNN
	1    8620 1345
	0    -1   -1   0   
$EndComp
$Comp
L R_Small R208
U 1 1 58F16C16
P 8880 1165
F 0 "R208" H 8910 1185 50  0000 L CNN
F 1 "10K" H 8910 1125 50  0000 L CNN
F 2 "Resistors_SMD:R_0603" H 8880 1165 50  0001 C CNN
F 3 "" H 8880 1165 50  0001 C CNN
	1    8880 1165
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR039
U 1 1 58F16C1D
P 8330 1655
F 0 "#PWR039" H 8330 1405 50  0001 C CNN
F 1 "GND" H 8330 1505 50  0000 C CNN
F 2 "" H 8330 1655 50  0001 C CNN
F 3 "" H 8330 1655 50  0001 C CNN
	1    8330 1655
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR040
U 1 1 58F16C2D
P 8880 2185
F 0 "#PWR040" H 8880 1935 50  0001 C CNN
F 1 "GND" H 8880 2035 50  0000 C CNN
F 2 "" H 8880 2185 50  0001 C CNN
F 3 "" H 8880 2185 50  0001 C CNN
	1    8880 2185
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR041
U 1 1 58F16C33
P 9215 2185
F 0 "#PWR041" H 9215 1935 50  0001 C CNN
F 1 "GND" H 9215 2035 50  0000 C CNN
F 2 "" H 9215 2185 50  0001 C CNN
F 3 "" H 9215 2185 50  0001 C CNN
	1    9215 2185
	1    0    0    -1  
$EndComp
$Comp
L +3V3 #PWR042
U 1 1 58F16C3C
P 8880 1005
F 0 "#PWR042" H 8880 855 50  0001 C CNN
F 1 "+3V3" H 8880 1145 50  0000 C CNN
F 2 "" H 8880 1005 50  0001 C CNN
F 3 "" H 8880 1005 50  0001 C CNN
	1    8880 1005
	1    0    0    -1  
$EndComp
$Comp
L +3V3 #PWR043
U 1 1 58F16C42
P 8330 1005
F 0 "#PWR043" H 8330 855 50  0001 C CNN
F 1 "+3V3" H 8330 1145 50  0000 C CNN
F 2 "" H 8330 1005 50  0001 C CNN
F 3 "" H 8330 1005 50  0001 C CNN
	1    8330 1005
	1    0    0    -1  
$EndComp
Text GLabel 9580 1695 2    60   Input ~ 0
Reset
Text Label 8080 1345 2    60   ~ 0
VPP
$Comp
L PIC32MX775F U?
U 3 1 58F191FC
P 9010 4380
AR Path="/58F191FC" Ref="U?"  Part="3" 
AR Path="/58F14E7A/58F191FC" Ref="U101"  Part="3" 
F 0 "U101" H 8240 4120 55  0000 C CNN
F 1 "PIC32MX775F" H 8480 4640 63  0000 C CNN
F 2 "Housings_QFP:TQFP-64_10x10mm_Pitch0.5mm" H 9720 4250 60  0001 C CNN
F 3 "" H 9720 4250 60  0001 C CNN
	3    9010 4380
	1    0    0    -1  
$EndComp
$Comp
L R_Small R211
U 1 1 58F19203
P 9380 4720
F 0 "R211" H 9410 4740 50  0000 L CNN
F 1 "680" H 9410 4680 50  0000 L CNN
F 2 "Resistors_SMD:R_0603" H 9380 4720 50  0001 C CNN
F 3 "" H 9380 4720 50  0001 C CNN
	1    9380 4720
	-1   0    0    -1  
$EndComp
$Comp
L GND #PWR044
U 1 1 58F1920A
P 9580 5360
F 0 "#PWR044" H 9580 5110 50  0001 C CNN
F 1 "GND" H 9580 5210 50  0000 C CNN
F 2 "" H 9580 5360 50  0001 C CNN
F 3 "" H 9580 5360 50  0001 C CNN
	1    9580 5360
	1    0    0    -1  
$EndComp
$Comp
L C_Small C211
U 1 1 58F1921B
P 9380 5160
F 0 "C211" H 9390 5230 50  0000 L CNN
F 1 "30pF" H 9390 5080 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 9380 5160 50  0001 C CNN
F 3 "" H 9380 5160 50  0001 C CNN
	1    9380 5160
	1    0    0    -1  
$EndComp
$Comp
L C_Small C212
U 1 1 58F19222
P 9760 5160
F 0 "C212" H 9770 5230 50  0000 L CNN
F 1 "30pF" H 9770 5080 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 9760 5160 50  0001 C CNN
F 3 "" H 9760 5160 50  0001 C CNN
	1    9760 5160
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR045
U 1 1 58F19237
P 10200 5360
F 0 "#PWR045" H 10200 5110 50  0001 C CNN
F 1 "GND" H 10200 5210 50  0000 C CNN
F 2 "" H 10200 5360 50  0001 C CNN
F 3 "" H 10200 5360 50  0001 C CNN
	1    10200 5360
	1    0    0    -1  
$EndComp
$Comp
L C_Small C213
U 1 1 58F19241
P 10000 5160
F 0 "C213" H 10010 5230 50  0000 L CNN
F 1 "11pF" H 10010 5080 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 10000 5160 50  0001 C CNN
F 3 "" H 10000 5160 50  0001 C CNN
	1    10000 5160
	1    0    0    -1  
$EndComp
$Comp
L C_Small C214
U 1 1 58F19248
P 10380 5160
F 0 "C214" H 10390 5230 50  0000 L CNN
F 1 "11pF" H 10390 5080 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 10380 5160 50  0001 C CNN
F 3 "" H 10380 5160 50  0001 C CNN
	1    10380 5160
	1    0    0    -1  
$EndComp
$Comp
L AP3417C U202
U 1 1 58F23B2B
P 2500 1660
F 0 "U202" H 2500 1410 60  0000 C CNN
F 1 "AP3417C" H 2500 1890 60  0000 C CNN
F 2 "TO_SOT_Packages_SMD:SOT-23-5" H 2510 1550 60  0001 C CNN
F 3 "" H 2510 1550 60  0001 C CNN
	1    2500 1660
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR046
U 1 1 58F23E2E
P 2160 1860
F 0 "#PWR046" H 2160 1610 50  0001 C CNN
F 1 "GND" H 2160 1710 50  0000 C CNN
F 2 "" H 2160 1860 50  0001 C CNN
F 3 "" H 2160 1860 50  0001 C CNN
	1    2160 1860
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR047
U 1 1 58F23EC6
P 1290 1385
F 0 "#PWR047" H 1290 1235 50  0001 C CNN
F 1 "+5V" H 1290 1525 50  0000 C CNN
F 2 "" H 1290 1385 50  0001 C CNN
F 3 "" H 1290 1385 50  0001 C CNN
	1    1290 1385
	1    0    0    -1  
$EndComp
$Comp
L R_Small R205
U 1 1 58F23FBC
P 3120 1920
F 0 "R205" H 3155 1965 50  0000 L CNN
F 1 "100k" H 3155 1880 50  0000 L CNN
F 2 "Resistors_SMD:R_0603" H 3120 1920 50  0001 C CNN
F 3 "" H 3120 1920 50  0001 C CNN
	1    3120 1920
	1    0    0    -1  
$EndComp
$Comp
L C_Small C204
U 1 1 58F244E8
P 3420 1680
F 0 "C204" H 3430 1750 50  0000 L CNN
F 1 "10u" H 3430 1600 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 3420 1680 50  0001 C CNN
F 3 "" H 3420 1680 50  0001 C CNN
	1    3420 1680
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR048
U 1 1 58F24A34
P 3120 2035
F 0 "#PWR048" H 3120 1785 50  0001 C CNN
F 1 "GND" H 3120 1885 50  0000 C CNN
F 2 "" H 3120 2035 50  0001 C CNN
F 3 "" H 3120 2035 50  0001 C CNN
	1    3120 2035
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR049
U 1 1 58F24A96
P 1880 1860
F 0 "#PWR049" H 1880 1610 50  0001 C CNN
F 1 "GND" H 1880 1710 50  0000 C CNN
F 2 "" H 1880 1860 50  0001 C CNN
F 3 "" H 1880 1860 50  0001 C CNN
	1    1880 1860
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR050
U 1 1 58F2501F
P 3420 1820
F 0 "#PWR050" H 3420 1570 50  0001 C CNN
F 1 "GND" H 3420 1670 50  0000 C CNN
F 2 "" H 3420 1820 50  0001 C CNN
F 3 "" H 3420 1820 50  0001 C CNN
	1    3420 1820
	1    0    0    -1  
$EndComp
$Comp
L +3V3 #PWR051
U 1 1 58F25A91
P 3420 1500
F 0 "#PWR051" H 3420 1350 50  0001 C CNN
F 1 "+3V3" H 3420 1640 50  0000 C CNN
F 2 "" H 3420 1500 50  0001 C CNN
F 3 "" H 3420 1500 50  0001 C CNN
	1    3420 1500
	1    0    0    -1  
$EndComp
$Comp
L L_Small L201
U 1 1 58F24098
P 2960 1540
F 0 "L201" V 3020 1570 50  0000 L CNN
F 1 "2.2u" V 2900 1470 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805" H 2960 1540 50  0001 C CNN
F 3 "" H 2960 1540 50  0001 C CNN
	1    2960 1540
	0    -1   -1   0   
$EndComp
$Comp
L R_Small R204
U 1 1 58F23F0A
P 3120 1650
F 0 "R204" H 3155 1695 50  0000 L CNN
F 1 "450k" H 3155 1610 50  0000 L CNN
F 2 "Resistors_SMD:R_0603" H 3120 1650 50  0001 C CNN
F 3 "" H 3120 1650 50  0001 C CNN
	1    3120 1650
	1    0    0    -1  
$EndComp
Text GLabel 10690 2035 2    60   Output ~ 0
ENABLE_X
Text GLabel 10690 2205 2    60   Output ~ 0
STEP_Y
Text Label 10320 1455 0    60   ~ 0
PGC
Text Label 10100 1355 0    60   ~ 0
PGD
$Comp
L MMBZ52xxBTS U204
U 3 1 58F37917
P 10320 2495
F 0 "U204" H 10550 2480 60  0000 C CNN
F 1 "MMBZ5226BTS-3.3v" H 10785 2370 50  0000 C CNN
F 2 "TO_SOT_Packages_SMD:SOT-363_SC-70-6" H 10320 2495 60  0001 C CNN
F 3 "" H 10320 2495 60  0001 C CNN
	3    10320 2495
	1    0    0    -1  
$EndComp
$Comp
L MMBZ52xxBTS U204
U 2 1 58F37C5E
P 10100 2495
F 0 "U204" H 9890 2480 60  0000 C CNN
F 1 "MMBZ5226BTS-3.3v" H 10410 2605 50  0001 C CNN
F 2 "TO_SOT_Packages_SMD:SOT-363_SC-70-6" H 10100 2495 60  0001 C CNN
F 3 "" H 10100 2495 60  0001 C CNN
	2    10100 2495
	1    0    0    -1  
$EndComp
$Comp
L MMBZ52xxBTS U204
U 1 1 58F38141
P 9490 2015
F 0 "U204" H 9685 2105 60  0000 C CNN
F 1 "MMBZ5226BTS-3.3v" H 9800 2125 50  0001 C CNN
F 2 "TO_SOT_Packages_SMD:SOT-363_SC-70-6" H 9490 2015 60  0001 C CNN
F 3 "" H 9490 2015 60  0001 C CNN
	1    9490 2015
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR052
U 1 1 58F3856D
P 9490 2185
F 0 "#PWR052" H 9490 1935 50  0001 C CNN
F 1 "GND" H 9490 2035 50  0000 C CNN
F 2 "" H 9490 2185 50  0001 C CNN
F 3 "" H 9490 2185 50  0001 C CNN
	1    9490 2185
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR053
U 1 1 58F38678
P 10100 2675
F 0 "#PWR053" H 10100 2425 50  0001 C CNN
F 1 "GND" H 10100 2525 50  0000 C CNN
F 2 "" H 10100 2675 50  0001 C CNN
F 3 "" H 10100 2675 50  0001 C CNN
	1    10100 2675
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR054
U 1 1 58F386E3
P 10320 2675
F 0 "#PWR054" H 10320 2425 50  0001 C CNN
F 1 "GND" H 10320 2525 50  0000 C CNN
F 2 "" H 10320 2675 50  0001 C CNN
F 3 "" H 10320 2675 50  0001 C CNN
	1    10320 2675
	1    0    0    -1  
$EndComp
$Comp
L BAV99W D205
U 1 1 58F39092
P 8330 1345
F 0 "D205" H 8400 1385 50  0000 L CNN
F 1 "BAV99W" H 8400 1265 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:SOT-323_SC-70" H 8330 1245 50  0001 C CNN
F 3 "" H 8330 1245 50  0000 C CNN
	1    8330 1345
	-1   0    0    -1  
$EndComp
$Comp
L MC34063 U201
U 1 1 58F3AB63
P 1925 3015
F 0 "U201" H 1925 2755 60  0000 C CNN
F 1 "MC34063" H 1925 3265 60  0000 C CNN
F 2 "Housings_SOIC:SOIC-8_3.9x4.9mm_Pitch1.27mm" H 2035 2975 60  0001 C CNN
F 3 "" H 2035 2975 60  0001 C CNN
	1    1925 3015
	1    0    0    -1  
$EndComp
$Comp
L L_Small L202
U 1 1 58F3AD70
P 2880 2975
F 0 "L202" V 2930 3000 50  0000 L CNN
F 1 "47u" V 2830 2925 50  0000 L CNN
F 2 "Inductors:Inductor_Taiyo-Yuden_MD-5050" H 2880 2975 50  0001 C CNN
F 3 "" H 2880 2975 50  0001 C CNN
	1    2880 2975
	0    -1   -1   0   
$EndComp
$Comp
L C_Small C203
U 1 1 58F3AE93
P 2450 3205
F 0 "C203" H 2460 3275 50  0000 L CNN
F 1 "220p" H 2460 3125 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 2450 3205 50  0001 C CNN
F 3 "" H 2450 3205 50  0001 C CNN
	1    2450 3205
	1    0    0    -1  
$EndComp
$Comp
L D_Schottky_Small D203
U 1 1 58F3B0E8
P 2710 3115
F 0 "D203" V 2635 2990 50  0000 L CNN
F 1 "10BQ100PBF" V 2785 3130 39  0000 L CNN
F 2 "Diodes_SMD:D_SMB_Standard" V 2710 3115 50  0001 C CNN
F 3 "" V 2710 3115 50  0001 C CNN
	1    2710 3115
	0    1    1    0   
$EndComp
$Comp
L CP_Small C205
U 1 1 58F3B92B
P 3210 3115
F 0 "C205" H 3220 3185 50  0000 L CNN
F 1 "330u" H 3220 3035 50  0000 L CNN
F 2 "Capacitors_SMD:CP_Elec_8x10" H 3210 3115 50  0001 C CNN
F 3 "" H 3210 3115 50  0001 C CNN
	1    3210 3115
	1    0    0    -1  
$EndComp
$Comp
L R_Small R203
U 1 1 58F3E392
P 1905 3545
F 0 "R203" V 1835 3550 50  0000 L CNN
F 1 "3k" V 1975 3465 50  0000 L CNN
F 2 "Resistors_SMD:R_0603" H 1905 3545 50  0001 C CNN
F 3 "" H 1905 3545 50  0001 C CNN
	1    1905 3545
	0    1    1    0   
$EndComp
$Comp
L R_Small R202
U 1 1 58F3E49B
P 1545 3730
F 0 "R202" H 1575 3750 50  0000 L CNN
F 1 "1k" H 1575 3690 50  0000 L CNN
F 2 "Resistors_SMD:R_0603" H 1545 3730 50  0001 C CNN
F 3 "" H 1545 3730 50  0001 C CNN
	1    1545 3730
	-1   0    0    1   
$EndComp
$Comp
L R_Small R201
U 1 1 58F3E5C1
P 1400 2975
F 0 "R201" V 1575 2925 50  0000 L CNN
F 1 "0.33" V 1475 2925 50  0000 L CNN
F 2 "Resistors_SMD:R_0805" H 1400 2975 50  0001 C CNN
F 3 "" H 1400 2975 50  0001 C CNN
	1    1400 2975
	0    -1   -1   0   
$EndComp
$Comp
L CP_Small C201
U 1 1 58F3EE6A
P 1300 3255
F 0 "C201" H 1310 3325 50  0000 L CNN
F 1 "100u" H 1310 3175 50  0000 L CNN
F 2 "Capacitors_SMD:CP_Elec_6.3x7.7" H 1300 3255 50  0001 C CNN
F 3 "" H 1300 3255 50  0001 C CNN
	1    1300 3255
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR055
U 1 1 58F3EF41
P 3210 3255
F 0 "#PWR055" H 3210 3005 50  0001 C CNN
F 1 "GND" H 3210 3105 50  0000 C CNN
F 2 "" H 3210 3255 50  0001 C CNN
F 3 "" H 3210 3255 50  0001 C CNN
	1    3210 3255
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR056
U 1 1 58F3F049
P 2710 3255
F 0 "#PWR056" H 2710 3005 50  0001 C CNN
F 1 "GND" H 2710 3105 50  0000 C CNN
F 2 "" H 2710 3255 50  0001 C CNN
F 3 "" H 2710 3255 50  0001 C CNN
	1    2710 3255
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR057
U 1 1 58F3F392
P 2450 3330
F 0 "#PWR057" H 2450 3080 50  0001 C CNN
F 1 "GND" H 2450 3180 50  0000 C CNN
F 2 "" H 2450 3330 50  0001 C CNN
F 3 "" H 2450 3330 50  0001 C CNN
	1    2450 3330
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR058
U 1 1 58F3F493
P 2300 3225
F 0 "#PWR058" H 2300 2975 50  0001 C CNN
F 1 "GND" H 2300 3075 50  0000 C CNN
F 2 "" H 2300 3225 50  0001 C CNN
F 3 "" H 2300 3225 50  0001 C CNN
	1    2300 3225
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR059
U 1 1 58F3F56B
P 1545 3875
F 0 "#PWR059" H 1545 3625 50  0001 C CNN
F 1 "GND" H 1545 3725 50  0000 C CNN
F 2 "" H 1545 3875 50  0001 C CNN
F 3 "" H 1545 3875 50  0001 C CNN
	1    1545 3875
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR060
U 1 1 58F3F79B
P 1300 3395
F 0 "#PWR060" H 1300 3145 50  0001 C CNN
F 1 "GND" H 1300 3245 50  0000 C CNN
F 2 "" H 1300 3395 50  0001 C CNN
F 3 "" H 1300 3395 50  0001 C CNN
	1    1300 3395
	1    0    0    -1  
$EndComp
Text GLabel 1100 3065 0    60   Input ~ 0
VIN
$Comp
L CONN_01X03 J201
U 1 1 58F443C1
P 3660 3245
F 0 "J201" H 3660 3445 50  0000 C CNN
F 1 "5V Bypass" V 3760 3245 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x03_Pitch2.54mm" H 3660 3245 50  0001 C CNN
F 3 "" H 3660 3245 50  0001 C CNN
	1    3660 3245
	0    1    1    0   
$EndComp
Text GLabel 3880 2975 2    60   Input ~ 0
VIN
$Comp
L +5V #PWR061
U 1 1 58F45216
P 3660 2975
F 0 "#PWR061" H 3660 2825 50  0001 C CNN
F 1 "+5V" H 3660 3115 50  0000 C CNN
F 2 "" H 3660 2975 50  0001 C CNN
F 3 "" H 3660 2975 50  0001 C CNN
	1    3660 2975
	1    0    0    -1  
$EndComp
$Comp
L C_Small C202
U 1 1 58F4D0A5
P 1880 1680
F 0 "C202" H 1890 1750 50  0000 L CNN
F 1 "4.7u" H 1890 1600 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 1880 1680 50  0001 C CNN
F 3 "" H 1880 1680 50  0001 C CNN
	1    1880 1680
	1    0    0    -1  
$EndComp
$Comp
L D_Small_ALT D201
U 1 1 58F5728A
P 1470 1420
F 0 "D201" H 1420 1500 50  0000 L CNN
F 1 "PMEG6030ETPX" H 1040 1345 50  0000 L CNN
F 2 "Diodes_SMD:D_SMA_Standard" V 1470 1420 50  0001 C CNN
F 3 "" V 1470 1420 50  0001 C CNN
	1    1470 1420
	-1   0    0    1   
$EndComp
$Comp
L D_Small_ALT D202
U 1 1 58F57348
P 1470 1650
F 0 "D202" H 1420 1730 50  0000 L CNN
F 1 "PMEG6030ETPX" H 1260 1815 50  0000 L CNN
F 2 "Diodes_SMD:D_SMA_Standard" V 1470 1650 50  0001 C CNN
F 3 "" V 1470 1650 50  0001 C CNN
	1    1470 1650
	-1   0    0    1   
$EndComp
Text GLabel 1290 1650 0    60   Input ~ 0
VBUS
Wire Wire Line
	5815 2185 7125 2185
Wire Wire Line
	5815 2265 6905 2265
Wire Wire Line
	5815 2345 6685 2345
Wire Wire Line
	5815 2425 6465 2425
Wire Wire Line
	5815 3105 5925 3105
Wire Wire Line
	5925 3105 5925 3425
Wire Wire Line
	5815 3185 5925 3185
Connection ~ 5925 3185
Wire Wire Line
	5815 3265 5925 3265
Connection ~ 5925 3265
Wire Wire Line
	5815 3345 5925 3345
Connection ~ 5925 3345
Wire Wire Line
	6465 2625 6465 2645
Wire Wire Line
	6465 2645 7125 2645
Wire Wire Line
	6685 2545 6685 2645
Connection ~ 6685 2645
Wire Wire Line
	7125 2645 7125 2385
Wire Wire Line
	6905 2465 6905 2645
Connection ~ 6905 2645
Wire Wire Line
	6795 2665 6795 2645
Connection ~ 6795 2645
Wire Wire Line
	6005 2025 6005 2665
Connection ~ 6005 2185
Connection ~ 6005 2265
Connection ~ 6005 2345
Connection ~ 6005 2425
Wire Wire Line
	5815 2985 6185 2985
Wire Wire Line
	6185 2985 6185 3105
Wire Wire Line
	6185 3425 6185 3305
Wire Wire Line
	5815 2865 5925 2865
Wire Wire Line
	5815 2545 6005 2545
Connection ~ 6005 2545
Wire Wire Line
	10480 1155 10280 1155
Wire Wire Line
	10280 1155 10280 865 
Wire Wire Line
	10320 1455 10320 2355
Wire Wire Line
	10320 1455 10480 1455
Wire Wire Line
	10100 1355 10100 2355
Wire Wire Line
	10100 1355 10480 1355
Wire Wire Line
	10480 1255 9980 1255
Wire Wire Line
	9980 1255 9980 1325
Wire Wire Line
	8880 1265 8880 1445
Wire Wire Line
	8720 1345 9490 1345
Connection ~ 8880 1345
Wire Wire Line
	8330 1545 8330 1655
Wire Wire Line
	8880 1845 8880 1925
Wire Wire Line
	9180 1695 9580 1695
Wire Wire Line
	9215 1695 9215 1925
Wire Wire Line
	9215 2125 9215 2185
Wire Wire Line
	8880 2125 8880 2185
Wire Wire Line
	8880 1005 8880 1065
Wire Wire Line
	8330 1005 8330 1145
Wire Wire Line
	9490 1055 10480 1055
Wire Wire Line
	9490 1345 9490 1055
Connection ~ 9215 1695
Wire Wire Line
	9290 4260 9760 4260
Wire Wire Line
	9760 4260 9760 5060
Wire Wire Line
	9760 5000 9680 5000
Wire Wire Line
	9290 4500 9380 4500
Wire Wire Line
	9380 4500 9380 4620
Wire Wire Line
	9380 4820 9380 5060
Wire Wire Line
	9380 5000 9480 5000
Connection ~ 9760 5000
Connection ~ 9380 5000
Wire Wire Line
	9380 5260 9380 5320
Wire Wire Line
	9380 5320 9760 5320
Connection ~ 9580 5320
Wire Wire Line
	9760 5320 9760 5260
Wire Wire Line
	10380 5000 10300 5000
Wire Wire Line
	10000 5000 10100 5000
Wire Wire Line
	10000 5260 10000 5320
Wire Wire Line
	10000 5320 10380 5320
Connection ~ 10200 5320
Wire Wire Line
	10380 5320 10380 5260
Wire Wire Line
	9290 4340 10380 4340
Wire Wire Line
	10380 4340 10380 5060
Wire Wire Line
	9290 4420 10000 4420
Wire Wire Line
	10000 4420 10000 5060
Connection ~ 10000 5000
Connection ~ 10380 5000
Wire Wire Line
	2200 1660 2120 1660
Wire Wire Line
	2120 1660 2120 1540
Wire Wire Line
	2200 1780 2160 1780
Wire Wire Line
	2160 1780 2160 1860
Wire Wire Line
	2800 1540 2860 1540
Wire Wire Line
	3120 1750 3120 1820
Wire Wire Line
	2800 1780 3120 1780
Connection ~ 3120 1780
Wire Wire Line
	3120 2020 3120 2035
Wire Wire Line
	1880 1780 1880 1860
Wire Wire Line
	3420 1780 3420 1820
Wire Wire Line
	3420 1500 3420 1580
Connection ~ 3420 1540
Wire Wire Line
	3060 1540 3650 1540
Wire Wire Line
	3120 1550 3120 1540
Connection ~ 3120 1540
Wire Wire Line
	10320 2035 10405 2035
Wire Wire Line
	10100 2205 10405 2205
Connection ~ 10320 2035
Wire Wire Line
	10320 2635 10320 2675
Wire Wire Line
	10100 2675 10100 2635
Connection ~ 10100 2205
Wire Wire Line
	9490 2155 9490 2185
Wire Wire Line
	8405 1345 8520 1345
Wire Wire Line
	8080 1345 8460 1345
Connection ~ 8460 1345
Wire Wire Line
	1500 2975 1595 2975
Wire Wire Line
	1550 2700 1550 2975
Wire Wire Line
	1550 2885 1595 2885
Connection ~ 1550 2975
Wire Wire Line
	1550 2700 2290 2700
Wire Wire Line
	2290 2700 2290 2885
Wire Wire Line
	2290 2885 2255 2885
Connection ~ 1550 2885
Wire Wire Line
	2255 2975 2780 2975
Wire Wire Line
	2710 3015 2710 2975
Connection ~ 2710 2975
Wire Wire Line
	2980 2975 3560 2975
Wire Wire Line
	3210 2975 3210 3015
Wire Wire Line
	3210 3215 3210 3255
Wire Wire Line
	2710 3215 2710 3255
Wire Wire Line
	2255 3065 2450 3065
Wire Wire Line
	2450 3065 2450 3105
Wire Wire Line
	2450 3305 2450 3330
Wire Wire Line
	1100 3065 1595 3065
Wire Wire Line
	1300 2975 1300 3155
Connection ~ 1300 3065
Wire Wire Line
	1300 3395 1300 3355
Wire Wire Line
	2255 3155 2300 3155
Wire Wire Line
	2300 3155 2300 3225
Wire Wire Line
	3080 2975 3080 3545
Wire Wire Line
	3080 3545 2005 3545
Connection ~ 3080 2975
Wire Wire Line
	1595 3155 1545 3155
Wire Wire Line
	1545 3155 1545 3630
Wire Wire Line
	1545 3545 1805 3545
Wire Wire Line
	1545 3830 1545 3875
Connection ~ 1545 3545
Wire Wire Line
	3560 2975 3560 3045
Connection ~ 3210 2975
Wire Wire Line
	3880 2975 3760 2975
Wire Wire Line
	3760 2975 3760 3045
Wire Wire Line
	3660 2975 3660 3045
Wire Wire Line
	1290 1650 1370 1650
Wire Wire Line
	1370 1420 1290 1420
Wire Wire Line
	1290 1420 1290 1385
Wire Wire Line
	1570 1420 1630 1420
Wire Wire Line
	1630 1650 1570 1650
Wire Wire Line
	1630 1420 1630 1650
Connection ~ 1630 1540
$Comp
L R_Small R206
U 1 1 58F58D36
P 3650 1650
F 0 "R206" H 3685 1700 50  0000 L CNN
F 1 "680" H 3685 1610 50  0000 L CNN
F 2 "Resistors_SMD:R_0603" H 3650 1650 50  0001 C CNN
F 3 "" H 3650 1650 50  0001 C CNN
	1    3650 1650
	1    0    0    -1  
$EndComp
$Comp
L LED_Small_ALT D204
U 1 1 58F5905C
P 3650 1890
F 0 "D204" V 3655 1630 50  0000 L CNN
F 1 "3.3v LED Green" V 3545 1290 50  0000 L CNN
F 2 "LEDs:LED_0603" V 3650 1890 50  0001 C CNN
F 3 "" V 3650 1890 50  0001 C CNN
	1    3650 1890
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3650 1540 3650 1550
Wire Wire Line
	3650 1750 3650 1790
$Comp
L GND #PWR062
U 1 1 58F596CE
P 3650 2005
F 0 "#PWR062" H 3650 1755 50  0001 C CNN
F 1 "GND" H 3650 1855 50  0000 C CNN
F 2 "" H 3650 2005 50  0001 C CNN
F 3 "" H 3650 2005 50  0001 C CNN
	1    3650 2005
	1    0    0    -1  
$EndComp
Wire Wire Line
	3650 1990 3650 2005
$Comp
L Crystal_Small Y202
U 1 1 58F47E34
P 10200 5000
F 0 "Y202" H 10200 5180 50  0000 C CNN
F 1 "32.768kHz" H 10200 5100 50  0000 C CNN
F 2 "Crystals:Crystal_SMD_MicroCrystal_CC5V-T1A-2pin_4.1x1.5mm" H 10200 5000 50  0001 C CNN
F 3 "" H 10200 5000 50  0001 C CNN
	1    10200 5000
	1    0    0    -1  
$EndComp
Wire Wire Line
	10200 5360 10200 5320
$Comp
L Crystal_Small Y201
U 1 1 58F48773
P 9580 5000
F 0 "Y201" H 9580 5180 50  0000 C CNN
F 1 "20.0MHz" H 9580 5100 50  0000 C CNN
F 2 "Crystals:Crystal_SMD_0603-2pin_6.0x3.5mm" H 9580 5000 50  0001 C CNN
F 3 "" H 9580 5000 50  0001 C CNN
	1    9580 5000
	1    0    0    -1  
$EndComp
Wire Wire Line
	9580 5360 9580 5320
Wire Wire Line
	10605 2035 10690 2035
Wire Wire Line
	10605 2205 10690 2205
Wire Wire Line
	9490 1695 9490 1875
Connection ~ 9490 1695
Text Notes 8280 3080 0    60   ~ 0
To program through ICSP, X & Y axes must be disconnected.
Wire Wire Line
	5815 2745 6225 2745
Wire Wire Line
	6225 2745 6225 3070
Wire Wire Line
	6225 3070 6450 3070
Wire Wire Line
	6005 2665 5815 2665
Text GLabel 1975 1205 2    60   Output ~ 0
V_leds
Wire Wire Line
	1975 1205 1880 1205
Wire Wire Line
	1630 1540 2200 1540
Wire Wire Line
	1880 1205 1880 1580
Connection ~ 1880 1540
Connection ~ 2120 1540
$EndSCHEMATC
