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
Sheet 1 3
Title "KReal"
Date ""
Rev "0.5"
Comp "TiZed"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Text GLabel 925  3590 0    60   Output ~ 0
Fault_X
Text GLabel 1565 3450 2    60   Input ~ 0
STEP_X
Text GLabel 1565 3590 2    60   Input ~ 0
DIR_X
Text GLabel 1570 3730 2    60   Input ~ 0
ENABLE_X
$Comp
L GND #PWR01
U 1 1 58E6B3F4
P 935 3740
F 0 "#PWR01" H 935 3490 50  0001 C CNN
F 1 "GND" H 935 3590 50  0000 C CNN
F 2 "" H 935 3740 50  0001 C CNN
F 3 "" H 935 3740 50  0001 C CNN
	1    935  3740
	1    0    0    -1  
$EndComp
$Comp
L +3V3 #PWR02
U 1 1 58E6B669
P 935 3425
F 0 "#PWR02" H 935 3275 50  0001 C CNN
F 1 "+3V3" H 935 3565 50  0000 C CNN
F 2 "" H 935 3425 50  0001 C CNN
F 3 "" H 935 3425 50  0001 C CNN
	1    935  3425
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X03 P105
U 1 1 58E6CBC9
P 3080 3560
F 0 "P105" H 3080 3760 50  0000 C CNN
F 1 "X SW" V 3180 3560 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x03_Pitch2.54mm" H 3080 3560 50  0001 C CNN
F 3 "" H 3080 3560 50  0001 C CNN
	1    3080 3560
	1    0    0    -1  
$EndComp
Text GLabel 2680 3460 0    60   Output ~ 0
LIMIT_X
Text GLabel 2680 3660 0    60   Output ~ 0
HOME_X
$Comp
L GND #PWR03
U 1 1 58E6CDF6
P 2820 3740
F 0 "#PWR03" H 2820 3490 50  0001 C CNN
F 1 "GND" H 2820 3590 50  0000 C CNN
F 2 "" H 2820 3740 50  0001 C CNN
F 3 "" H 2820 3740 50  0001 C CNN
	1    2820 3740
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X03 P106
U 1 1 58E6D0AB
P 3080 4160
F 0 "P106" H 3080 4360 50  0000 C CNN
F 1 "Y SW" V 3180 4160 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x03_Pitch2.54mm" H 3080 4160 50  0001 C CNN
F 3 "" H 3080 4160 50  0001 C CNN
	1    3080 4160
	1    0    0    -1  
$EndComp
Text GLabel 2680 4060 0    60   Output ~ 0
LIMIT_Y
Text GLabel 2680 4260 0    60   Output ~ 0
HOME_Y
$Comp
L GND #PWR04
U 1 1 58E6D0B3
P 2820 4340
F 0 "#PWR04" H 2820 4090 50  0001 C CNN
F 1 "GND" H 2820 4190 50  0000 C CNN
F 2 "" H 2820 4340 50  0001 C CNN
F 3 "" H 2820 4340 50  0001 C CNN
	1    2820 4340
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X03 P107
U 1 1 58E6D522
P 3080 4760
F 0 "P107" H 3080 4960 50  0000 C CNN
F 1 "Z SW" V 3180 4760 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x03_Pitch2.54mm" H 3080 4760 50  0001 C CNN
F 3 "" H 3080 4760 50  0001 C CNN
	1    3080 4760
	1    0    0    -1  
$EndComp
Text GLabel 2680 4660 0    60   Output ~ 0
LIMIT_Z
Text GLabel 2680 4860 0    60   Output ~ 0
HOME_Z
$Comp
L GND #PWR05
U 1 1 58E6D52A
P 2820 4940
F 0 "#PWR05" H 2820 4690 50  0001 C CNN
F 1 "GND" H 2820 4790 50  0000 C CNN
F 2 "" H 2820 4940 50  0001 C CNN
F 3 "" H 2820 4940 50  0001 C CNN
	1    2820 4940
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X03 P109
U 1 1 58E6DCCC
P 3080 5980
F 0 "P109" H 3080 6180 50  0000 C CNN
F 1 "EMO Z SW" V 3180 5980 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x03_Pitch2.54mm" H 3080 5980 50  0001 C CNN
F 3 "" H 3080 5980 50  0001 C CNN
	1    3080 5980
	1    0    0    -1  
$EndComp
Text GLabel 2680 5880 0    60   Output ~ 0
EMO
Text GLabel 2680 6080 0    60   Output ~ 0
Z_LEVEL
$Comp
L GND #PWR06
U 1 1 58E6DCD4
P 2820 6160
F 0 "#PWR06" H 2820 5910 50  0001 C CNN
F 1 "GND" H 2820 6010 50  0000 C CNN
F 2 "" H 2820 6160 50  0001 C CNN
F 3 "" H 2820 6160 50  0001 C CNN
	1    2820 6160
	1    0    0    -1  
$EndComp
Text GLabel 2690 6710 0    60   Input ~ 0
PWM_S
Text GLabel 2690 6510 0    60   Input ~ 0
PWM_L
Wire Wire Line
	2680 3460 2880 3460
Wire Wire Line
	2680 3660 2880 3660
Wire Wire Line
	2820 3560 2820 3740
Wire Wire Line
	2820 3560 2880 3560
Wire Wire Line
	2680 4060 2880 4060
Wire Wire Line
	2680 4260 2880 4260
Wire Wire Line
	2820 4160 2820 4340
Wire Wire Line
	2820 4160 2880 4160
Wire Wire Line
	2680 4660 2880 4660
Wire Wire Line
	2680 4860 2880 4860
Wire Wire Line
	2820 4760 2820 4940
Wire Wire Line
	2820 4760 2880 4760
Wire Wire Line
	2680 5880 2880 5880
Wire Wire Line
	2680 6080 2880 6080
Wire Wire Line
	2820 5980 2820 6160
Wire Wire Line
	2820 5980 2880 5980
Wire Wire Line
	2880 6510 2690 6510
Wire Wire Line
	2880 6710 2690 6710
Text GLabel 10200 950  0    60   Output ~ 0
Reset
Text GLabel 10200 770  0    60   Output ~ 0
RX
Text GLabel 10860 770  2    60   Input ~ 0
TX
$Comp
L GND #PWR07
U 1 1 58E82CB8
P 10780 970
F 0 "#PWR07" H 10780 720 50  0001 C CNN
F 1 "GND" H 10780 820 50  0000 C CNN
F 2 "" H 10780 970 50  0001 C CNN
F 3 "" H 10780 970 50  0001 C CNN
	1    10780 970 
	1    0    0    -1  
$EndComp
$Comp
L CONN_02X03 P113
U 1 1 58E89FE9
P 10530 1570
F 0 "P113" H 10530 1770 50  0000 C CNN
F 1 "SPI" H 10530 1370 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_2x03_Pitch2.54mm" H 10530 370 50  0001 C CNN
F 3 "" H 10530 370 50  0001 C CNN
	1    10530 1570
	1    0    0    -1  
$EndComp
$Comp
L CONN_02X02 P112
U 1 1 58E8A0C8
P 10530 820
F 0 "P112" H 10530 970 50  0000 C CNN
F 1 "UART" H 10530 670 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_2x02_Pitch2.54mm" H 10530 -380 50  0001 C CNN
F 3 "" H 10530 -380 50  0001 C CNN
	1    10530 820 
	1    0    0    -1  
$EndComp
Wire Wire Line
	10780 770  10860 770 
Wire Wire Line
	10780 870  10780 970 
Wire Wire Line
	10280 870  10240 870 
Wire Wire Line
	10240 870  10240 950 
Wire Wire Line
	10240 950  10200 950 
Wire Wire Line
	10280 770  10200 770 
Text GLabel 10180 1390 0    60   Input ~ 0
MISO
Text GLabel 10880 1570 2    60   Output ~ 0
MOSI
Text GLabel 10180 1570 0    60   Output ~ 0
SCK
$Comp
L GND #PWR08
U 1 1 58E8BD44
P 10780 1760
F 0 "#PWR08" H 10780 1510 50  0001 C CNN
F 1 "GND" H 10780 1610 50  0000 C CNN
F 2 "" H 10780 1760 50  0001 C CNN
F 3 "" H 10780 1760 50  0001 C CNN
	1    10780 1760
	1    0    0    -1  
$EndComp
Text GLabel 10180 1750 0    60   Output ~ 0
SS
Wire Wire Line
	10880 1570 10780 1570
Wire Wire Line
	10780 1760 10780 1670
Wire Wire Line
	10280 1670 10240 1670
Wire Wire Line
	10240 1670 10240 1750
Wire Wire Line
	10240 1750 10180 1750
Wire Wire Line
	10280 1570 10180 1570
Wire Wire Line
	10280 1470 10240 1470
Wire Wire Line
	10240 1470 10240 1390
Wire Wire Line
	10240 1390 10180 1390
$Comp
L CONN_01X03 P110
U 1 1 58E8E1C9
P 3080 6610
F 0 "P110" H 3080 6810 50  0000 C CNN
F 1 "PWM" V 3180 6610 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x03_Pitch2.54mm" H 3080 6610 50  0001 C CNN
F 3 "" H 3080 6610 50  0001 C CNN
	1    3080 6610
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR09
U 1 1 58E8E4DD
P 2800 6810
F 0 "#PWR09" H 2800 6560 50  0001 C CNN
F 1 "GND" H 2800 6660 50  0000 C CNN
F 2 "" H 2800 6810 50  0001 C CNN
F 3 "" H 2800 6810 50  0001 C CNN
	1    2800 6810
	1    0    0    -1  
$EndComp
Wire Wire Line
	2880 6610 2800 6610
Wire Wire Line
	2800 6610 2800 6810
$Comp
L USB_OTG J101
U 1 1 58E8E987
P 9060 1200
F 0 "J101" H 8860 1650 50  0000 L CNN
F 1 "USB" H 8860 1550 50  0000 L CNN
F 2 "Connect:USB_Mini-B" H 9210 1150 50  0001 C CNN
F 3 "" H 9210 1150 50  0001 C CNN
	1    9060 1200
	-1   0    0    -1  
$EndComp
$Comp
L GND #PWR010
U 1 1 58E8EF95
P 9060 1700
F 0 "#PWR010" H 9060 1450 50  0001 C CNN
F 1 "GND" H 9060 1550 50  0000 C CNN
F 2 "" H 9060 1700 50  0001 C CNN
F 3 "" H 9060 1700 50  0001 C CNN
	1    9060 1700
	1    0    0    -1  
$EndComp
Wire Wire Line
	9060 1600 9060 1700
Wire Wire Line
	9160 1600 9160 1660
Wire Wire Line
	9160 1660 9060 1660
Connection ~ 9060 1660
Text GLabel 8620 1000 0    60   Output ~ 0
VBUS
Text GLabel 8620 1180 0    60   BiDi ~ 0
D_P
Text GLabel 8620 1320 0    60   BiDi ~ 0
D_N
Text GLabel 8620 1500 0    60   Output ~ 0
USB_ID
Wire Wire Line
	8620 1180 8700 1180
Wire Wire Line
	8700 1180 8700 1200
Wire Wire Line
	8700 1200 8760 1200
Wire Wire Line
	8620 1320 8700 1320
Wire Wire Line
	8700 1320 8700 1300
Wire Wire Line
	8700 1300 8760 1300
Wire Wire Line
	8620 1000 8760 1000
Wire Wire Line
	8620 1500 8700 1500
Wire Wire Line
	8700 1500 8700 1400
Wire Wire Line
	8700 1400 8760 1400
$Comp
L PIC32MX775F U101
U 6 1 58EF8AE9
P 9230 2600
F 0 "U101" H 8380 2300 55  0000 C CNN
F 1 "PIC32MX775F" H 8620 2910 63  0000 C CNN
F 2 "Housings_QFP:TQFP-64_10x10mm_Pitch0.5mm" H 9940 2470 60  0001 C CNN
F 3 "" H 9940 2470 60  0001 C CNN
	6    9230 2600
	1    0    0    -1  
$EndComp
Text GLabel 10560 2520 2    60   Input ~ 0
USB_ID
Text GLabel 10570 2840 2    60   Output ~ 0
TX
Text GLabel 10560 2680 2    60   Input ~ 0
RX
Wire Wire Line
	10170 2680 10560 2680
Wire Wire Line
	10170 2760 10420 2760
Wire Wire Line
	10420 2760 10420 2840
Wire Wire Line
	10420 2840 10570 2840
Wire Wire Line
	10170 2600 10420 2600
Wire Wire Line
	10420 2600 10420 2520
Wire Wire Line
	10420 2520 10560 2520
$Comp
L PIC32MX775F U101
U 7 1 58EFC3B4
P 9390 3480
F 0 "U101" H 8680 3140 55  0000 C CNN
F 1 "PIC32MX775F" H 8910 3830 63  0000 C CNN
F 2 "Housings_QFP:TQFP-64_10x10mm_Pitch0.5mm" H 10100 3350 60  0001 C CNN
F 3 "" H 10100 3350 60  0001 C CNN
	7    9390 3480
	1    0    0    -1  
$EndComp
Text GLabel 10580 3280 2    60   BiDi ~ 0
D_N
Text GLabel 10580 3120 2    60   BiDi ~ 0
D_P
Text GLabel 10580 3760 2    60   Output ~ 0
MISO
Text GLabel 10580 3440 2    60   Input ~ 0
SCK
Text GLabel 10580 3920 2    60   Input ~ 0
SS
Text GLabel 10580 3600 2    60   Input ~ 0
MOSI
Wire Wire Line
	10580 3440 10170 3440
Wire Wire Line
	10580 3280 10500 3280
Wire Wire Line
	10500 3280 10500 3360
Wire Wire Line
	10500 3360 10170 3360
Wire Wire Line
	10580 3120 10440 3120
Wire Wire Line
	10440 3120 10440 3280
Wire Wire Line
	10440 3280 10170 3280
Wire Wire Line
	10580 3600 10500 3600
Wire Wire Line
	10500 3600 10500 3520
Wire Wire Line
	10500 3520 10170 3520
Wire Wire Line
	10580 3760 10440 3760
Wire Wire Line
	10440 3760 10440 3600
Wire Wire Line
	10440 3600 10170 3600
Wire Wire Line
	10580 3920 10380 3920
Wire Wire Line
	10380 3920 10380 3680
Wire Wire Line
	10380 3680 10170 3680
Text GLabel 3860 1440 2    60   Output ~ 0
STEP_X
Text GLabel 3860 1580 2    60   Output ~ 0
DIR_X
Text GLabel 3860 1720 2    60   Output ~ 0
ENABLE_X
Text GLabel 3860 1900 2    60   Output ~ 0
STEP_Y
Text GLabel 3860 2040 2    60   Output ~ 0
DIR_Y
Text GLabel 3860 2180 2    60   Output ~ 0
ENABLE_Y
Text GLabel 3860 2360 2    60   Output ~ 0
STEP_Z
Text GLabel 3860 2500 2    60   Output ~ 0
DIR_Z
Text GLabel 3860 2640 2    60   Output ~ 0
ENABLE_Z
$Comp
L PIC32MX775F U101
U 2 1 58F0CE09
P 1830 2050
F 0 "U101" H 770 1300 55  0000 C CNN
F 1 "PIC32MX775F" H 1030 2800 63  0000 C CNN
F 2 "Housings_QFP:TQFP-64_10x10mm_Pitch0.5mm" H 2540 1920 60  0001 C CNN
F 3 "" H 2540 1920 60  0001 C CNN
	2    1830 2050
	1    0    0    -1  
$EndComp
$Sheet
S 760  720  1770 300 
U 58F14E7A
F0 "Power and Reset" 60
F1 "kreal_power_reset.sch" 60
$EndSheet
$Sheet
S 5300 690  680  1360
U 58F1A0F2
F0 "Optocouplers" 60
F1 "kreal_optocouplers.sch" 60
F2 "Emo_oc" O R 5980 760 60 
F3 "Z_level_oc" O R 5980 840 60 
F4 "Limit_z_oc" O R 5980 1400 60 
F5 "Home_z_oc" O R 5980 1480 60 
F6 "Limit_x_oc" O R 5980 1000 60 
F7 "Home_x_oc" O R 5980 1080 60 
F8 "Limit_y_oc" O R 5980 1200 60 
F9 "Home_y_oc" O R 5980 1280 60 
F10 "Pwm_s_oc" I R 5980 1840 60 
F11 "Pwm_l_oc" I R 5980 1920 60 
F12 "Limit_a_oc" O R 5980 1600 60 
F13 "Home_a_oc" O R 5980 1680 60 
$EndSheet
Text Label 7430 2440 0    60   ~ 0
Emo_oc
$Comp
L PIC32MX775F U?
U 4 1 58F5D59E
P 5990 2880
AR Path="/58F1A0F2/58F5D59E" Ref="U?"  Part="4" 
AR Path="/58F5D59E" Ref="U101"  Part="4" 
F 0 "U101" H 4900 2290 55  0000 C CNN
F 1 "PIC32MX775F" H 5150 3460 63  0000 C CNN
F 2 "Housings_QFP:TQFP-64_10x10mm_Pitch0.5mm" H 6700 2750 60  0001 C CNN
F 3 "" H 6700 2750 60  0001 C CNN
	4    5990 2880
	1    0    0    -1  
$EndComp
Text Label 7430 2520 0    60   ~ 0
Pwm_s_oc
Text Label 7430 2600 0    60   ~ 0
Pwm_l_oc
Text Label 7430 2760 0    60   ~ 0
Limit_x_oc
Text Label 7430 2840 0    60   ~ 0
Home_x_oc
Text Label 7430 2920 0    60   ~ 0
Limit_y_oc
Text Label 7430 3000 0    60   ~ 0
Home_y_oc
Text Label 7430 3080 0    60   ~ 0
Z_level_oc
Wire Wire Line
	7170 2440 7430 2440
Wire Wire Line
	7170 2520 7430 2520
Wire Wire Line
	7170 2600 7430 2600
Wire Wire Line
	7170 2760 7430 2760
Wire Wire Line
	7170 2840 7430 2840
Wire Wire Line
	7170 2920 7430 2920
Wire Wire Line
	7170 3000 7430 3000
Wire Wire Line
	7170 3080 7430 3080
Text Label 3860 1080 0    60   ~ 0
Limit_z_oc
Text Label 3860 1160 0    60   ~ 0
Home_z_oc
Wire Wire Line
	3220 1080 3860 1080
Wire Wire Line
	3280 1160 3860 1160
Wire Wire Line
	2970 1530 3280 1530
Wire Wire Line
	3280 1530 3280 1160
Wire Wire Line
	2970 1450 3220 1450
Wire Wire Line
	3220 1450 3220 1080
Text Label 6360 760  0    60   ~ 0
Emo_oc
Text Label 6360 1840 0    60   ~ 0
Pwm_s_oc
Text Label 6360 1920 0    60   ~ 0
Pwm_l_oc
Text Label 6360 1000 0    60   ~ 0
Limit_x_oc
Text Label 6360 1080 0    60   ~ 0
Home_x_oc
Text Label 6360 1200 0    60   ~ 0
Limit_y_oc
Text Label 6360 1280 0    60   ~ 0
Home_y_oc
Text Label 6360 840  0    60   ~ 0
Z_level_oc
Wire Wire Line
	5980 760  6360 760 
Wire Wire Line
	6360 840  5980 840 
Wire Wire Line
	5980 1000 6360 1000
Wire Wire Line
	6360 1080 5980 1080
Wire Wire Line
	5980 1200 6360 1200
Wire Wire Line
	6360 1280 5980 1280
Wire Wire Line
	6360 1840 5980 1840
Wire Wire Line
	6360 1920 5980 1920
Text Label 6360 1400 0    60   ~ 0
Limit_z_oc
Text Label 6360 1480 0    60   ~ 0
Home_z_oc
Wire Wire Line
	5980 1400 6360 1400
Wire Wire Line
	6360 1480 5980 1480
$Comp
L CD4048B U102
U 1 1 58F0E624
P 4700 4740
F 0 "U102" H 4480 4200 60  0000 C CNN
F 1 "CD4048B" H 4400 5290 60  0000 C CNN
F 2 "Housings_SSOP:TSSOP-16_4.4x5mm_Pitch0.65mm" H 4700 5080 60  0001 C CNN
F 3 "" H 4700 5080 60  0001 C CNN
	1    4700 4740
	1    0    0    -1  
$EndComp
$Comp
L +3V3 #PWR011
U 1 1 58F0E77B
P 4700 4000
F 0 "#PWR011" H 4700 3850 50  0001 C CNN
F 1 "+3V3" H 4700 4140 50  0000 C CNN
F 2 "" H 4700 4000 50  0001 C CNN
F 3 "" H 4700 4000 50  0001 C CNN
	1    4700 4000
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR012
U 1 1 58F103A6
P 4300 5220
F 0 "#PWR012" H 4300 4970 50  0001 C CNN
F 1 "GND" H 4300 5070 50  0000 C CNN
F 2 "" H 4300 5220 50  0001 C CNN
F 3 "" H 4300 5220 50  0001 C CNN
	1    4300 5220
	1    0    0    -1  
$EndComp
Wire Wire Line
	4340 5060 4300 5060
Wire Wire Line
	4300 5060 4300 5220
Wire Wire Line
	4340 5160 4300 5160
Connection ~ 4300 5160
Text Label 5200 4340 0    60   ~ 0
Switches_int
Wire Wire Line
	5060 4340 5200 4340
$Comp
L C_Small C101
U 1 1 58F10FBA
P 4840 4050
F 0 "C101" V 4950 3970 50  0000 L CNN
F 1 "100n" V 4740 3970 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 4840 4050 50  0001 C CNN
F 3 "" H 4840 4050 50  0001 C CNN
	1    4840 4050
	0    1    1    0   
$EndComp
$Comp
L GND #PWR013
U 1 1 58F11605
P 5010 4050
F 0 "#PWR013" H 5010 3800 50  0001 C CNN
F 1 "GND" H 5010 3900 50  0000 C CNN
F 2 "" H 5010 4050 50  0001 C CNN
F 3 "" H 5010 4050 50  0001 C CNN
	1    5010 4050
	1    0    0    -1  
$EndComp
Wire Wire Line
	4700 4000 4700 4160
Wire Wire Line
	4740 4050 4700 4050
Connection ~ 4700 4050
Wire Wire Line
	4940 4050 5010 4050
Text Label 4140 4420 2    60   ~ 0
Limit_x_oc
Text Label 4140 4340 2    60   ~ 0
Home_x_oc
Text Label 4140 4580 2    60   ~ 0
Limit_y_oc
Text Label 4140 4500 2    60   ~ 0
Home_y_oc
Text Label 4140 4740 2    60   ~ 0
Limit_z_oc
Text Label 4140 4660 2    60   ~ 0
Home_z_oc
$Comp
L CONN_01X03 P108
U 1 1 58F1A9DE
P 3080 5360
F 0 "P108" H 3080 5560 50  0000 C CNN
F 1 "A SW" V 3180 5360 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x03_Pitch2.54mm" H 3080 5360 50  0001 C CNN
F 3 "" H 3080 5360 50  0001 C CNN
	1    3080 5360
	1    0    0    -1  
$EndComp
Text GLabel 2680 5260 0    60   Output ~ 0
LIMIT_A
Text GLabel 2680 5460 0    60   Output ~ 0
HOME_A
$Comp
L GND #PWR014
U 1 1 58F1A9E6
P 2820 5540
F 0 "#PWR014" H 2820 5290 50  0001 C CNN
F 1 "GND" H 2820 5390 50  0000 C CNN
F 2 "" H 2820 5540 50  0001 C CNN
F 3 "" H 2820 5540 50  0001 C CNN
	1    2820 5540
	1    0    0    -1  
$EndComp
Wire Wire Line
	2680 5260 2880 5260
Wire Wire Line
	2680 5460 2880 5460
Wire Wire Line
	2820 5360 2820 5540
Wire Wire Line
	2820 5360 2880 5360
Text GLabel 3860 2820 2    60   Output ~ 0
STEP_A
Text GLabel 3860 2960 2    60   Output ~ 0
DIR_A
Text GLabel 3860 3100 2    60   Output ~ 0
ENABLE_A
Text Label 3860 1240 0    60   ~ 0
Limit_a_oc
Text Label 3860 1320 0    60   ~ 0
Home_a_oc
Wire Wire Line
	2970 1610 3340 1610
Wire Wire Line
	3340 1610 3340 1240
Wire Wire Line
	3340 1240 3860 1240
Wire Wire Line
	2970 1690 3400 1690
Wire Wire Line
	3400 1690 3400 1320
Wire Wire Line
	3400 1320 3860 1320
Wire Wire Line
	2970 1770 3460 1770
Wire Wire Line
	3460 1770 3460 1440
Wire Wire Line
	3460 1440 3860 1440
Wire Wire Line
	2970 1850 3520 1850
Wire Wire Line
	3520 1850 3520 1580
Wire Wire Line
	3520 1580 3860 1580
Wire Wire Line
	2970 1930 3580 1930
Wire Wire Line
	3580 1930 3580 1720
Wire Wire Line
	3580 1720 3860 1720
Wire Wire Line
	2970 2010 3640 2010
Wire Wire Line
	3640 2010 3640 1900
Wire Wire Line
	3640 1900 3860 1900
Wire Wire Line
	2970 2090 3700 2090
Wire Wire Line
	3700 2090 3700 2040
Wire Wire Line
	3700 2040 3860 2040
Wire Wire Line
	2970 2170 3700 2170
Wire Wire Line
	3700 2170 3700 2180
Wire Wire Line
	3700 2180 3860 2180
Wire Wire Line
	2970 2250 3640 2250
Wire Wire Line
	3640 2250 3640 2360
Wire Wire Line
	3640 2360 3860 2360
Wire Wire Line
	2970 2330 3580 2330
Wire Wire Line
	3580 2330 3580 2500
Wire Wire Line
	3580 2500 3860 2500
Wire Wire Line
	2970 2410 3520 2410
Wire Wire Line
	3520 2410 3520 2640
Wire Wire Line
	3520 2640 3860 2640
Wire Wire Line
	2970 2490 3460 2490
Wire Wire Line
	3460 2490 3460 2820
Wire Wire Line
	3460 2820 3860 2820
Wire Wire Line
	2970 2570 3400 2570
Wire Wire Line
	3400 2570 3400 2960
Wire Wire Line
	3400 2960 3860 2960
Wire Wire Line
	2970 2650 3340 2650
Wire Wire Line
	3340 2650 3340 3100
Wire Wire Line
	3340 3100 3860 3100
Text Label 4140 4900 2    60   ~ 0
Limit_a_oc
Text Label 4140 4820 2    60   ~ 0
Home_a_oc
Wire Wire Line
	4140 4340 4340 4340
Wire Wire Line
	4140 4420 4340 4420
Wire Wire Line
	4140 4500 4340 4500
Wire Wire Line
	4140 4580 4340 4580
Wire Wire Line
	4140 4660 4340 4660
Wire Wire Line
	4140 4740 4340 4740
Wire Wire Line
	4140 4820 4340 4820
Wire Wire Line
	4140 4900 4340 4900
Text Label 7430 3240 0    60   ~ 0
Switches_int
Wire Wire Line
	7170 3160 7430 3160
Text Label 6360 1600 0    60   ~ 0
Limit_a_oc
Text Label 6360 1680 0    60   ~ 0
Home_a_oc
Wire Wire Line
	5980 1600 6360 1600
Wire Wire Line
	5980 1680 6360 1680
$Comp
L PIC32MX775F U101
U 5 1 58F28F29
P 6850 4450
F 0 "U101" H 6270 4010 55  0000 C CNN
F 1 "PIC32MX775F" H 6490 4890 63  0000 C CNN
F 2 "Housings_QFP:TQFP-64_10x10mm_Pitch0.5mm" H 7560 4320 60  0001 C CNN
F 3 "" H 7560 4320 60  0001 C CNN
	5    6850 4450
	1    0    0    -1  
$EndComp
Text GLabel 7830 3990 2    60   Input ~ 0
Fault_X
Text GLabel 7840 4130 2    60   Input ~ 0
Fault_Y
Text GLabel 7850 4270 2    60   Input ~ 0
Fault_Z
Text GLabel 7850 4410 2    60   Input ~ 0
Fault_A
Wire Wire Line
	7510 4410 7850 4410
Wire Wire Line
	7510 4330 7790 4330
Wire Wire Line
	7790 4330 7790 4270
Wire Wire Line
	7790 4270 7850 4270
Wire Wire Line
	7510 4250 7730 4250
Wire Wire Line
	7730 4250 7730 4130
Wire Wire Line
	7730 4130 7840 4130
Wire Wire Line
	7510 4170 7670 4170
Wire Wire Line
	7670 4170 7670 3990
Wire Wire Line
	7670 3990 7830 3990
$Comp
L CD4048B U103
U 1 1 58F19CEE
P 4700 6380
F 0 "U103" H 4480 5840 60  0000 C CNN
F 1 "CD4048B" H 4400 6930 60  0000 C CNN
F 2 "Housings_SSOP:TSSOP-16_4.4x5mm_Pitch0.65mm" H 4700 6720 60  0001 C CNN
F 3 "" H 4700 6720 60  0001 C CNN
	1    4700 6380
	1    0    0    -1  
$EndComp
$Comp
L +3V3 #PWR015
U 1 1 58F19CF4
P 6625 5535
F 0 "#PWR015" H 6625 5385 50  0001 C CNN
F 1 "+3V3" H 6625 5675 50  0000 C CNN
F 2 "" H 6625 5535 50  0001 C CNN
F 3 "" H 6625 5535 50  0001 C CNN
	1    6625 5535
	1    0    0    -1  
$EndComp
Text Label 5200 5980 0    60   ~ 0
Faults_int
Wire Wire Line
	5060 5980 5200 5980
$Comp
L C_Small C102
U 1 1 58F19CFC
P 4840 5690
F 0 "C102" V 4950 5610 50  0000 L CNN
F 1 "100n" V 4740 5610 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 4840 5690 50  0001 C CNN
F 3 "" H 4840 5690 50  0001 C CNN
	1    4840 5690
	0    1    1    0   
$EndComp
$Comp
L GND #PWR016
U 1 1 58F19D02
P 5010 5690
F 0 "#PWR016" H 5010 5440 50  0001 C CNN
F 1 "GND" H 5010 5540 50  0000 C CNN
F 2 "" H 5010 5690 50  0001 C CNN
F 3 "" H 5010 5690 50  0001 C CNN
	1    5010 5690
	1    0    0    -1  
$EndComp
Wire Wire Line
	4700 5640 4700 5800
Wire Wire Line
	4740 5690 4700 5690
Connection ~ 4700 5690
Wire Wire Line
	4940 5690 5010 5690
Text GLabel 4060 5900 0    60   Input ~ 0
Fault_X
Text GLabel 4060 6040 0    60   Input ~ 0
Fault_Y
Text GLabel 4060 6180 0    60   Input ~ 0
Fault_Z
Text GLabel 4060 6320 0    60   Input ~ 0
Fault_A
Wire Wire Line
	4340 5980 4200 5980
Wire Wire Line
	4200 5980 4200 5900
Wire Wire Line
	4200 5900 4060 5900
Wire Wire Line
	4340 6060 4180 6060
Wire Wire Line
	4180 6060 4180 6040
Wire Wire Line
	4180 6040 4060 6040
Wire Wire Line
	4340 6140 4180 6140
Wire Wire Line
	4180 6140 4180 6180
Wire Wire Line
	4180 6180 4060 6180
Wire Wire Line
	4340 6220 4200 6220
Wire Wire Line
	4200 6220 4200 6320
Wire Wire Line
	4200 6320 4060 6320
$Comp
L GND #PWR017
U 1 1 58F1EA4F
P 4300 6860
F 0 "#PWR017" H 4300 6610 50  0001 C CNN
F 1 "GND" H 4300 6710 50  0000 C CNN
F 2 "" H 4300 6860 50  0001 C CNN
F 3 "" H 4300 6860 50  0001 C CNN
	1    4300 6860
	1    0    0    -1  
$EndComp
Wire Wire Line
	4340 6300 4300 6300
Wire Wire Line
	4300 6300 4300 6860
Wire Wire Line
	4340 6380 4300 6380
Connection ~ 4300 6380
Wire Wire Line
	4340 6460 4300 6460
Connection ~ 4300 6460
Wire Wire Line
	4340 6540 4300 6540
Connection ~ 4300 6540
Wire Wire Line
	4340 6700 4300 6700
Connection ~ 4300 6700
Wire Wire Line
	4340 6800 4300 6800
Connection ~ 4300 6800
Text Label 7430 3160 0    60   ~ 0
Faults_int
Wire Wire Line
	7170 3240 7430 3240
Text Label 5260 4840 0    60   ~ 0
Logic_Ka
Text Label 5260 4920 0    60   ~ 0
Logic_Kb
Text Label 5260 5000 0    60   ~ 0
Logic_Kc
Wire Wire Line
	5060 4840 5260 4840
Wire Wire Line
	5060 4920 5260 4920
Wire Wire Line
	5060 5000 5260 5000
Text Label 5260 5160 0    60   ~ 0
Logic_En
Wire Wire Line
	5060 5160 5260 5160
Text Label 5260 6480 0    60   ~ 0
Logic_Ka
Text Label 5260 6560 0    60   ~ 0
Logic_Kb
Text Label 5260 6640 0    60   ~ 0
Logic_Kc
Wire Wire Line
	5060 6480 5260 6480
Wire Wire Line
	5060 6560 5260 6560
Wire Wire Line
	5060 6640 5260 6640
Text Label 5260 6800 0    60   ~ 0
Logic_En
Wire Wire Line
	5060 6800 5260 6800
Text Label 7850 4570 0    60   ~ 0
Logic_Ka
Text Label 7850 4650 0    60   ~ 0
Logic_Kb
Text Label 7850 4730 0    60   ~ 0
Logic_Kc
Text Label 7850 4810 0    60   ~ 0
Logic_En
Wire Wire Line
	7510 4490 7770 4490
Wire Wire Line
	7770 4490 7770 4570
Wire Wire Line
	7770 4570 7850 4570
Wire Wire Line
	7510 4570 7710 4570
Wire Wire Line
	7710 4570 7710 4650
Wire Wire Line
	7710 4650 7850 4650
Wire Wire Line
	7510 4650 7650 4650
Wire Wire Line
	7650 4650 7650 4730
Wire Wire Line
	7650 4730 7850 4730
Wire Wire Line
	7510 4730 7590 4730
Wire Wire Line
	7590 4730 7590 4810
Wire Wire Line
	7590 4810 7850 4810
$Comp
L Mount_hole J104
U 1 1 58F3533D
P 10660 4580
F 0 "J104" H 10660 4455 39  0000 C CNN
F 1 "3mm" V 10760 4580 39  0000 C CNN
F 2 "Mounting_Holes:MountingHole_3mm_Pad" H 10660 4580 60  0001 C CNN
F 3 "" H 10660 4580 60  0000 C CNN
	1    10660 4580
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR018
U 1 1 58F357F6
P 10660 4720
F 0 "#PWR018" H 10660 4470 50  0001 C CNN
F 1 "GND" H 10660 4570 50  0000 C CNN
F 2 "" H 10660 4720 50  0001 C CNN
F 3 "" H 10660 4720 50  0001 C CNN
	1    10660 4720
	1    0    0    -1  
$EndComp
Wire Wire Line
	10660 4680 10660 4720
$Comp
L Mount_hole J102
U 1 1 58F35D14
P 10390 4580
F 0 "J102" H 10390 4455 39  0000 C CNN
F 1 "3mm" V 10490 4580 39  0000 C CNN
F 2 "Mounting_Holes:MountingHole_3mm_Pad" H 10390 4580 60  0001 C CNN
F 3 "" H 10390 4580 60  0000 C CNN
	1    10390 4580
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR019
U 1 1 58F35D1A
P 10390 4720
F 0 "#PWR019" H 10390 4470 50  0001 C CNN
F 1 "GND" H 10390 4570 50  0000 C CNN
F 2 "" H 10390 4720 50  0001 C CNN
F 3 "" H 10390 4720 50  0001 C CNN
	1    10390 4720
	1    0    0    -1  
$EndComp
Wire Wire Line
	10390 4680 10390 4720
$Comp
L Mount_hole J105
U 1 1 58F35FFF
P 10660 5080
F 0 "J105" H 10660 4955 39  0000 C CNN
F 1 "3mm" V 10760 5080 39  0000 C CNN
F 2 "Mounting_Holes:MountingHole_3mm_Pad" H 10660 5080 60  0001 C CNN
F 3 "" H 10660 5080 60  0000 C CNN
	1    10660 5080
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR020
U 1 1 58F36005
P 10660 5220
F 0 "#PWR020" H 10660 4970 50  0001 C CNN
F 1 "GND" H 10660 5070 50  0000 C CNN
F 2 "" H 10660 5220 50  0001 C CNN
F 3 "" H 10660 5220 50  0001 C CNN
	1    10660 5220
	1    0    0    -1  
$EndComp
Wire Wire Line
	10660 5180 10660 5220
$Comp
L Mount_hole J103
U 1 1 58F3600C
P 10390 5080
F 0 "J103" H 10390 4955 39  0000 C CNN
F 1 "3mm" V 10490 5080 39  0000 C CNN
F 2 "Mounting_Holes:MountingHole_3mm_Pad" H 10390 5080 60  0001 C CNN
F 3 "" H 10390 5080 60  0000 C CNN
	1    10390 5080
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR021
U 1 1 58F36012
P 10390 5220
F 0 "#PWR021" H 10390 4970 50  0001 C CNN
F 1 "GND" H 10390 5070 50  0000 C CNN
F 2 "" H 10390 5220 50  0001 C CNN
F 3 "" H 10390 5220 50  0001 C CNN
	1    10390 5220
	1    0    0    -1  
$EndComp
Wire Wire Line
	10390 5180 10390 5220
Wire Notes Line
	10190 4400 10900 4400
Wire Notes Line
	10900 4400 10900 5500
Wire Notes Line
	10900 5500 10190 5500
Wire Notes Line
	10190 5500 10190 4400
Text Notes 10200 5490 0    39   ~ 0
Mount Holes
$Comp
L CONN_02X05 P111
U 1 1 58F46CD2
P 7505 5695
F 0 "P111" H 7505 5995 50  0000 C CNN
F 1 "Power" H 7505 5395 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_2x05_Pitch2.54mm" H 7505 4495 50  0001 C CNN
F 3 "" H 7505 4495 50  0001 C CNN
	1    7505 5695
	1    0    0    -1  
$EndComp
Text GLabel 7135 5495 0    60   Output ~ 0
VIN
$Comp
L +5V #PWR022
U 1 1 58F48423
P 6835 5535
F 0 "#PWR022" H 6835 5385 50  0001 C CNN
F 1 "+5V" H 6835 5675 50  0000 C CNN
F 2 "" H 6835 5535 50  0001 C CNN
F 3 "" H 6835 5535 50  0001 C CNN
	1    6835 5535
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR023
U 1 1 58F49FC7
P 7875 5945
F 0 "#PWR023" H 7875 5695 50  0001 C CNN
F 1 "GND" H 7875 5795 50  0000 C CNN
F 2 "" H 7875 5945 50  0001 C CNN
F 3 "" H 7875 5945 50  0001 C CNN
	1    7875 5945
	1    0    0    -1  
$EndComp
Wire Wire Line
	7875 5495 7875 5945
Wire Wire Line
	7875 5495 7755 5495
Wire Wire Line
	7755 5595 7875 5595
Connection ~ 7875 5595
Wire Wire Line
	7755 5695 7875 5695
Connection ~ 7875 5695
Wire Wire Line
	7755 5795 7875 5795
Connection ~ 7875 5795
Wire Wire Line
	7755 5895 7875 5895
Connection ~ 7875 5895
Wire Wire Line
	7135 5495 7255 5495
Wire Wire Line
	6835 5535 6835 5595
Wire Wire Line
	6835 5595 7255 5595
Wire Wire Line
	7255 5695 7205 5695
Wire Wire Line
	7205 5695 7205 5595
Connection ~ 7205 5595
Wire Wire Line
	6625 5535 6625 5795
Wire Wire Line
	6625 5795 7255 5795
Wire Wire Line
	7255 5895 7205 5895
Wire Wire Line
	7205 5895 7205 5795
Connection ~ 7205 5795
$Comp
L R_Small R102
U 1 1 58F5C004
P 9620 4530
F 0 "R102" H 9655 4575 50  0000 L CNN
F 1 "470" H 9655 4490 50  0000 L CNN
F 2 "Resistors_SMD:R_0603" H 9620 4530 50  0001 C CNN
F 3 "" H 9620 4530 50  0001 C CNN
	1    9620 4530
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR024
U 1 1 58F5C013
P 9620 6195
F 0 "#PWR024" H 9620 5945 50  0001 C CNN
F 1 "GND" H 9620 6045 50  0000 C CNN
F 2 "" H 9620 6195 50  0001 C CNN
F 3 "" H 9620 6195 50  0001 C CNN
	1    9620 6195
	1    0    0    -1  
$EndComp
$Comp
L R_Small R101
U 1 1 58F5C4B2
P 9420 4530
F 0 "R101" H 9190 4575 50  0000 L CNN
F 1 "470" H 9240 4490 50  0000 L CNN
F 2 "Resistors_SMD:R_0603" H 9420 4530 50  0001 C CNN
F 3 "" H 9420 4530 50  0001 C CNN
	1    9420 4530
	1    0    0    -1  
$EndComp
Text Label 8800 5510 2    60   ~ 0
Led_b
Text Label 9000 5955 2    60   ~ 0
Led_a
Wire Wire Line
	9620 4285 9620 4430
Wire Wire Line
	9420 4285 9420 4430
Text Label 10600 2370 0    60   ~ 0
Led_b
Text Label 10600 2260 0    60   ~ 0
Led_a
Wire Wire Line
	10170 2520 10390 2520
Wire Wire Line
	10390 2520 10390 2370
Wire Wire Line
	10390 2370 10600 2370
Wire Wire Line
	10170 2440 10360 2440
Wire Wire Line
	10360 2440 10360 2260
Wire Wire Line
	10360 2260 10600 2260
$Comp
L CONN_02X03 P101
U 1 1 58F3979C
P 1245 3590
F 0 "P101" H 1245 3790 50  0000 C CNN
F 1 "X AXIS" H 1245 3390 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_2x03_Pitch2.54mm" H 1245 2390 50  0001 C CNN
F 3 "" H 1245 2390 50  0001 C CNN
	1    1245 3590
	1    0    0    -1  
$EndComp
Wire Wire Line
	1565 3590 1495 3590
Wire Wire Line
	1565 3450 1495 3450
Wire Wire Line
	1495 3450 1495 3490
Wire Wire Line
	1570 3730 1495 3730
Wire Wire Line
	1495 3730 1495 3690
Wire Wire Line
	935  3740 935  3690
Wire Wire Line
	935  3690 995  3690
Wire Wire Line
	935  3425 935  3490
Wire Wire Line
	935  3490 995  3490
Wire Wire Line
	925  3590 995  3590
Text GLabel 925  4395 0    60   Output ~ 0
Fault_Y
Text GLabel 1565 4255 2    60   Input ~ 0
STEP_Y
Text GLabel 1565 4395 2    60   Input ~ 0
DIR_Y
Text GLabel 1570 4535 2    60   Input ~ 0
ENABLE_Y
$Comp
L GND #PWR025
U 1 1 58F3F407
P 935 4545
F 0 "#PWR025" H 935 4295 50  0001 C CNN
F 1 "GND" H 935 4395 50  0000 C CNN
F 2 "" H 935 4545 50  0001 C CNN
F 3 "" H 935 4545 50  0001 C CNN
	1    935  4545
	1    0    0    -1  
$EndComp
$Comp
L +3V3 #PWR026
U 1 1 58F3F40D
P 935 4230
F 0 "#PWR026" H 935 4080 50  0001 C CNN
F 1 "+3V3" H 935 4370 50  0000 C CNN
F 2 "" H 935 4230 50  0001 C CNN
F 3 "" H 935 4230 50  0001 C CNN
	1    935  4230
	1    0    0    -1  
$EndComp
$Comp
L CONN_02X03 P102
U 1 1 58F3F413
P 1245 4395
F 0 "P102" H 1245 4595 50  0000 C CNN
F 1 "Y AXIS" H 1245 4195 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_2x03_Pitch2.54mm" H 1245 3195 50  0001 C CNN
F 3 "" H 1245 3195 50  0001 C CNN
	1    1245 4395
	1    0    0    -1  
$EndComp
Wire Wire Line
	1565 4395 1495 4395
Wire Wire Line
	1565 4255 1495 4255
Wire Wire Line
	1495 4255 1495 4295
Wire Wire Line
	1570 4535 1495 4535
Wire Wire Line
	1495 4535 1495 4495
Wire Wire Line
	935  4545 935  4495
Wire Wire Line
	935  4495 995  4495
Wire Wire Line
	935  4230 935  4295
Wire Wire Line
	935  4295 995  4295
Wire Wire Line
	925  4395 995  4395
Text GLabel 925  5200 0    60   Output ~ 0
Fault_Z
Text GLabel 1565 5060 2    60   Input ~ 0
STEP_Z
Text GLabel 1565 5200 2    60   Input ~ 0
DIR_Z
Text GLabel 1570 5340 2    60   Input ~ 0
ENABLE_Z
$Comp
L GND #PWR027
U 1 1 58F3F8E3
P 935 5350
F 0 "#PWR027" H 935 5100 50  0001 C CNN
F 1 "GND" H 935 5200 50  0000 C CNN
F 2 "" H 935 5350 50  0001 C CNN
F 3 "" H 935 5350 50  0001 C CNN
	1    935  5350
	1    0    0    -1  
$EndComp
$Comp
L +3V3 #PWR028
U 1 1 58F3F8E9
P 935 5035
F 0 "#PWR028" H 935 4885 50  0001 C CNN
F 1 "+3V3" H 935 5175 50  0000 C CNN
F 2 "" H 935 5035 50  0001 C CNN
F 3 "" H 935 5035 50  0001 C CNN
	1    935  5035
	1    0    0    -1  
$EndComp
$Comp
L CONN_02X03 P103
U 1 1 58F3F8EF
P 1245 5200
F 0 "P103" H 1245 5400 50  0000 C CNN
F 1 "Z AXIS" H 1245 5000 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_2x03_Pitch2.54mm" H 1245 4000 50  0001 C CNN
F 3 "" H 1245 4000 50  0001 C CNN
	1    1245 5200
	1    0    0    -1  
$EndComp
Wire Wire Line
	1565 5200 1495 5200
Wire Wire Line
	1565 5060 1495 5060
Wire Wire Line
	1495 5060 1495 5100
Wire Wire Line
	1570 5340 1495 5340
Wire Wire Line
	1495 5340 1495 5300
Wire Wire Line
	935  5350 935  5300
Wire Wire Line
	935  5300 995  5300
Wire Wire Line
	935  5035 935  5100
Wire Wire Line
	935  5100 995  5100
Wire Wire Line
	925  5200 995  5200
Text GLabel 925  6005 0    60   Output ~ 0
Fault_A
Text GLabel 1565 5865 2    60   Input ~ 0
STEP_A
Text GLabel 1565 6005 2    60   Input ~ 0
DIR_A
Text GLabel 1570 6145 2    60   Input ~ 0
ENABLE_A
$Comp
L GND #PWR029
U 1 1 58F3FC26
P 935 6155
F 0 "#PWR029" H 935 5905 50  0001 C CNN
F 1 "GND" H 935 6005 50  0000 C CNN
F 2 "" H 935 6155 50  0001 C CNN
F 3 "" H 935 6155 50  0001 C CNN
	1    935  6155
	1    0    0    -1  
$EndComp
$Comp
L +3V3 #PWR030
U 1 1 58F3FC2C
P 935 5840
F 0 "#PWR030" H 935 5690 50  0001 C CNN
F 1 "+3V3" H 935 5980 50  0000 C CNN
F 2 "" H 935 5840 50  0001 C CNN
F 3 "" H 935 5840 50  0001 C CNN
	1    935  5840
	1    0    0    -1  
$EndComp
$Comp
L CONN_02X03 P104
U 1 1 58F3FC32
P 1245 6005
F 0 "P104" H 1245 6205 50  0000 C CNN
F 1 "A AXIS" H 1245 5805 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_2x03_Pitch2.54mm" H 1245 4805 50  0001 C CNN
F 3 "" H 1245 4805 50  0001 C CNN
	1    1245 6005
	1    0    0    -1  
$EndComp
Wire Wire Line
	1565 6005 1495 6005
Wire Wire Line
	1565 5865 1495 5865
Wire Wire Line
	1495 5865 1495 5905
Wire Wire Line
	1570 6145 1495 6145
Wire Wire Line
	1495 6145 1495 6105
Wire Wire Line
	935  6155 935  6105
Wire Wire Line
	935  6105 995  6105
Wire Wire Line
	935  5840 935  5905
Wire Wire Line
	935  5905 995  5905
Wire Wire Line
	925  6005 995  6005
$Comp
L +3V3 #PWR031
U 1 1 58F423C8
P 4700 5640
F 0 "#PWR031" H 4700 5490 50  0001 C CNN
F 1 "+3V3" H 4700 5780 50  0000 C CNN
F 2 "" H 4700 5640 50  0001 C CNN
F 3 "" H 4700 5640 50  0001 C CNN
	1    4700 5640
	1    0    0    -1  
$EndComp
$Comp
L LED_Dual_AACC D101
U 1 1 58F4CA8A
P 9520 4970
F 0 "D101" V 9390 5255 50  0000 C CNN
F 1 "Blue_Red" V 9710 4685 50  0000 C CNN
F 2 "TiZed:LEDs_AACC_1.6mm_1.5mm" H 9550 4970 50  0001 C CNN
F 3 "" H 9550 4970 50  0001 C CNN
	1    9520 4970
	0    1    1    0   
$EndComp
$Comp
L GND #PWR032
U 1 1 58F4D04F
P 9420 5750
F 0 "#PWR032" H 9420 5500 50  0001 C CNN
F 1 "GND" H 9420 5600 50  0000 C CNN
F 2 "" H 9420 5750 50  0001 C CNN
F 3 "" H 9420 5750 50  0001 C CNN
	1    9420 5750
	1    0    0    -1  
$EndComp
Wire Wire Line
	9620 4630 9620 4670
Wire Wire Line
	9420 4630 9420 4670
Wire Wire Line
	9620 5270 9620 5755
Wire Wire Line
	9420 5270 9420 5310
$Comp
L Q_NPN_BEC Q101
U 1 1 5907D80E
P 9320 5510
F 0 "Q101" H 9520 5560 50  0000 L CNN
F 1 "MMBT2222A" H 9520 5460 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:SOT-23" H 9520 5610 50  0001 C CNN
F 3 "" H 9320 5510 50  0001 C CNN
	1    9320 5510
	1    0    0    -1  
$EndComp
$Comp
L Q_NPN_BEC Q102
U 1 1 5907D91C
P 9520 5955
F 0 "Q102" H 9720 6005 50  0000 L CNN
F 1 "MMBT2222A" H 9720 5905 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:SOT-23" H 9720 6055 50  0001 C CNN
F 3 "" H 9520 5955 50  0001 C CNN
	1    9520 5955
	1    0    0    -1  
$EndComp
Wire Wire Line
	9420 5750 9420 5710
Wire Wire Line
	9620 6195 9620 6155
$Comp
L R_Small R103
U 1 1 5907FD43
P 8970 5510
F 0 "R103" V 9045 5500 50  0000 L CNN
F 1 "4.7k" V 8875 5465 50  0000 L CNN
F 2 "Resistors_SMD:R_0603" H 8970 5510 50  0001 C CNN
F 3 "" H 8970 5510 50  0001 C CNN
	1    8970 5510
	0    -1   -1   0   
$EndComp
$Comp
L R_Small R104
U 1 1 59080054
P 9170 5955
F 0 "R104" V 9245 5940 50  0000 L CNN
F 1 "4.7k" V 9070 5920 50  0000 L CNN
F 2 "Resistors_SMD:R_0603" H 9170 5955 50  0001 C CNN
F 3 "" H 9170 5955 50  0001 C CNN
	1    9170 5955
	0    -1   -1   0   
$EndComp
Wire Wire Line
	9270 5955 9320 5955
Wire Wire Line
	9070 5510 9120 5510
Wire Wire Line
	9070 5955 9000 5955
Wire Wire Line
	8870 5510 8800 5510
Wire Wire Line
	9420 4285 9620 4285
Text GLabel 9690 4145 2    60   Input ~ 0
V_leds
Wire Wire Line
	9515 4145 9690 4145
Wire Wire Line
	9515 4145 9515 4285
Connection ~ 9515 4285
$EndSCHEMATC
