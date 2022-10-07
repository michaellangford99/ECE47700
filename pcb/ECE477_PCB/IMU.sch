EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 3 3
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Text GLabel 2100 3650 0    50   Input ~ 0
I2C1_SCL
Text GLabel 2100 3550 0    50   Input ~ 0
I2C1_SDA
$Comp
L Sensor_Motion:MPU-6050 U?
U 1 1 638B6202
P 3650 3850
AR Path="/638B6202" Ref="U?"  Part="1" 
AR Path="/638AC442/638B6202" Ref="U?"  Part="1" 
F 0 "U?" H 3200 3300 50  0000 C CNN
F 1 "MPU-6050" H 3350 3200 50  0000 C CNN
F 2 "Sensor_Motion:InvenSense_QFN-24_4x4mm_P0.5mm" H 3650 3050 50  0001 C CNN
F 3 "https://store.invensense.com/datasheets/invensense/MPU-6050_DataSheet_V3%204.pdf" H 3650 3700 50  0001 C CNN
F 4 "C24112" H 3650 3850 50  0001 C CNN "LCSC Part #"
F 5 "C24112" H 3650 3850 50  0001 C CNN "LCSC"
	1    3650 3850
	1    0    0    -1  
$EndComp
Wire Wire Line
	2850 3800 2850 3750
Wire Wire Line
	2850 3750 2950 3750
$Comp
L Device:C_Small C?
U 1 1 638B620C
P 4950 4300
AR Path="/638B620C" Ref="C?"  Part="1" 
AR Path="/638AC442/638B620C" Ref="C?"  Part="1" 
F 0 "C?" H 5042 4346 50  0000 L CNN
F 1 "2.2nF, 50V" H 5042 4255 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 4950 4300 50  0001 C CNN
F 3 "~" H 4950 4300 50  0001 C CNN
F 4 "C28260" H 4950 4300 50  0001 C CNN "LCSC Part #"
F 5 "C28260" H 4950 4300 50  0001 C CNN "LCSC"
	1    4950 4300
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C?
U 1 1 638B6214
P 4400 4300
AR Path="/638B6214" Ref="C?"  Part="1" 
AR Path="/638AC442/638B6214" Ref="C?"  Part="1" 
F 0 "C?" H 4492 4346 50  0000 L CNN
F 1 "0.1uF 2V" H 4492 4255 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 4400 4300 50  0001 C CNN
F 3 "~" H 4400 4300 50  0001 C CNN
F 4 "C1525" H 4400 4300 50  0001 C CNN "LCSC Part #"
F 5 "C1525" H 4400 4300 50  0001 C CNN "LCSC"
	1    4400 4300
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 638B621A
P 4400 4400
AR Path="/638B621A" Ref="#PWR?"  Part="1" 
AR Path="/638AC442/638B621A" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 4400 4150 50  0001 C CNN
F 1 "GND" H 4405 4227 50  0000 C CNN
F 2 "" H 4400 4400 50  0001 C CNN
F 3 "" H 4400 4400 50  0001 C CNN
	1    4400 4400
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 638B6220
P 4950 4400
AR Path="/638B6220" Ref="#PWR?"  Part="1" 
AR Path="/638AC442/638B6220" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 4950 4150 50  0001 C CNN
F 1 "GND" H 4955 4227 50  0000 C CNN
F 2 "" H 4950 4400 50  0001 C CNN
F 3 "" H 4950 4400 50  0001 C CNN
	1    4950 4400
	1    0    0    -1  
$EndComp
Wire Wire Line
	4950 4200 4950 4050
Wire Wire Line
	4350 4150 4400 4150
Wire Wire Line
	4400 4150 4400 4200
Wire Wire Line
	4350 4050 4950 4050
NoConn ~ 4350 3850
NoConn ~ 4350 3750
NoConn ~ 4350 3550
NoConn ~ 2950 4050
$Comp
L Device:C_Small C?
U 1 1 638B6230
P 3250 3100
AR Path="/638B6230" Ref="C?"  Part="1" 
AR Path="/638AC442/638B6230" Ref="C?"  Part="1" 
F 0 "C?" V 3479 3100 50  0000 C CNN
F 1 "10nF, 4V" V 3388 3100 50  0000 C CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 3250 3100 50  0001 C CNN
F 3 "~" H 3250 3100 50  0001 C CNN
F 4 "C15195" H 3250 3100 50  0001 C CNN "LCSC Part #"
F 5 "C15195" H 3250 3100 50  0001 C CNN "LCSC"
	1    3250 3100
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 638B6236
P 3050 3150
AR Path="/638B6236" Ref="#PWR?"  Part="1" 
AR Path="/638AC442/638B6236" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 3050 2900 50  0001 C CNN
F 1 "GND" H 3055 2977 50  0000 C CNN
F 2 "" H 3050 3150 50  0001 C CNN
F 3 "" H 3050 3150 50  0001 C CNN
	1    3050 3150
	1    0    0    -1  
$EndComp
Wire Wire Line
	3050 3150 3050 3100
Wire Wire Line
	3050 3100 3150 3100
Wire Wire Line
	3350 3100 3550 3100
Wire Wire Line
	3550 3100 3550 3150
$Comp
L power:+3.3V #PWR?
U 1 1 638B6240
P 3550 3050
AR Path="/638B6240" Ref="#PWR?"  Part="1" 
AR Path="/638AC442/638B6240" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 3550 2900 50  0001 C CNN
F 1 "+3.3V" H 3565 3223 50  0000 C CNN
F 2 "" H 3550 3050 50  0001 C CNN
F 3 "" H 3550 3050 50  0001 C CNN
	1    3550 3050
	1    0    0    -1  
$EndComp
Wire Wire Line
	3550 3050 3550 3100
Connection ~ 3550 3100
Wire Wire Line
	3550 3100 3750 3100
Wire Wire Line
	3750 3100 3750 3150
$Comp
L Device:C_Small C?
U 1 1 638B624C
P 4000 3100
AR Path="/638B624C" Ref="C?"  Part="1" 
AR Path="/638AC442/638B624C" Ref="C?"  Part="1" 
F 0 "C?" V 4229 3100 50  0000 C CNN
F 1 "0.1uF 4V" V 4138 3100 50  0000 C CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 4000 3100 50  0001 C CNN
F 3 "~" H 4000 3100 50  0001 C CNN
F 4 "C1525" H 4000 3100 50  0001 C CNN "LCSC Part #"
F 5 "C1525" H 4000 3100 50  0001 C CNN "LCSC"
	1    4000 3100
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 638B6252
P 4250 3150
AR Path="/638B6252" Ref="#PWR?"  Part="1" 
AR Path="/638AC442/638B6252" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 4250 2900 50  0001 C CNN
F 1 "GND" H 4255 2977 50  0000 C CNN
F 2 "" H 4250 3150 50  0001 C CNN
F 3 "" H 4250 3150 50  0001 C CNN
	1    4250 3150
	1    0    0    -1  
$EndComp
Wire Wire Line
	3900 3100 3750 3100
Connection ~ 3750 3100
Wire Wire Line
	4100 3100 4250 3100
Wire Wire Line
	4250 3150 4250 3100
NoConn ~ 2950 4150
$Comp
L Device:R_Small R?
U 1 1 638B625F
P 2250 3400
AR Path="/638B625F" Ref="R?"  Part="1" 
AR Path="/638AC442/638B625F" Ref="R?"  Part="1" 
F 0 "R?" H 2309 3446 50  0000 L CNN
F 1 "4.7K" H 2309 3355 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 2250 3400 50  0001 C CNN
F 3 "~" H 2250 3400 50  0001 C CNN
F 4 "C23162" H 2250 3400 50  0001 C CNN "LCSC Part #"
F 5 "C23162" H 2250 3400 50  0001 C CNN "LCSC"
	1    2250 3400
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R?
U 1 1 638B6267
P 2550 3400
AR Path="/638B6267" Ref="R?"  Part="1" 
AR Path="/638AC442/638B6267" Ref="R?"  Part="1" 
F 0 "R?" H 2609 3446 50  0000 L CNN
F 1 "4.7K" H 2609 3355 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 2550 3400 50  0001 C CNN
F 3 "~" H 2550 3400 50  0001 C CNN
F 4 "C23162" H 2550 3400 50  0001 C CNN "LCSC Part #"
F 5 "C23162" H 2550 3400 50  0001 C CNN "LCSC"
	1    2550 3400
	1    0    0    -1  
$EndComp
Wire Wire Line
	2950 3650 2250 3650
Wire Wire Line
	2250 3650 2250 3500
Wire Wire Line
	2550 3500 2550 3550
Wire Wire Line
	2550 3550 2950 3550
$Comp
L power:+3.3V #PWR?
U 1 1 638B6271
P 2250 3300
AR Path="/638B6271" Ref="#PWR?"  Part="1" 
AR Path="/638AC442/638B6271" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 2250 3150 50  0001 C CNN
F 1 "+3.3V" H 2265 3473 50  0000 C CNN
F 2 "" H 2250 3300 50  0001 C CNN
F 3 "" H 2250 3300 50  0001 C CNN
	1    2250 3300
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR?
U 1 1 638B6277
P 2550 3300
AR Path="/638B6277" Ref="#PWR?"  Part="1" 
AR Path="/638AC442/638B6277" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 2550 3150 50  0001 C CNN
F 1 "+3.3V" H 2565 3473 50  0000 C CNN
F 2 "" H 2550 3300 50  0001 C CNN
F 3 "" H 2550 3300 50  0001 C CNN
	1    2550 3300
	1    0    0    -1  
$EndComp
Wire Wire Line
	2100 3650 2250 3650
Connection ~ 2250 3650
Wire Wire Line
	2100 3550 2550 3550
Connection ~ 2550 3550
$Comp
L power:GND #PWR?
U 1 1 638B6281
P 3650 4550
AR Path="/638B6281" Ref="#PWR?"  Part="1" 
AR Path="/638AC442/638B6281" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 3650 4300 50  0001 C CNN
F 1 "GND" H 3655 4377 50  0000 C CNN
F 2 "" H 3650 4550 50  0001 C CNN
F 3 "" H 3650 4550 50  0001 C CNN
	1    3650 4550
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 638B6287
P 2850 3800
AR Path="/638B6287" Ref="#PWR?"  Part="1" 
AR Path="/638AC442/638B6287" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 2850 3550 50  0001 C CNN
F 1 "GND" H 2855 3627 50  0000 C CNN
F 2 "" H 2850 3800 50  0001 C CNN
F 3 "" H 2850 3800 50  0001 C CNN
	1    2850 3800
	1    0    0    -1  
$EndComp
$Comp
L Sensor_Motion:MPU-9250 U?
U 1 1 6395568F
P 3600 1400
F 0 "U?" H 3600 411 50  0000 C CNN
F 1 "MPU-9250" H 3600 320 50  0000 C CNN
F 2 "Sensor_Motion:InvenSense_QFN-24_3x3mm_P0.4mm" H 3600 400 50  0001 C CNN
F 3 "https://store.invensense.com/datasheets/invensense/MPU9250REV1.0.pdf" H 3600 1250 50  0001 C CNN
	1    3600 1400
	1    0    0    -1  
$EndComp
$Comp
L Sensor_Motion:LSM6DS3 U?
U 1 1 63956215
P 6900 2700
F 0 "U?" H 7544 2746 50  0000 L CNN
F 1 "LSM6DS3" H 7544 2655 50  0000 L CNN
F 2 "Package_LGA:LGA-14_3x2.5mm_P0.5mm_LayoutBorder3x4y" H 6500 2000 50  0001 L CNN
F 3 "www.st.com/resource/en/datasheet/lsm6ds3.pdf" H 7000 2050 50  0001 C CNN
	1    6900 2700
	1    0    0    -1  
$EndComp
$EndSCHEMATC