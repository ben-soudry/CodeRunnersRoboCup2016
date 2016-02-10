//Pinout.h

//RoboCup 2016 PCB R1 Pinout:
	//Other constants needed to interface with hardware are included
//Sensors
	//Ground Sensor + Light Gate
	#define GS_ANI A1
	#define GS_S0 26
	#define GS_S1 27
	#define GS_S2 28
	#define GS_S3 29
	#define VLED 30
	#define GS0 0 
	#define GS1 1
    #define GS2 2
	#define GS3 3
    #define GS4 4
    #define GS5 5
	#define GS6 6
    #define GS7 7
    #define GS8 8
    #define GS9 9
    #define GS10 10
    #define GS11 11
    #define GS12 12
    #define GS13 13
    #define GS14 14
	#define GS_THRESHOLD 650
    #define LG 15
	#define LG_THRESHOLD 972
	//IR Sensor Array
	#define IR_ANI A0
	#define IR_S0 22
	#define IR_S1 23
	#define IR_S2 24
	#define IR_S3 25
	#define IR1 0 
	#define IR2 1
    #define IR3 2
	#define IR4 3
    #define IR5 4
    #define IR6 5
	#define IR7 6
    #define IR8 7
    #define IR9 8
    #define IR10 9
    #define IR11 10
    #define IR12 11
    #define IR13 12
    #define IR14 13
    #define IR15 14
    #define IR16 15
	#define IR17 A2
	#define IR18 A3
	#define IR19 A4
	#define IR20 A5
	#define IR_min 10
	#define IR_max 290
	//Sonar Sensors
	#define US1 A6 //Front Left
	#define US2 A7 //Front Middle
	#define US3 A8 //Front Right
	#define US4 A9 //Left
	#define US5 A10 //Back
	#define US6 A11 //Right
	//Compass Sensor - Serial1
	#define compassCommand 0x13
	#define startCalibration1 0xF0 //F0
	#define startCalibration2 0xF5 //F5
	#define startCalibration3 0xF7 //F7 for simple calibration //F6 for tilt-compensation
	#define endCalibration 0xF8


//Mechanical Control
	//Motor Drivers
	#define M1 1
	#define M1_IN1 31
	#define M1_IN2 32
	#define M1_PWM 2

	#define M2 2
	#define M2_IN1 33
	#define M2_IN2 34
	#define M2_PWM 3

	#define M3 3
	#define M3_IN1 35
	#define M3_IN2 36
	#define M3_PWM 4

    #define M4 4
	#define M4_IN1 37
	#define M4_IN2 38
	#define M4_PWM 5
    
    #define M5 5
	#define M5_IN1 39
	#define M5_IN2 40
	#define M5_PWM 6
	//Solenoid
	#define KICKER 7

//UI/Debugging
	//Buttons
	#define B1 41
	#define B2 42
	#define B3 43
	//LEDs
	#define LED1 44
	#define LED2 45
	#define LED3 46
	#define LED4 47
	//LCD
	#define LCD_RS 48
	#define LCD_E 49
	#define LCD_DB4 50
	#define LCD_DB5 51
	#define LCD_DB6 52
	#define LCD_DB7 53

//Robot Communication
	//Bluetooth Module - Serial2
