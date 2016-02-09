#include <HID.h>

//CodeRunnersRoboCup.ino

#include "Pinout.h" 
#include "StateMachine.h"
int state = START_FORWARD;


float fieldNorth;
long startTime;

//LCD Library
#include <LiquidCrystal.h>
LiquidCrystal lcd(LCD_RS, LCD_E, LCD_DB4, LCD_DB5, LCD_DB6, LCD_DB7);
String lcdLine1;
String lcdLine2;
long lastLCDUpdateTime;


float lastBallAngle = -1;
int ballProximity = 0; //0 - No Ball //1 - Far away //2 - Moderate //3 - Close //4 - Very Close
long startKickTime;

int groundSensor[15];

void setup() 
{
  Serial.begin(9600);
  Serial.println("Start");

  //Motor Setup
  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);
  pinMode(M1_PWM, OUTPUT);
  
  pinMode(M2_IN1, OUTPUT);
  pinMode(M2_IN2, OUTPUT);
  pinMode(M2_PWM, OUTPUT);
  
  pinMode(M3_IN1, OUTPUT);
  pinMode(M3_IN2, OUTPUT);
  pinMode(M3_PWM, OUTPUT);

  pinMode(M4_IN1, OUTPUT);
  pinMode(M4_IN2, OUTPUT);
  pinMode(M4_PWM, OUTPUT);
  
  pinMode(M5_IN1, OUTPUT);
  pinMode(M5_IN2, OUTPUT);
  pinMode(M5_PWM, OUTPUT);

  //Kicker Setup
  pinMode(KICKER, OUTPUT);
  digitalWrite(KICKER, LOW);

  //Compass Sensor Setup
  Serial1.begin(9600);

  //IR array setup
  pinMode(IR_ANI, INPUT);
  pinMode(IR_S0, OUTPUT); pinMode(IR_S1, OUTPUT); pinMode(IR_S2, OUTPUT); pinMode(IR_S3, OUTPUT);

  //Ground Sensor Setup
  pinMode(GS_ANI, INPUT);
  pinMode(GS_S0, OUTPUT); pinMode(GS_S1, OUTPUT); pinMode(GS_S2, OUTPUT); pinMode(GS_S3, OUTPUT);
  pinMode(VLED, OUTPUT);
  digitalWrite(VLED, HIGH);
  
  //Ultrasonic Sensor Setup
  pinMode(US1, INPUT);
  pinMode(US2, INPUT);
  pinMode(US3, INPUT);
  pinMode(US4, INPUT);
  pinMode(US5, INPUT);
  pinMode(US6, INPUT);

  //LCD setup
  pinMode(LCD_RS, OUTPUT);
  pinMode(LCD_E,OUTPUT);
  pinMode(LCD_DB4, OUTPUT);
  pinMode(LCD_DB5, OUTPUT);
  pinMode(LCD_DB6, OUTPUT);
  pinMode(LCD_DB7, OUTPUT);
  lcd.begin(8, 2);
  lcd.print("RoboCup!");

  //LED setup
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(LED4, OUTPUT);
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);
  digitalWrite(LED3, LOW);
  digitalWrite(LED4, LOW);

  //Button setup
  pinMode(B1, INPUT);
  pinMode(B2, INPUT);
  pinMode(B3, INPUT);

  delay(50);
  fieldNorth = readCompass();
  lastLCDUpdateTime = millis();
}

void loop() 
{
  //State Machine
  switch(state)
  { 
    //Emergency Stop State
    case E_STOP:

      break;
    //Forward Strategic States
    case FORWARD_BALL_PURSUIT:
      forwardBallPursuit();
      break;
    case FORWARD_HAS_BALL:
      forwardHasBall();
      break;
    case FORWARD_AVOID_OUTOFBOUNDS:

      break; 
    //Defense Strategic States    
    case DEFENSE_MOVE_AND_BLOCK:

      break; 
    case DEFENSE_HAS_BALL:

      break;
    case DEFENSE_AVOID_OUTOFBOX:

      break; 
    case DEFENSE_AVOID_OUTOFBOUNDS:

      break; 
    //Menu States
    case START_FORWARD:
      startForward();
      break;
    case START_DEFENSE:
      startDefense();
      break;
    case IR_TEST:
      IR_Test();
      break;
    case CMPS_TEST:
      compassTest();
      break;
    case CMPS_CALIBRATION:
      compassCalibration();
      break;
    case GS_TEST:
      groundSensorTest();
      break;
    case GS_CALIBRATION:
      groundSensorCalibration();
      break;
    case LG_TEST:
      lightGateTest();
      break;
    case LG_CALIBRATION:
      lightGateCalibration();
      break;
    case KICKER_TEST:
      kickerTest();
      break;
    }


  long currentTime = millis();
  if(currentTime - lastLCDUpdateTime > 200)
  {
    LCD_SendBuffer();
    lastLCDUpdateTime = millis();
  }
  if((state == FORWARD_BALL_PURSUIT || state == FORWARD_HAS_BALL) && ((currentTime - startTime) > 2500))
  {
    Serial.println("Stop");
    digitalWrite(LED1, LOW);
    digitalWrite(LED2, LOW);
    digitalWrite(LED3, LOW);
    digitalWrite(LED4, LOW);
    while(true)
    {
      
       motor(M1, 0);
       motor(M2, 0);
       motor(M3, 0);
       motor(M4, 0);
       delay(10);
      //stop
    }
  }
}
//////////////
//State Machine Functions - Menu//
//////////////
void startForward()
{
  //lcd.setCursor(0,0);
  //lcd.print("Start   ");
  //lcd.setCursor(0,1);
  //lcd.print("Forward ");
  lcdLine1 =  "Start   ";
  lcdLine2 =  "Forward ";

  //Reading Buttons
  if(digitalRead(B3) == HIGH)
  {
    delay(500);
    startTime = millis();
    fieldNorth = readCompass();
    state = FORWARD_BALL_PURSUIT; //Change state to the default forward state
  }
  else if(digitalRead(B1) == HIGH)
  {
    state = KICKER_TEST;
    delay(500);
  }
  else if(digitalRead(B2) == HIGH)
  {
    state = START_DEFENSE;
    delay(500);
  }
}
void startDefense()
{
  //lcd.setCursor(0,0);
  //lcd.print("Start   ");
  //lcd.setCursor(0,1);
  //lcd.print("Defense ");
  lcdLine1 =  "Start   ";
  lcdLine2 =  "Defense ";

  //Reading Buttons
  if(digitalRead(B3) == HIGH)
  {
    state = DEFENSE_MOVE_AND_BLOCK; //Change state to the default defense state
    delay(500);
  }
  else if(digitalRead(B1) == HIGH)
  {
    state = START_FORWARD;
    delay(500);
  }
  else if(digitalRead(B2) == HIGH)
  {
    state = IR_TEST;
    delay(500);
    
  }
}

bool IR_TestMode = 0; //0 for ballAngle, 1 for ballProximity

void IR_Test()
{ 
  //lcd.setCursor(0,0);
  //lcd.print("IR Test ");
  lcdLine1 =  "IR Test ";
  float ballAngle = readIR();
  if(IR_TestMode == 0)
  {
    //lcd.setCursor(0,1);
    //lcd.print(ballAngle);
    if(ballAngle == -1)
    {
      lcdLine2 = "No Ball";
    }
    else
    {
      lcdLine2 = String(ballAngle);
    }
    Serial.print("Ball Angle: ");
    Serial.println(ballAngle);
  }
  else if(IR_TestMode == 1)
  {
    if(ballProximity == 0)
    {
      lcdLine2 = "No Ball";
    }
    else if(ballProximity == 1)
    {
      lcdLine2 = "Very Far";
    }
    else if(ballProximity == 2)
    {
      lcdLine2 = "Far Away";
    }
    else if(ballProximity == 3)
    {
      lcdLine2 = "Moderate";
    }
    else if(ballProximity == 4)
    {
      lcdLine2 = "Close";
    }
  }
  //Reading Buttons
  if(digitalRead(B3) == HIGH) //Toggle IR Test Mode
  {
    if(IR_TestMode == 0)
    {
      IR_TestMode = 1;
    }
    else
    {
      IR_TestMode = 0;
    }
    delay(500);
  }
  else if(digitalRead(B1) == HIGH)
  {
    state = START_DEFENSE;
    delay(500);
  }
  else if(digitalRead(B2) == HIGH)
  {
    state = CMPS_TEST;
    delay(500);
    
  }
}
void compassTest()
{
  //lcd.setCursor(0,0);
  //lcd.print("Compass ");
  lcdLine1 =  "Compass ";

  float compassHeading = readCompass();
  //lcd.setCursor(0,1);
  //lcd.print(compassHeading);
  lcdLine2 =  String(compassHeading);

  Serial.print("Compass Heading: ");
  Serial.println(compassHeading);

  //Reading Buttons
  if(digitalRead(B3) == HIGH)
  {
  }
  else if(digitalRead(B1) == HIGH)
  {
    state = IR_TEST;
    delay(500);
    
  }
  else if(digitalRead(B2) == HIGH)
  {
    state = CMPS_CALIBRATION;
    delay(500);
  }
  //delay(20);
}
void compassCalibration()
{
  lcdLine1 = "CMPS_Cal";

  lcdLine2 =  "Start   ";
  Serial.println("Press to Start then rotate Robot 360 Degrees to Calibrate");
  if(digitalRead(B3) == HIGH) //Run Calibration Sequence
  {
    lcd.setCursor(0,1);
    lcd.print("Confirm ");
    //send calibration start sequence and recieve confirmation bytes
    byte confirm1;
    byte confirm2;
    byte confirm3;
    Serial1.write(startCalibration1);
    delay(10);
    if(Serial1.available()){  
      confirm1 = Serial1.read();
    }
    else{
      Serial.println("Error - compass not available");
      lcd.setCursor(0,1);
      lcd.print("Error   ");
      return;
    }
    delay(100);
    Serial1.write(startCalibration2);
    delay(10);
    if(Serial1.available()){  
      byte confirm2 = Serial1.read();
    }
    else
    {
      Serial.println("Error - compass not available");
      lcd.setCursor(0,1);
      lcd.print("Error   ");
      return;
    }
    delay(100);
    Serial1.write(startCalibration3);
    delay(10);
    if(Serial1.available()){  
      confirm3 = Serial1.read();
    }
    else{
      Serial.println("Error - compass not available");
      lcd.setCursor(0,1);
      lcd.print("Error   ");
      return;
    }
    if(confirm1 != 0x55 || confirm2 != 0x55 || confirm3 != 0x55)
    {
      Serial.println("Error - Wrong confirmation code");
      Serial.println(confirm1);
      Serial.println(confirm2);
      Serial.println(confirm3);
      lcd.setCursor(0,1);
      //lcd.print("Error   ");
      //return;
    }
    delay(100);
    
    byte confirm4;
    Serial.println("Press Button to finish when you cannot get further LED flashes");
    lcd.print("Rotate..");
    while(digitalRead(B3)== LOW){  
    }
    Serial1.write(endCalibration);
    delay(5);
    if(Serial1.available()){  
      confirm4 = Serial1.read();
    }
    else
    {
      Serial.println("Error - compass not available");
      return;
    }
    Serial.println("Calibration Complete");
    lcd.setCursor(0,1);
    lcd.print("Complete");
    delay(1000);
  }
  else if(digitalRead(B1) == HIGH)
  {
    state = CMPS_TEST;
    delay(500);
    
  }
  else if(digitalRead(B2) == HIGH)
  {
    state = GS_TEST;
    delay(500);
  }
}
void groundSensorTest()
{
  lcdLine1 = "GS Test ";

  lcdLine2 = "";
  readGroundSensor();
  if(groundSensor[GS0] > GS_THRESHOLD || groundSensor[GS1] > GS_THRESHOLD || groundSensor[GS2] > GS_THRESHOLD)
  {
    digitalWrite(LED1, HIGH);
  }
  else{
    digitalWrite(LED1, LOW);
  }
  if(groundSensor[GS3] > GS_THRESHOLD || groundSensor[GS4] > GS_THRESHOLD || groundSensor[GS5] > GS_THRESHOLD || groundSensor[GS6] > GS_THRESHOLD)
  {
    digitalWrite(LED2, HIGH);
  }
  else{
    digitalWrite(LED2, LOW);
  }
  if(groundSensor[GS7] > GS_THRESHOLD || groundSensor[GS8] > GS_THRESHOLD || groundSensor[GS9] > GS_THRESHOLD || groundSensor[GS10] > GS_THRESHOLD)
  {
    digitalWrite(LED4, HIGH);
  }
  else{
    digitalWrite(LED4, LOW);
  }
  if(groundSensor[GS11] > GS_THRESHOLD || groundSensor[GS12] > GS_THRESHOLD || groundSensor[GS13] > GS_THRESHOLD || groundSensor[GS14] > GS_THRESHOLD)
  {
    digitalWrite(LED3, HIGH);
  }
  else{
    digitalWrite(LED3, LOW);
  }
  //Reading Buttons
  if(digitalRead(B3) == HIGH)
  {
  }
  else if(digitalRead(B1) == HIGH)
  {
    state = CMPS_CALIBRATION;
    delay(500);
  }
  else if(digitalRead(B2) == HIGH)
  {
    state = GS_CALIBRATION;
    delay(500);
  }
}
void groundSensorCalibration()
{
  //lcd.setCursor(0,0);
  //lcd.print("GS Cal  ");
  lcdLine1 =  "GS Cal  ";
  //lcd.setCursor(0,1);
  //lcd.print("N/A     ");
  lcdLine2 =  "N/A     ";
  //Reading Buttons
  if(digitalRead(B3) == HIGH)
  {
  }
  else if(digitalRead(B1) == HIGH)
  {
    state = GS_TEST;
    delay(500);
  }
  else if(digitalRead(B2) == HIGH)
  {
    state = LG_TEST;
    delay(500);
  }
  
}
void lightGateTest()
{
  //lcd.setCursor(0,0);
  //lcd.print("LG Test ");
  lcdLine1 =  "LG Test ";
  //lcd.setCursor(0,1);
  //lcd.print("N/A     ");
  bool LG_Status = readLightGate();
  float IR_Status = readIR();
  //float ballAngle = 90.0;
  //Serial.print("LG_Status: "); Serial.println(LG_Status);
  //Serial.print("IR_Status: "); Serial.println(IR_Status);
  //Basic Light Gate testing 
  if(LG_Status == true)
  {
    digitalWrite(LED2, HIGH);
    digitalWrite(LED4, LOW);
    lcdLine2 =  "Object";
    if(IR_Status > 80 && IR_Status < 100 && ballProximity == 4)
    {
      digitalWrite(LED1, HIGH);
      lcdLine2 =  "Ball!";
    }
  }
  else
  {
    digitalWrite(LED2, LOW);
    digitalWrite(LED4, HIGH);
    digitalWrite(LED1, LOW);
    lcdLine2 =  "Empty";
  }

  
  //Reading Buttons
  if(digitalRead(B3) == HIGH)
  {
  }
  else if(digitalRead(B1) == HIGH)
  {
    state = GS_CALIBRATION;
    delay(500);
  }
  else if(digitalRead(B2) == HIGH)
  {
    state = LG_CALIBRATION;
    delay(500);
  }
  
}
void lightGateCalibration()
{
  
  //lcd.setCursor(0,0);
  //lcd.print("LG Cal  ");
  lcdLine1 =  "LG Cal  ";
  //lcd.setCursor(0,1);
  //lcd.print("N/A     ");
  lcdLine2 =  "N/A     ";
  
  if(digitalRead(B3) == HIGH)
  {
    
  }
  else if(digitalRead(B1) == HIGH)
  {
    state = LG_TEST;
    delay(500);
  }
  else if(digitalRead(B2) == HIGH)
  {
    state = KICKER_TEST;
    delay(500);
  }
  
}
void kickerTest()
{
  //lcd.setCursor(0,0);
  //lcd.print("Kicker  ");
  lcdLine1 = "Kicker  ";
  //lcd.setCursor(0,1);
  //lcd.print("Ready   ");
  lcdLine2 = "Ready   ";
  
  if(digitalRead(B3) == HIGH)
  {
    lcd.setCursor(0,1);
    lcd.print("Kick!   ");
    digitalWrite(KICKER, HIGH);
    digitalWrite(LED2, HIGH);
    delay(80);
    digitalWrite(KICKER, LOW);
    digitalWrite(LED2,LOW);
    delay(600);
  }
  else if(digitalRead(B1) == HIGH)
  {
    state = LG_CALIBRATION;
    delay(500);
  }
  else if(digitalRead(B2) == HIGH)
  {
    state = START_FORWARD;
    delay(500);
  }
  
}
//////////////
//State Machine Functions - Forward Strategic States//
//////////////

void forwardBallPursuit()
{
  lcdLine1 = "BallPur";
  digitalWrite(LED1, HIGH);
  digitalWrite(LED2, LOW);
  digitalWrite(LED4, LOW);

  if(digitalRead(B3) == HIGH)
  {
    state = START_FORWARD;
      motor(M1, 0);
      motor(M2, 0);
      motor(M3, 0);
      motor(M4, 0);
      delay(500);
  } 
  float ballAngle = readIR();
  lcdLine2 = String(ballProximity);

  //See if the ball was captured
  bool LG_Status = readLightGate();
  if(LG_Status == true && ballAngle > 80 && ballAngle < 100 && ballProximity == 4)
  {
    state = FORWARD_HAS_BALL; //Transition to Has Ball State
    startKickTime = millis();
  }
  //Ball Pursuit - Chooses speed and compensation modified based on proximity to ball
  int pursuitSpeed = 0;
  float compensationModifier = 0.375;
  if(ballProximity == 4)
  {
    pursuitSpeed = 110;
    compensationModifier = 0.5;
  }
  else if(ballProximity == 3)
  {
    pursuitSpeed = 135;
    compensationModifier = 0.375; //large differences in modifier creates less smooth motion
  }
  else if(ballProximity == 2)
  {
    pursuitSpeed = 160;
    compensationModifier = 0.35;
  }
  else if(ballProximity == 1)
  {
    pursuitSpeed = 180;
    compensationModifier = 0.3;
  }
  else if(ballProximity == 0)
  {
    pursuitSpeed = 0;
  }
  //Ball Pursuit - angle compensation algorithm - puts the robot on a path to capture the ball (get behind it)
  if(ballAngle != -1)
  {
    //Ensuring the bounds of ball angle are between 0 and 360
    if(ballAngle < 0 && ballAngle >= -180)
    {
      ballAngle = ballAngle + 360;
    }
    float pursuitAngle;
    if(ballAngle >= 80 && ballAngle <= 100) // Ball straight ahead
    {
      pursuitAngle = ballAngle;
    }
    else if(ballAngle > 100 && ballAngle <= 270) // Ball is on the left of robot
    {
        pursuitAngle = ballAngle + compensationModifier*(ballAngle - 100); //Add compensation
    }
    else if((ballAngle < 80 && ballAngle >= 0)) // Ball is on the right of robot
    {
      pursuitAngle = ballAngle - compensationModifier*(80 - ballAngle);
    }
    else if (ballAngle <= 360 && ballAngle > 270)
    {
      pursuitAngle = ballAngle - compensationModifier*80 - compensationModifier*(360 - ballAngle);
    }
   
    drivePID(pursuitAngle,pursuitSpeed);

  }
  else
  {
      motor(M1, 0);
      motor(M2, 0);
      motor(M3, 0);
      motor(M4, 0);
      digitalWrite(LED1, HIGH);
  }

}
void forwardHasBall()
{
  lcdLine1 = "HasBall";
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, HIGH);
  digitalWrite(LED4, LOW);


  //Timeline
  long currentTime = millis();
  if((currentTime - startKickTime) < 320) 
  {
    drivePID(90, 190); //Drive Straight Ahead
  }
  else if((currentTime - startKickTime) < 400)
  {
    digitalWrite(KICKER, HIGH); //Kick
    drivePID(90, 190);
  }
  else if((currentTime - startKickTime) < 800)
  {
    //Stop and Kicker Off
    motor(M1, 0);
    motor(M2, 0);
    motor(M3, 0);
    motor(M4, 0);
    digitalWrite(KICKER, LOW);
  }
  else
  {
    state = FORWARD_BALL_PURSUIT; //Transistion back to ball pursuit
  }

}
//////////////
//State Machine Functions - Defense Strategic States//
//////////////

//////////////
//Functions for Controlling Robot Drive//
//////////////
float kp = 4.0;  
float kd  = 0.00; 
float lastErr = 0.0;

void drivePID(int dir, int PWR)
{
  float targetVal = fieldNorth; //make the setpoint field north

  float currentVal = readCompass();
  //lcdLine2 = String(currentVal);
  //P - Proportional
  float pErr = angleDif(currentVal,targetVal); //difference of the angles
  /*
  if(pErr > 180) //if the compass passes over 0 and 360 going counterclockwise
  {
    pErr = pErr-360;
  }
  else if(pErr < -180) //if the compass passes over 0 and 360 going clockwise
  {
    pErr = 360-pErr;
  }
  */
  float pGain = pErr * kp;
  //Serial.print("pErr: "); Serial.println(pErr);

  //D - Derivative
  float dErr = pErr - lastErr;
  float dGain = dErr * kd;

  //total
  float totalRotGain = pGain + dGain;

  if(PWR != 0 && currentVal != -1) //the robot is driving
  {
    totalRotGain = constrain(totalRotGain,-60, 60);
    if(totalRotGain > 10 || totalRotGain < -10) //20
    {  
      //digitalWrite(LED2, LOW);
      //digitalWrite(LED4, HIGH);
    }
    else
    {
      //digitalWrite(LED2, HIGH);
      //digitalWrite(LED4, LOW);
    }
    //adjust heading based on rotational error
    int correctedDir = dir-pErr;

    drive(correctedDir,-totalRotGain, PWR);
  }
  else //the robot is stopped
  {
    if((totalRotGain > 24 || totalRotGain < -24) && currentVal != -1) //20
    {  
      Serial.print(" Total Gain:");
      Serial.println(totalRotGain);
      totalRotGain = constrain(totalRotGain,-60,60);
      //digitalWrite(LED2, LOW);
      //digitalWrite(LED4, HIGH);
      drive(dir,-totalRotGain, 0);
    }
    else
    {
      Serial.print(" Total Gain:");
      Serial.println(totalRotGain);
      //digitalWrite(LED2, HIGH);
      //digitalWrite(LED4, LOW);
      
       motor(M1, 0);
       motor(M2, 0);
       motor(M3, 0);
       motor(M4, 0);
       //brakeMotors();
    }
  }
  //store the last pErr
  lastErr = pErr; 

}

#define POSITIVE true
#define NEGATIVE false
void drive(int dir, int rot, int PWR)
{
  //In order to make the math for x-configuration omnidrive simpler, the axes are shifted 45 degrees
  //Instead of using an x-axis for left/right and a y-axis for forward/back,
  //an alpha axis that points forward-right/backward-left and a beta axis that points forward-left/backward right are used.

  int alphaVec = 0; //this vector points forward-right
  int betaVec = 0; //this vector points forward-left

  dir = dir - 45; //shifting axis
  if(dir < 0){
    dir = dir + 360;
  }
  else if(dir >= 360){
    dir = dir - 360;
  }

  int scale = 10000; //creates a range between -10,000 and 10,000 for alphaVec and betaVec

  float dirRad = (float)dir * 71 / 4068; //convert dir to radians
  
  //calculate alpha and beta vectors
  alphaVec = (float)(scale*cos(dirRad));
  betaVec = (float)(scale*sin(dirRad));
  
  //record the signs of alphaVec and betaVec for later use
  bool alphaSign; 
  if(alphaVec >= 0)
  {
    alphaSign = POSITIVE;
  }
  else
  {
    alphaSign = NEGATIVE;
  }
  bool betaSign;
  if(betaVec >= 0)
  {
    betaSign = POSITIVE;
  }
  else
  {
    betaSign = NEGATIVE;
  }
  //remove the signs from alpha and betaVec
  alphaVec = abs(alphaVec);
  betaVec = abs(betaVec);
  //Create a maximum value based on the larger of the two vectors (in magnitude, without signs)
  int max;
  if(alphaVec >= betaVec)
  {
    max = alphaVec;
  }
  else
  {
    max = betaVec;
  }
  //map vectors using the max value obtained and the given motor PWR
  alphaVec = map(alphaVec, 0,max, 0,PWR);
  betaVec = map(betaVec, 0,max, 0,PWR);
  //put the signs back
  if(alphaSign == NEGATIVE)
  {
    alphaVec = -(alphaVec);
  }
  if(betaSign == NEGATIVE)
  {
    betaVec = -(betaVec);
  }
  //Serial.print("AlphaVec: "); Serial.println(alphaVec);
  //Serial.print("BetaVec: "); Serial.println(betaVec);
  //Serial.print("AlphaSign: "); Serial.println(alphaSign);
  //Serial.print("BetaSign: "); Serial.println(betaSign);
  //Serial.print("Rot: "); Serial.println(rot);

  //Send Commands to motors
  motor(M1, betaVec + rot);
  motor(M2, alphaVec + rot);
  motor(M3, betaVec - rot);
  motor(M4, alphaVec - rot);
}
void motor(int motorID, int PWR) //produces low level signals to each individual motor drivers
{
  bool motorDirection;
  //Checking for valid PWR value
  if(PWR <= 255 && PWR >= 0){
    motorDirection = true;
  }
  else if(PWR >= -255 && PWR < 0){
    motorDirection = false;
    PWR = -(PWR);
  }
  else{
    Serial.println("Invalid motor power");
    return; //error 
  }
  //Selecting a motor to run
  if(motorID == M1)
  {
    if(motorDirection == true){
        digitalWrite(M1_IN1, LOW);
        digitalWrite(M1_IN2, HIGH);
    }    
    else{
        digitalWrite(M1_IN1, HIGH);
        digitalWrite(M1_IN2, LOW);
    }
    analogWrite(M1_PWM, PWR);
  }
  else if(motorID == M2)
  {
    if(motorDirection == true){
      digitalWrite(M2_IN1, LOW);
        digitalWrite(M2_IN2, HIGH);
    }    
    else{
        digitalWrite(M2_IN1, HIGH);
        digitalWrite(M2_IN2, LOW);
    }
    analogWrite(M2_PWM, PWR);
  }
  else if(motorID == M3)
  {
    if(motorDirection == true){
      digitalWrite(M3_IN1, LOW);
        digitalWrite(M3_IN2, HIGH);
    }    
    else{
        digitalWrite(M3_IN1, HIGH);
        digitalWrite(M3_IN2, LOW);
    }
    analogWrite(M3_PWM, PWR);
  }
  else if(motorID == M4)
  {
    if(motorDirection == true){
      digitalWrite(M4_IN1, LOW);
        digitalWrite(M4_IN2, HIGH);
    }    
    else{
        digitalWrite(M4_IN1, HIGH);
        digitalWrite(M4_IN2, LOW);
    }
    analogWrite(M4_PWM, PWR);
  }
  else if(motorID == M5)
  {
    if(motorDirection == true){
      digitalWrite(M5_IN1, LOW);
        digitalWrite(M5_IN2, HIGH);
    }    
    else{
        digitalWrite(M5_IN1, HIGH);
        digitalWrite(M5_IN2, LOW);
    }
    analogWrite(M5_PWM, PWR);
  }
}
void brakeMotors() //Stop all drive motors
{
  digitalWrite(M1_IN1, LOW);
  digitalWrite(M1_IN2, LOW);
  analogWrite(M1_PWM, 0); 

  digitalWrite(M2_IN1, LOW);
  digitalWrite(M2_IN2, LOW);
  analogWrite(M2_PWM, 0);   
  
  digitalWrite(M3_IN1, LOW);
  digitalWrite(M3_IN2, LOW);
  analogWrite(M3_PWM, 0); 

  digitalWrite(M4_IN1, LOW);
  digitalWrite(M4_IN2, LOW);
  analogWrite(M4_PWM, 0); 
}

//////////////
//Functions for Reading Sensors//
//////////////
////Reading Compass Sensor
float readCompass()
{
  Serial1.write(compassCommand);
  delay(10);
  if(Serial1.available())
  {  
    byte byte1 = Serial1.read();
    byte byte2 = Serial1.read();
    
    word compassVal = word(byte1,byte2);
    //Serial.println(compassVal);
    float processedVal =  (float) compassVal/10 ;
    
    //Serial.print("CMPS: "); Serial.println(processedVal);
    return processedVal;
  }
  else
  {
    Serial.println("compass not available");
    return -1;
  }
}
////Reading Light Gate Sensor
bool readLightGate()
{
  int lightGateVal = readGSMux(LG);
  Serial.print("Raw LG: "); Serial.println(lightGateVal);
  if(lightGateVal > LG_THRESHOLD)
  {
    return true;
  }
  else
  {
    return false;
  }
}
////Reading IR Sensor Array
const float IR_sin[20] = {sin(1.41372), sin(1.09956), sin(0.785398), sin(0.471239), sin(0.15708), 
						  sin(6.12611), sin(5.81195), sin(5.49779),  sin(5.18363),  sin(4.86947), 
						  sin(4.55531), sin(4.24115), sin(3.92699),  sin(3.61283),  sin(3.29867),
						  sin(2.98451), sin(2.67035), sin(2.35619),  sin(2.04204),  sin(1.72788) };
const float IR_cos[20] = {cos(1.41372), cos(1.09956), cos(0.785398), cos(0.471239), cos(0.15708), 
						  cos(6.12611), cos(5.81195), cos(5.49779),  cos(5.18363),  cos(4.86947), 
						  cos(4.55531), cos(4.24115), cos(3.92699),  cos(3.61283),  cos(3.29867),
						  cos(2.98451), cos(2.67035), cos(2.35619),  cos(2.04204),  cos(1.72788) };
const float IR_angle[20] = {81,63,45,27,9,351,333,315,297,279,261,243,225,207,189,171,153,135,117,99}; //IR angles in degrees

int IR_unsorted[20]; //raw sensor data from 20 IR sensors
int IR_sorted[20]; 
int IR_ID[20];



float readIR()
{
  //Reading from the 20 IR recievers
  IR_unsorted[0] = readIRMux(IR1); //Serial.print("IR1: "); Serial.println(IR_unsorted[0]);
  IR_unsorted[1] = readIRMux(IR2); //Serial.print("IR2: "); Serial.println(IR_unsorted[1]);
  IR_unsorted[2] = readIRMux(IR3); //Serial.print("IR3: "); Serial.println(IR_unsorted[2]);
  IR_unsorted[3] = readIRMux(IR4); //Serial.print("IR4: "); Serial.println(IR_unsorted[3]);
  IR_unsorted[4] = readIRMux(IR5); //Serial.print("IR5: "); Serial.println(IR_unsorted[4]);
  IR_unsorted[5] = readIRMux(IR6); //Serial.print("IR6: "); Serial.println(IR_unsorted[5]);
  IR_unsorted[6] = readIRMux(IR7); //Serial.print("IR7: "); Serial.println(IR_unsorted[6]);
  IR_unsorted[7] = readIRMux(IR8); //Serial.print("IR8: "); Serial.println(IR_unsorted[7]);
  IR_unsorted[8] = readIRMux(IR9); //Serial.print("IR9: "); Serial.println(IR_unsorted[8]);
  IR_unsorted[9] = readIRMux(IR10); //Serial.print("IR10: "); Serial.println(IR_unsorted[9]);
  IR_unsorted[10] = readIRMux(IR11); //Serial.print("IR11: "); Serial.println(IR_unsorted[10]);
  IR_unsorted[11] = readIRMux(IR12); //Serial.print("IR12: "); Serial.println(IR_unsorted[11]);
  IR_unsorted[12] = readIRMux(IR13); //Serial.print("IR13: "); Serial.println(IR_unsorted[12]);
  IR_unsorted[13] = readIRMux(IR14); //Serial.print("IR14: "); Serial.println(IR_unsorted[13]);
  IR_unsorted[14] = readIRMux(IR15); //Serial.print("IR15: "); Serial.println(IR_unsorted[14]);
  IR_unsorted[15] = readIRMux(IR16); //Serial.print("IR16: "); Serial.println(IR_unsorted[15]);
  IR_unsorted[16] = analogRead(IR17); //Serial.print("IR17: "); Serial.println(IR_unsorted[16]);
  IR_unsorted[17] = analogRead(IR18); //Serial.print("IR18: "); Serial.println(IR_unsorted[17]);
  IR_unsorted[18] = analogRead(IR19); //Serial.print("IR19: "); Serial.println(IR_unsorted[18]);
  IR_unsorted[19] = analogRead(IR20); //Serial.print("IR20: "); Serial.println(IR_unsorted[19]);
  //set IR IDs
  for(int i = 0; i < 20; i++){
   IR_ID[i] = i;
  }
  //copy data to the sorted array before sorting
  for(int i = 0; i < 20; i++){
    IR_sorted[i] = IR_unsorted[i];
  }
  //perform insertion sort
  insertSort(IR_sorted, IR_ID); 

  Serial.print("IR Sorted 0: "); Serial.println(IR_sorted[0]);
  //Update Ball Proximity
  if(IR_sorted[0] < IR_min || IR_sorted[0] > IR_max) //if none of the sensors see the ball
  {
    ballProximity = 0;
    return -1;
  }
  else if(IR_sorted[0] > IR_min && IR_sorted[0] < 40) //Ball Very Close
  {
    ballProximity = 4; 
  }
  else if(IR_sorted[0] >= 40 && IR_sorted[0] < 65) //Ball Close
  {
    ballProximity = 3; 
  }
  else if(IR_sorted[0] >= 65 && IR_sorted[0] < 100) //Ball Moderate
  {
    ballProximity = 2; 
  }
  else  //Ball Far Away
  {
    ballProximity = 1; 
  }

  float calculatedAngle = IR_angle[IR_ID[0]];

  //Serial.print("Raw IR: "); Serial.println(calculatedAngle);
  //tie checker
  /*
  if(IR_sorted[0] == IR_sorted[1])
  {
    //Serial.println("Tie!");
    calculatedAngle = angleAvg(IR_angle[IR_ID[0]], IR_angle[IR_ID[1]]);
  }
  */
  //Smoothing
  /*
  if(lastBallAngle != -1)
  {
    calculatedAngle = angleAvg(calculatedAngle, lastBallAngle);
  }
  lastBallAngle = calculatedAngle;
  */
  return calculatedAngle; 
}
void readGroundSensor()
{
  for(int i=0; i < 15; i++)
  {
    groundSensor[i] = readMux(i);
  }
}
//////////////
//Low Level Sensor Functions//
//////////////
int readIRMux(int channel) //selects the given channel on the IR multiplexor and reads the analog signal
{
  switch(channel){ //Uses truth table in multiplexor datasheet for selecting channels: http://www.mouser.com/ds/2/405/cd74hc4067-441121.pdf
    case 0:
      digitalWrite(IR_S0,LOW); digitalWrite(IR_S1,LOW); digitalWrite(IR_S2,LOW); digitalWrite(IR_S3,LOW);
      break;
    case 1:
      digitalWrite(IR_S0,HIGH); digitalWrite(IR_S1,LOW); digitalWrite(IR_S2,LOW); digitalWrite(IR_S3,LOW);
      break;
    case 2:
      digitalWrite(IR_S0,LOW); digitalWrite(IR_S1,HIGH); digitalWrite(IR_S2,LOW); digitalWrite(IR_S3,LOW);
      break;
    case 3:
      digitalWrite(IR_S0,HIGH); digitalWrite(IR_S1,HIGH); digitalWrite(IR_S2,LOW); digitalWrite(IR_S3,LOW);
    break;
    case 4:
      digitalWrite(IR_S0,LOW); digitalWrite(IR_S1,LOW); digitalWrite(IR_S2,HIGH); digitalWrite(IR_S3,LOW);
    break;
    case 5:
      digitalWrite(IR_S0,HIGH); digitalWrite(IR_S1,LOW); digitalWrite(IR_S2,HIGH); digitalWrite(IR_S3,LOW);
    break;
    case 6:
      digitalWrite(IR_S0,LOW); digitalWrite(IR_S1,HIGH); digitalWrite(IR_S2,HIGH); digitalWrite(IR_S3,LOW);
    break;
    case 7:
      digitalWrite(IR_S0,HIGH); digitalWrite(IR_S1,HIGH); digitalWrite(IR_S2,HIGH); digitalWrite(IR_S3,LOW);
    break;
    case 8:
      digitalWrite(IR_S0,LOW); digitalWrite(IR_S1,LOW); digitalWrite(IR_S2,LOW); digitalWrite(IR_S3,HIGH);
    break;
    case 9:
      digitalWrite(IR_S0,HIGH); digitalWrite(IR_S1,LOW); digitalWrite(IR_S2,LOW); digitalWrite(IR_S3,HIGH);
    break;
    case 10:
      digitalWrite(IR_S0,LOW); digitalWrite(IR_S1,HIGH); digitalWrite(IR_S2,LOW); digitalWrite(IR_S3,HIGH);
    break;
    case 11:
      digitalWrite(IR_S0,HIGH); digitalWrite(IR_S1,HIGH); digitalWrite(IR_S2,LOW); digitalWrite(IR_S3,HIGH);
    break;
    case 12:
      digitalWrite(IR_S0,LOW); digitalWrite(IR_S1,LOW); digitalWrite(IR_S2,HIGH); digitalWrite(IR_S3,HIGH);
    break;
    case 13:
      digitalWrite(IR_S0,HIGH); digitalWrite(IR_S1,LOW); digitalWrite(IR_S2,HIGH); digitalWrite(IR_S3,HIGH);
    break;
    case 14:
      digitalWrite(IR_S0,LOW); digitalWrite(IR_S1,HIGH); digitalWrite(IR_S2,HIGH); digitalWrite(IR_S3,HIGH);
    break;
    case 15:
      digitalWrite(IR_S0,HIGH); digitalWrite(IR_S1,HIGH); digitalWrite(IR_S2,HIGH); digitalWrite(IR_S3,HIGH);
    break;
    }
  delayMicroseconds(2); //Give time for the multiplexor to switch - can change to 1 microsecond
  int analogInput = analogRead(IR_ANI);
  return analogInput;
}
int readGSMux(int channel) //selects the given channel on the ground sensor multiplexor and reads the analog signal
{
  switch(channel){ //Uses truth table in multiplexor datasheet for selecting channels: http://www.mouser.com/ds/2/405/cd74hc4067-441121.pdf
    case 0:
      digitalWrite(GS_S0,LOW); digitalWrite(GS_S1,LOW); digitalWrite(GS_S2,LOW); digitalWrite(GS_S3,LOW);
      break;
    case 1:
      digitalWrite(GS_S0,HIGH); digitalWrite(GS_S1,LOW); digitalWrite(GS_S2,LOW); digitalWrite(GS_S3,LOW);
      break;
    case 2:
      digitalWrite(GS_S0,LOW); digitalWrite(GS_S1,HIGH); digitalWrite(GS_S2,LOW); digitalWrite(GS_S3,LOW);
      break;
    case 3:
      digitalWrite(GS_S0,HIGH); digitalWrite(GS_S1,HIGH); digitalWrite(GS_S2,LOW); digitalWrite(GS_S3,LOW);
    break;
    case 4:
      digitalWrite(GS_S0,LOW); digitalWrite(GS_S1,LOW); digitalWrite(GS_S2,HIGH); digitalWrite(GS_S3,LOW);
    break;
    case 5:
      digitalWrite(GS_S0,HIGH); digitalWrite(GS_S1,LOW); digitalWrite(GS_S2,HIGH); digitalWrite(GS_S3,LOW);
    break;
    case 6:
      digitalWrite(GS_S0,LOW); digitalWrite(GS_S1,HIGH); digitalWrite(GS_S2,HIGH); digitalWrite(GS_S3,LOW);
    break;
    case 7:
      digitalWrite(GS_S0,HIGH); digitalWrite(GS_S1,HIGH); digitalWrite(GS_S2,HIGH); digitalWrite(GS_S3,LOW);
    break;
    case 8:
      digitalWrite(GS_S0,LOW); digitalWrite(GS_S1,LOW); digitalWrite(GS_S2,LOW); digitalWrite(GS_S3,HIGH);
    break;
    case 9:
      digitalWrite(GS_S0,HIGH); digitalWrite(GS_S1,LOW); digitalWrite(GS_S2,LOW); digitalWrite(GS_S3,HIGH);
    break;
    case 10:
      digitalWrite(GS_S0,LOW); digitalWrite(GS_S1,HIGH); digitalWrite(GS_S2,LOW); digitalWrite(GS_S3,HIGH);
    break;
    case 11:
      digitalWrite(GS_S0,HIGH); digitalWrite(GS_S1,HIGH); digitalWrite(GS_S2,LOW); digitalWrite(GS_S3,HIGH);
    break;
    case 12:
      digitalWrite(GS_S0,LOW); digitalWrite(GS_S1,LOW); digitalWrite(GS_S2,HIGH); digitalWrite(GS_S3,HIGH);
    break;
    case 13:
      digitalWrite(GS_S0,HIGH); digitalWrite(GS_S1,LOW); digitalWrite(GS_S2,HIGH); digitalWrite(GS_S3,HIGH);
    break;
    case 14:
      digitalWrite(GS_S0,LOW); digitalWrite(GS_S1,HIGH); digitalWrite(GS_S2,HIGH); digitalWrite(GS_S3,HIGH);
    break;
    case 15:
      digitalWrite(GS_S0,HIGH); digitalWrite(GS_S1,HIGH); digitalWrite(GS_S2,HIGH); digitalWrite(GS_S3,HIGH);
    break;
    }
  delayMicroseconds(2); //Give time for the multiplexor to switch - can change to 1 microsecond
  int analogInput = analogRead(GS_ANI);
  return analogInput;
}
//////////////
//LCD Functions//
//////////////

void LCD_SendBuffer()
{
  lcd.clear();
  digitalWrite(LED3, HIGH);
  lcd.setCursor(0,0);
  lcd.print(lcdLine1);
  lcd.setCursor(0,1);
  lcd.print(lcdLine2);
  digitalWrite(LED3, LOW);
}

//////////////
//Math/Misc Functions//
//////////////
float angleDif(float x, float y) //used to calculate the difference between actual heading and the target heading in drivePID()
{
  float radianDif = ((x-y)*71)/4068;
  float a = (float) atan2(sin(radianDif), cos(radianDif));
  a =  (a * 4068) / 71; //to degrees 

  return a;
}
float angleAvg(float x, float y)
{
  //Serial.print("Averaging "); Serial.print(x); Serial.print(" and "); Serial.println(y);
  x = x * 71 / 4068; //Convert to radians
  y = y * 71 / 4068; //Convert to radians
  float xVec = cos(x) + cos(y);
  float yVec = sin(x) + sin(y);
  float a = (float) atan2(yVec,xVec)* (4068 / 71);
  return a;
}
void insertSort(int array[], int arrayKey[])
{
  //array[] holds the data to be sorted numerically, arrayKey holds the original position of each value
    for (int i = 1 ; i < 20; i++) 
    {
      int j = i;
      while (j > 0 && array[j] < array[j-1]) {
        //perform swap - value
        int savedVal = array[j]; //keeps the value in index j so it can be used to complete the swap after it is overode
        array[j]  = array[j-1];
        array[j-1] = savedVal;
        //perform swap - key
        int savedKey = arrayKey[j]; //keeps the position in index j so it can be used to complete the swap after it is overode
        arrayKey[j] = arrayKey[j-1];
        arrayKey[j-1] = savedKey;
        j--;
      }
    }
}