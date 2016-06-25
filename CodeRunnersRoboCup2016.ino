#include <HID.h>


#include "Robot1Config.h"
//CodeRunnersRoboCup.ino

#include "Pinout.h" 
#include "StateMachine.h"
int state = START_FORWARD;


float fieldNorth;
float rotOffset = 0; //Change this to point the robot at a different angle relative to fieldNorth
boolean compassWorking = true;

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
long startAvoidOutTime;


 bool driveAbsoluteMode = true; //Adjust drive angle based on rotational error
 bool lineLeft = false;
 bool lineRight = false;
 bool lineFront = false;
 bool lineBack = false;

 bool blackLineLeft = false;
 bool blackLineRight = false;
 bool blackLineFront = false;
 bool blackLineBack = false;

 //Sonar Localization Regions
 bool leftOfGoal = false;
 bool rightOfGoal = false;
 bool goalDetected = false;

 bool kickPhase = false;
 long startKickPhase;

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

  delay(100);
  fieldNorth = readCompass();

  lastLCDUpdateTime = millis();
}

void loop() 
{
  long startLoopTime = millis();
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
      forwardAvoidOutOfBounds();
      break; 
    //Defense Strategic States    
    case DEFENSE_MOVE_AND_BLOCK:
      defenseMoveAndBlock();
      break; 
    case DEFENSE_HAS_BALL:
      defenseHasBall();
      break;
    case DEFENSE_AVOID_OUTOFBOX:
      defenseAvoidOutOfBox();
      break; 
    case DEFENSE_AVOID_OUTOFBOUNDS:
      defenseAvoidOutOfBounds();
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
    case US_TEST:
      ultrasonicTest();
      break;
    case KICKER_TEST:
      kickerTest();
      break;
    }


  long currentTime = millis();
  //Refresh the Compass every 8ms
  /*
  if(currentTime - lastCompassUpdateTime > 8)
  {
    digitalWrite(LED3, HIGH);
    refreshCompass();
    digitalWrite(LED3,LOW);
  }
  */
  //Refresh the LCD at 5Hz
  if(currentTime - lastLCDUpdateTime > 200)
  {
    LCD_SendBuffer();
    lastLCDUpdateTime = millis();
  }
  
  //Timer
  /*
  if((state == FORWARD_BALL_PURSUIT || state == FORWARD_HAS_BALL) && ((currentTime - startTime) > 4000))
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
  if(state == DEFENSE_MOVE_AND_BLOCK && ((currentTime - startTime) > 25000))
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
  */
  long endLoopTime = millis();
  long loopDuration = endLoopTime - startLoopTime;

  if((state == FORWARD_BALL_PURSUIT || state == FORWARD_HAS_BALL) || state == FORWARD_AVOID_OUTOFBOUNDS)
  {
    //lcdLine2 = String(loopDuration);
  }
  
  //Serial.print("loopDuration: ");
  //Serial.println(loopDuration);
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

  //Keep LEDs off
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);
  digitalWrite(LED3, LOW);
  digitalWrite(LED4, LOW);
  //Make sure the kicker is off
  digitalWrite(KICKER, LOW);

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

  //Keep LEDs off
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);
  digitalWrite(LED3, LOW);
  digitalWrite(LED4, LOW);
  //Make sure the kicker is off
  digitalWrite(KICKER, LOW);

  //Reading Buttons
  if(digitalRead(B3) == HIGH)
  {
    delay(500);
    startTime = millis();
    fieldNorth = readCompass();

    state = DEFENSE_MOVE_AND_BLOCK; //Change state to the default defense state
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

  //Keep LEDs off
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);
  digitalWrite(LED3, LOW);
  digitalWrite(LED4, LOW);
  //Make sure the kicker is off
  digitalWrite(KICKER, LOW);

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

  //Keep LEDs off
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);
  digitalWrite(LED3, LOW);
  digitalWrite(LED4, LOW);
  //Make sure the kicker is off
  digitalWrite(KICKER, LOW);

  float compassHeading = readCompass();
  //lcd.setCursor(0,1);
  //lcd.print(compassHeading);
  if(compassWorking == true)
  {
    lcdLine2 =  String(compassHeading);
  }
  else
  {
    lcdLine2 = "NotFound";
  }
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
}
void compassCalibration()
{
  lcdLine1 = "CMPS_Cal";

  lcdLine2 =  "Start   ";

  //Keep LEDs off
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);
  digitalWrite(LED3, LOW);
  digitalWrite(LED4, LOW);
  //Make sure the kicker is off
  digitalWrite(KICKER, LOW);

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

    delay(500);
    //spin
    /*
    for(int i=0; i<10; i++)
    {
      motor(M1, 36);
      motor(M2, 36);
      motor(M3, -36);
      motor(M4, -36);
      delay(400);
      motor(M1, 0);
      motor(M2, 0);
      motor(M3, 0);
      motor(M4, 0);
      delay(400);
    }
    motor(M1, 0);
    motor(M2, 0);
    motor(M3, 0);
    motor(M4, 0);
    delay(500);
    for(int i=0; i<10; i++)
    {
      motor(M1, -36);
      motor(M2, -36);
      motor(M3, 36);
      motor(M4, 36);
      delay(400);
      motor(M1, 0);
      motor(M2, 0);
      motor(M3, 0);
      motor(M4, 0);
      delay(400);
    }
    motor(M1, 0);
    motor(M2, 0);
    motor(M3, 0);
    motor(M4, 0);
    */
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
bool GS_TestMode = 0; //0 for testing white lines
void groundSensorTest()
{
  lcdLine1 = "GS Test ";

  //Make sure the kicker is off
  digitalWrite(KICKER, LOW);

  readGroundSensor();
  if(GS_TestMode == 0)
  {
    lcdLine2 = "White";
    if(groundSensor[GS0] > GS_THRESHOLD_WHITE || groundSensor[GS1] > GS_THRESHOLD_WHITE || groundSensor[GS2] > GS_THRESHOLD_WHITE)
    {
      digitalWrite(LED1, HIGH);
    }
    else{
      digitalWrite(LED1, LOW);
    }
    if(groundSensor[GS3] > GS_THRESHOLD_WHITE || groundSensor[GS4] > GS_THRESHOLD_WHITE || groundSensor[GS5] > GS_THRESHOLD_WHITE || groundSensor[GS6] > GS_THRESHOLD_WHITE)
    {
      digitalWrite(LED2, HIGH);
    }
    else{
      digitalWrite(LED2, LOW);
    }
    if(groundSensor[GS7] > GS_THRESHOLD_WHITE || groundSensor[GS8] > GS_THRESHOLD_WHITE || groundSensor[GS9] > GS_THRESHOLD_WHITE || groundSensor[GS10] > GS_THRESHOLD_WHITE)
    {
      digitalWrite(LED4, HIGH);
    }
    else{
      digitalWrite(LED4, LOW);
    }
    if(groundSensor[GS11] > GS_THRESHOLD_WHITE || groundSensor[GS12] > GS_THRESHOLD_WHITE || groundSensor[GS13] > GS_THRESHOLD_WHITE || groundSensor[GS14] > GS_THRESHOLD_WHITE)
    {
      digitalWrite(LED3, HIGH);
    }
    else{
      digitalWrite(LED3, LOW);
    }
  }
  else if(GS_TestMode == 1)
  {
    lcdLine2 = "Black";
    if(groundSensor[GS0] < GS_THRESHOLD_BLACK || groundSensor[GS1] < GS_THRESHOLD_BLACK || groundSensor[GS2] < GS_THRESHOLD_BLACK)
    {
      digitalWrite(LED1, HIGH);
       if(groundSensor[GS0] < GS_THRESHOLD_BLACK)
       {
          Serial.print("Black Line at GS0: ");
          Serial.println(groundSensor[GS0]);
       }
       if(groundSensor[GS1] < GS_THRESHOLD_BLACK)
       {
          Serial.print("Black Line at GS1: ");
          Serial.println(groundSensor[GS1]);
       }
      if(groundSensor[GS2] < GS_THRESHOLD_BLACK)
       {
          Serial.print("Black Line at GS2: ");
          Serial.println(groundSensor[GS2]);
       }
    }
    else
    {
      digitalWrite(LED1, LOW);
    }
    if(groundSensor[GS3] < GS_THRESHOLD_BLACK || groundSensor[GS4] < GS_THRESHOLD_BLACK || groundSensor[GS5] < GS_THRESHOLD_BLACK)// || groundSensor[GS6] < GS_THRESHOLD_BLACK)
    {
      digitalWrite(LED2, HIGH);
      if(groundSensor[GS3] < GS_THRESHOLD_BLACK)
       {
          Serial.print("Black Line at GS3: ");
          Serial.println(groundSensor[GS3]);
       }
       if(groundSensor[GS4] < GS_THRESHOLD_BLACK)
       {
          Serial.print("Black Line at GS4: ");
          Serial.println(groundSensor[GS4]);
       }
       if(groundSensor[GS5] < GS_THRESHOLD_BLACK)
       {
          Serial.print("Black Line at GS5: ");
          Serial.println(groundSensor[GS5]);
       }
       /*
      if(groundSensor[GS6] < GS_THRESHOLD_BLACK)
       {
          Serial.print("Black Line at GS6: ");
          Serial.println(groundSensor[GS6]);
       }
       */
    }
    else{
      digitalWrite(LED2, LOW);
    }
    if(groundSensor[GS7] < GS_THRESHOLD_BLACK || groundSensor[GS8] < GS_THRESHOLD_BLACK || groundSensor[GS9] < GS_THRESHOLD_BLACK || groundSensor[GS10] < GS_THRESHOLD_BLACK)
    {
      digitalWrite(LED4, HIGH);
       if(groundSensor[GS7] < GS_THRESHOLD_BLACK)
       {
          Serial.print("Black Line at GS7: ");
          Serial.println(groundSensor[GS7]);
       }
       if(groundSensor[GS8] < GS_THRESHOLD_BLACK)
       {
          Serial.print("Black Line at GS8: ");
          Serial.println(groundSensor[GS8]);
       }
       if(groundSensor[GS9] < GS_THRESHOLD_BLACK)
       {
          Serial.print("Black Line at GS9: ");
          Serial.println(groundSensor[GS9]);
       }
       if(groundSensor[GS10] < GS_THRESHOLD_BLACK)
       {
          Serial.print("Black Line at GS10: ");
          Serial.println(groundSensor[GS0]);
       }
    }
    else{
      digitalWrite(LED4, LOW);
    }
    if(/*groundSensor[GS11] < GS_THRESHOLD_BLACK ||*/ groundSensor[GS12] < GS_THRESHOLD_BLACK || groundSensor[GS13] < GS_THRESHOLD_BLACK || groundSensor[GS14] < GS_THRESHOLD_BLACK)
    {
      digitalWrite(LED3, HIGH);
      /*
       if(groundSensor[GS11] < GS_THRESHOLD_BLACK)
       {
          Serial.print("Black Line at GS11: ");
          Serial.println(groundSensor[GS11]);
       }
       */
       if(groundSensor[GS12] < GS_THRESHOLD_BLACK)
       {
          Serial.print("Black Line at GS12: ");
          Serial.println(groundSensor[GS12]);
       }
       if(groundSensor[GS13] < GS_THRESHOLD_BLACK)
       {
          Serial.print("Black Line at GS13: ");
          Serial.println(groundSensor[GS13]);
       }
       if(groundSensor[GS14] < GS_THRESHOLD_BLACK)
       {
          Serial.print("Black Line at GS14: ");
          Serial.println(groundSensor[GS14]);
       }
    }
    else{
      digitalWrite(LED3, LOW);
    }
  }
  //Reading Buttons
  if(digitalRead(B3) == HIGH)
  {
    //toggle GS Test Mode
    if(GS_TestMode == 0)
    {
      GS_TestMode = 1;
    }
    else
    {
      GS_TestMode = 0;
    }
    delay(500);
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

  //Keep LEDs off
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);
  digitalWrite(LED3, LOW);
  digitalWrite(LED4, LOW);
  //Make sure the kicker is off
  digitalWrite(KICKER, LOW);

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
  lcdLine1 =  "LG Test ";

  //Make sure the kicker is off
  digitalWrite(KICKER, LOW);

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
    state = US_TEST;
    delay(500);
  }
}
int US_beingTested = 1; //This keeps track of which ultrasonic sensor is currently being tested
void ultrasonicTest()
{
  //Make sure the kicker is off
  digitalWrite(KICKER, LOW);

  if(readSonar(US4) > 111) //was 121
  {
    digitalWrite(LED2, HIGH);
  }
  else
  {
    digitalWrite(LED2, LOW);
  }
  if(readSonar(US6) > 111) //was 121
  {
    digitalWrite(LED4, HIGH);
  }
  else
  {
    digitalWrite(LED4, LOW);
  }
  //Goal Detection code
  int sonar1 = readSonar(US1);
  int sonar2 = readSonar(US2);
  int sonar3 = readSonar(US3);
  if(sonar1 > sonar2 && sonar1 > sonar3) //sonar1 is the highest value
  {
    if(sonar1 < 60) 
    {
      digitalWrite(LED1, HIGH);
    }
    else
    {
      digitalWrite(LED1, LOW);
    }
  }
  else if(sonar2 > sonar1 && sonar2 > sonar3) //sonar2 is the highest value
  {
    if(sonar2 < 60) 
    {
      digitalWrite(LED1, HIGH);
    }
    else
    {
      digitalWrite(LED1, LOW);
    }
  }
  else if(sonar3 > sonar1 && sonar3 > sonar2) //sonar3 is the highest value
  {
    if(sonar3 < 60) 
    {
      digitalWrite(LED1, HIGH);
    }
    else
    {
      digitalWrite(LED1, LOW);
    }
  }
  if(US_beingTested == 1)
  {
    lcdLine1 =  "US1 Test";
    lcdLine2 = String(readSonar(US1));
    Serial.print("US1: "); Serial.println(readSonar(US1));
  }
  else if(US_beingTested == 2)
  {
    lcdLine1 =  "US2 Test";
    lcdLine2 = String(readSonar(US2));
    Serial.print("US2: "); Serial.println(readSonar(US2));
  }
  else if(US_beingTested == 3)
  {
    lcdLine1 =  "US3 Test";
    lcdLine2 = String(readSonar(US3));
    Serial.print("US3: "); Serial.println(readSonar(US3));
  }
  else if(US_beingTested == 4)
  {
    lcdLine1 =  "US4 Test";
    lcdLine2 = String(readSonar(US4));
    Serial.print("US4: "); Serial.println(readSonar(US4));
  }
  else if(US_beingTested == 5)
  {
    lcdLine1 =  "US5 Test";
    lcdLine2 = String(readSonar(US5));
    Serial.print("US5: "); Serial.println(readSonar(US5));
  }
  else if(US_beingTested == 6)
  {
    lcdLine1 =  "US6 Test";
    lcdLine2 = String(readSonar(US6));
    Serial.print("US6: "); Serial.println(readSonar(US6));
  }
  if(digitalRead(B3) == HIGH) //Cycle through the US sensors
  {
    if(US_beingTested == 6)
    {
      US_beingTested = 1;
    }
    else
    {
      US_beingTested++;
    }
    delay(500);
  }
  else if(digitalRead(B1) == HIGH)
  {
    state = LG_TEST;
    //Keep LEDs off
    digitalWrite(LED1, LOW);
    digitalWrite(LED2, LOW);
    digitalWrite(LED3, LOW);
    digitalWrite(LED4, LOW);
    delay(500);
  }
  else if(digitalRead(B2) == HIGH)
  {
    state = KICKER_TEST;
    //Keep LEDs off
    digitalWrite(LED1, LOW);
    digitalWrite(LED2, LOW);
    digitalWrite(LED3, LOW);
    digitalWrite(LED4, LOW);
    delay(500);
  }
}
void kickerTest()
{
  //lcd.setCursor(0,0);
  //lcd.print("Kicker  ");
  lcdLine1 = "Kicker  ";
  lcd.setCursor(0,1);
  lcd.print("Ready   ");
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
    state = US_TEST;
    //Make sure the kicker is off
    digitalWrite(KICKER, LOW);
    //Keep LEDs off
    digitalWrite(LED1, LOW);
    digitalWrite(LED2, LOW);
    digitalWrite(LED3, LOW);
    digitalWrite(LED4, LOW);
    delay(500);
  }
  else if(digitalRead(B2) == HIGH)
  {
    state = START_FORWARD;
    //Make sure the kicker is off
    digitalWrite(KICKER, LOW);
    //Keep LEDs off
    digitalWrite(LED1, LOW);
    digitalWrite(LED2, LOW);
    digitalWrite(LED3, LOW);
    digitalWrite(LED4, LOW);
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

  //Make Sure Kicker is off
  digitalWrite(KICKER, LOW);
  rotOffset = 0;

  //Turn on absolute mode
  driveAbsoluteMode = true;
  
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
  
  //Ball Pursuit - Chooses speed and compensation modifier based on proximity to ball
  int pursuitSpeed = 0;
  float compensationModifier = 0.375; //this was the default
  if(ballProximity == 4)
  {
    pursuitSpeed = 140; //160
    compensationModifier = 0.5;
  }
  else if(ballProximity == 3)
  {
    pursuitSpeed = 155; //165
    compensationModifier = 0.4; //large differences in modifier creates less smooth motion
  }
  else if(ballProximity == 2)
  {
    pursuitSpeed = 165; 
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
  if(ballProximity != 0) //Only run if the ball is seen by the IR sensors
  {
    //Ensuring the bounds of ball angle are between 0 and 360
    if(ballAngle < 0 && ballAngle >= -180)
    {
      ballAngle = ballAngle + 360;
    }
    float pursuitAngle;
    if(ballAngle >= 85 && ballAngle <= 95) // Ball straight ahead
    {
      pursuitAngle = ballAngle;
    }
    else if(ballAngle > 95 && ballAngle <= 270) // Ball is on the left of robot
    {
        pursuitAngle = ballAngle + compensationModifier*(ballAngle - 95); //Add compensation
    }
    else if((ballAngle < 85 && ballAngle >= 0)) // Ball is on the right of robot
    {
      pursuitAngle = ballAngle - compensationModifier*(85 - ballAngle);
    }
    else if (ballAngle <= 360 && ballAngle > 270)
    {
      pursuitAngle = ballAngle - compensationModifier*85 - compensationModifier*(360 - ballAngle);
    }
   
    drivePID(pursuitAngle,pursuitSpeed);
  }
  else
  {
      motor(M1, 0);
      motor(M2, 0);
      motor(M3, 0);
      motor(M4, 0);
      lcdLine1 = "No Ball";
      //digitalWrite(LED3, HIGH);
  }
  ////////
  //State Transitions
  ////////
  //See if the ball was captured using light gate and IR sensors
  bool LG_Status = readLightGate();
  if(LG_Status == true && ballAngle > 80 && ballAngle < 100 && ballProximity == 4)
  {
    state = FORWARD_HAS_BALL; //Transition to HasBall State
    leftOfGoal = false;
    rightOfGoal = false;
    goalDetected = false;
    kickPhase = false;
    startKickTime = millis();
  }
  //See if the robot is out of bounds using ground sensor
  readGroundSensor();
  bool lineSeen = false;
  for(int i=0; i<15; i++)
  {
    if(groundSensor[i] > GS_THRESHOLD_WHITE)
    {
      lineSeen = true;
    }
  }
  if(lineSeen == true) //Start the transition
  {
       brakeMotors();
       lineLeft = false;
       lineRight = false;
       lineFront = false;
       lineBack = false;
        if(groundSensor[GS0] > GS_THRESHOLD_WHITE || groundSensor[GS1] > GS_THRESHOLD_WHITE || groundSensor[GS2] > GS_THRESHOLD_WHITE)
        {
          lineFront = true;
          //Corner Checks - Front left and front right corners
          if(readSonar(US1) < 36 && readSonar(US1) > 22 && readSonar(US6) < 36 && readSonar(US6) > 22) //Front left corner check
          {
            lineLeft = true;
          }
          if(readSonar(US3) < 36 && readSonar(US3) > 22 && readSonar(US4) < 36 && readSonar(US4) > 22) //Front right corner check
          {
            lineRight = true;
          }
        }
        if(groundSensor[GS3] > GS_THRESHOLD_WHITE || groundSensor[GS4] > GS_THRESHOLD_WHITE || groundSensor[GS5] > GS_THRESHOLD_WHITE || groundSensor[GS6] > GS_THRESHOLD_WHITE)
        {
          lineRight = true;
          //Corner Checks - Front right and back right corners
          if(readSonar(US3) < 36  && readSonar(US3) > 22 && readSonar(US4) < 36 && readSonar(US4) > 22) //Front right corner check
          {
            lineFront = true;
          }
          if(readSonar(US4) < 36 && readSonar(US4) > 22 && readSonar(US5) < 36  && readSonar(US5) > 22) //Back right Corner Check
          {
            lineBack = true;
          }
        }
        if(groundSensor[GS7] > GS_THRESHOLD_WHITE || groundSensor[GS8] > GS_THRESHOLD_WHITE || groundSensor[GS9] > GS_THRESHOLD_WHITE || groundSensor[GS10] > GS_THRESHOLD_WHITE)
        {
          lineLeft = true;
          //Corner Checks - Front left and Back left corners
          if(readSonar(US1) < 36 && readSonar(US1) > 22 && readSonar(US6) < 36  && readSonar(US6) > 22) //Front left corner check
          {
            lineFront = true;
          }
          if(readSonar(US5) < 36 && readSonar(US5) > 22 && readSonar(US6) < 36  && readSonar(US6) > 22) //Back left Corner Check
          {
            lineBack = true;
          }
        }
        if(groundSensor[GS11] > GS_THRESHOLD_WHITE || groundSensor[GS12] > GS_THRESHOLD_WHITE || groundSensor[GS13] > GS_THRESHOLD_WHITE || groundSensor[GS14] > GS_THRESHOLD_WHITE)
        {
          lineBack = true;
          if(readSonar(US5) < 36 && readSonar(US5) > 22 && readSonar(US6) < 36  && readSonar(US6) > 22) //Back left corner check
          {
            lineLeft = true;
          }
          if(readSonar(US4) < 36 && readSonar(US4) > 22 && readSonar(US5) < 36  && readSonar(US5) > 22) //Back right Corner Check
          {
            lineRight = true;
          }
        }
      state = FORWARD_AVOID_OUTOFBOUNDS; //Transition to avoid out of bounds state
      startAvoidOutTime = millis();
  }
}

void forwardHasBall()
{
  lcdLine1 = "HasBall";
  //lcdLine2 = "Forward";
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, HIGH);
  digitalWrite(LED4, LOW);
  //Turn on absolute mode
  driveAbsoluteMode = true;

  //Left/Right of Goal Checking
  if(readSonar(US4) > 111) //was 121
  {
    leftOfGoal = true;
    //digitalWrite(LED2, HIGH);
  }

  if(readSonar(US6) > 111) //was 121
  {
    rightOfGoal = true;
    //digitalWrite(LED4, HIGH);
  }
  
//Goal Detection code - checks for proximity to goal
  int sonar1 = readSonar(US1);
  int sonar2 = readSonar(US2);
  int sonar3 = readSonar(US3);
  if(sonar1 > sonar2 && sonar1 > sonar3) //sonar1 is the highest value
  {
    if(sonar1 < 60) {
      goalDetected = true;
      //digitalWrite(LED1, HIGH);
    }
  }
  else if(sonar2 > sonar1 && sonar2 > sonar3) //sonar2 is the highest value
  {
    if(sonar2 < 60) {
      goalDetected = true;
      //digitalWrite(LED1, HIGH);
    }
  }
  else if(sonar3 > sonar1 && sonar3 > sonar2) //sonar3 is the highest value
  {
    if(sonar3 < 60) {
      goalDetected = true;
      //digitalWrite(LED1, HIGH);
    }
  }

  //Timeline
  long currentTime = millis();
  
  //Conditions for KickPhase
  if((goalDetected == true && kickPhase == false && (currentTime - startKickTime) > 200 && (currentTime - startKickTime) < 620) || ((currentTime - startKickTime) > 620 && kickPhase == false))
  {
    kickPhase = true;
    startKickPhase = millis();
  }
  if(kickPhase == false)
  {
    digitalWrite(KICKER, LOW);
    ///Smart Aiming for Goal
    if(leftOfGoal && rightOfGoal) //Both sonars see above 121, this shouldn't happen - except in superteams
    {
      drivePID(90, 180); //Drive Straight Ahead
      rotOffset = 0;
    }
    else if(leftOfGoal)
    {
      drivePID(90, 190); //Drive Right and Forward
      rotOffset = -20;
    }
    else if(rightOfGoal)
    {
      drivePID(90, 190); //Drive Left and Forward
      rotOffset = 20;
    }
    else //we are in front of goal, just go straight
    {
      drivePID(90, 180); //Drive Straight Ahead
      rotOffset = 0;
    }
  }
  else //Kick Phase was initiated
  {
    if((currentTime - startKickPhase) < 80){
       digitalWrite(KICKER, HIGH); //Kick
      ///Smart Aiming for Goal
      if(leftOfGoal && rightOfGoal) //Both sonars see above 121, this shouldn't happen - except in superteams
      {
        drivePID(90, 180); //Drive Straight Ahead
        rotOffset = 0;
      }
      else if(leftOfGoal)
      {
        drivePID(90, 190); //Drive Right and Forward
        rotOffset = -20;
      }
      else if(rightOfGoal)
      {
        drivePID(90, 190); //Drive Left and Forward
        rotOffset = 20;
      }
      else //we are in front of goal, just go straight
      {
        drivePID(90, 180); //Drive Straight Ahead
        rotOffset = 0;
      }
    }
    else if((currentTime - startKickPhase) < 580)
    {
      digitalWrite(KICKER, LOW);
      //drivePID(0,0);
      rotOffset = 0;
      motor(M1, 0);
      motor(M2, 0);
      motor(M3, 0);
      motor(M4, 0);
    }
    else
    {
      digitalWrite(KICKER, LOW);
      state = FORWARD_BALL_PURSUIT; //Transistion back to ball pursuit
    }
  }
  
  /////////////////////
  ///State Transitions
  ////////////////////

  //See if the robot is out of bounds using ground sensor
  readGroundSensor();
  bool lineSeen = false;
  for(int i=0; i<15; i++)
  {
    if(groundSensor[i] > GS_THRESHOLD_WHITE)
    {
      lineSeen = true;
    }
  }
  if(lineSeen == true) //Start the transition
  {
       brakeMotors();
       digitalWrite(KICKER, LOW); //Turn off Kicker - This fixes the spinning problem
       rotOffset = 0;

       lineLeft = false;
       lineRight = false;
       lineFront = false;
       lineBack = false;
       if(groundSensor[GS0] > GS_THRESHOLD_WHITE || groundSensor[GS1] > GS_THRESHOLD_WHITE || groundSensor[GS2] > GS_THRESHOLD_WHITE)
        {
          lineFront = true;
          //Corner Checks - Front left and front right corners
          if(readSonar(US1) < 36 && readSonar(US1) > 22 && readSonar(US6) < 36 && readSonar(US6) > 22) //Front left corner check
          {
            lineLeft = true;
          }
          if(readSonar(US3) < 36 && readSonar(US3) > 22 && readSonar(US4) < 36 && readSonar(US4) > 22) //Front right corner check
          {
            lineRight = true;
          }
        }
        if(groundSensor[GS3] > GS_THRESHOLD_WHITE || groundSensor[GS4] > GS_THRESHOLD_WHITE || groundSensor[GS5] > GS_THRESHOLD_WHITE || groundSensor[GS6] > GS_THRESHOLD_WHITE)
        {
          lineRight = true;
          //Corner Checks - Front right and back right corners
          if(readSonar(US3) < 36  && readSonar(US3) > 22 && readSonar(US4) < 36 && readSonar(US4) > 22) //Front right corner check
          {
            lineFront = true;
          }
          if(readSonar(US4) < 36 && readSonar(US4) > 22 && readSonar(US5) < 36  && readSonar(US5) > 22) //Back right Corner Check
          {
            lineBack = true;
          }
        }
        if(groundSensor[GS7] > GS_THRESHOLD_WHITE || groundSensor[GS8] > GS_THRESHOLD_WHITE || groundSensor[GS9] > GS_THRESHOLD_WHITE || groundSensor[GS10] > GS_THRESHOLD_WHITE)
        {
          lineLeft = true;
          //Corner Checks - Front left and Back left corners
          if(readSonar(US1) < 36 && readSonar(US1) > 22 && readSonar(US6) < 36  && readSonar(US6) > 22) //Front left corner check
          {
            lineFront = true;
          }
          if(readSonar(US5) < 36 && readSonar(US5) > 22 && readSonar(US6) < 36  && readSonar(US6) > 22) //Back left Corner Check
          {
            lineBack = true;
          }
        }
        if(groundSensor[GS11] > GS_THRESHOLD_WHITE || groundSensor[GS12] > GS_THRESHOLD_WHITE || groundSensor[GS13] > GS_THRESHOLD_WHITE || groundSensor[GS14] > GS_THRESHOLD_WHITE)
        {
          lineBack = true;
          if(readSonar(US5) < 36 && readSonar(US5) > 22 && readSonar(US6) < 36  && readSonar(US6) > 22) //Back left corner check
          {
            lineLeft = true;
          }
          if(readSonar(US4) < 36 && readSonar(US4) > 22 && readSonar(US5) < 36  && readSonar(US5) > 22) //Back right Corner Check
          {
            lineRight = true;
          }
        }
      state = FORWARD_AVOID_OUTOFBOUNDS; //Transition to avoid out of bounds state
      startAvoidOutTime = millis();
  }
}
void forwardAvoidOutOfBounds()
{
  lcdLine1 = "AvoidOut";
  //lcdLine2 = "Forward";
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);
  digitalWrite(LED4, HIGH);

  //Make Sure Kicker is off
  digitalWrite(KICKER, LOW);
  rotOffset = 0;

  //Turn off absolute mode
  driveAbsoluteMode = false;

  //Only react to new sensor data once the robot has reacted to the initial data and is not sliding out of bounds (after 200ms)
  long currentTime = millis();
  if(currentTime - startAvoidOutTime > 100) //was 200ms
  {
    lineLeft = false;
    lineRight = false;
    lineFront = false;
    lineBack = false;
    readGroundSensor();
    if(groundSensor[GS0] > GS_THRESHOLD_WHITE || groundSensor[GS1] > GS_THRESHOLD_WHITE || groundSensor[GS2] > GS_THRESHOLD_WHITE)
    {
         lineFront = true;
    }
    if(groundSensor[GS3] > GS_THRESHOLD_WHITE || groundSensor[GS4] > GS_THRESHOLD_WHITE || groundSensor[GS5] > GS_THRESHOLD_WHITE || groundSensor[GS6] > GS_THRESHOLD_WHITE)
    {
         lineRight = true;
    }
    if(groundSensor[GS7] > GS_THRESHOLD_WHITE || groundSensor[GS8] > GS_THRESHOLD_WHITE || groundSensor[GS9] > GS_THRESHOLD_WHITE || groundSensor[GS10] > GS_THRESHOLD_WHITE)
    {
         lineLeft = true;
    }
    if(groundSensor[GS11] > GS_THRESHOLD_WHITE || groundSensor[GS12] > GS_THRESHOLD_WHITE || groundSensor[GS13] > GS_THRESHOLD_WHITE || groundSensor[GS14] > GS_THRESHOLD_WHITE)
    {
         lineBack = true;
    }
  }
  ////////
  //State Transitions
  ////////
  if(currentTime - startAvoidOutTime < 650)
  {
    //Handling Out of bounds cases
    if(lineFront == true && lineBack == true && lineLeft == true && lineRight == true) //All sides detected - this should never happen
    {
      drivePID(0,0); //Stop
    }
    else if(lineFront == true && lineLeft == true && lineRight == true) //The front, left and right are all detected
    {
      drivePID(270,140); //drive back
    }
    else if(lineBack == true && lineLeft == true && lineRight == true) //The back, left and right are all detected
    {
      drivePID(90,140); //drive forward
    }
    else if(lineBack == true && lineFront == true && lineRight == true) //The right, front and back are all detected
    {
      drivePID(180,140); //drive left
    }
    else if(lineBack == true && lineFront == true && lineLeft == true) //The left, front and back are all detected
    {
      drivePID(0,140); //drive right
    } 
    else if(lineFront == true && lineRight == true) //The Front and Right are detected (corner)
    {
       drivePID(225,140); //drive back and left
    }
    else if(lineFront == true && lineLeft == true) //The Front and Left are detected (corner)
    {
       drivePID(315,140); //drive back and Right
    }
    else if(lineBack == true && lineRight == true) //The Back and Right are detected (corner)
    {
       drivePID(135,140); //drive forward and left
    }
    else if(lineBack == true && lineLeft == true) //The Back and Left are detected (corner)
    {
       drivePID(45,140); //drive forward and Right
    }
    else if(lineFront == true) //The front is detected
    {
       drivePID(270,140); //drive back
    }
    else if(lineBack == true) //The Back is detected
    {
       drivePID(90,140); //drive forward
    }
    else if(lineRight == true) //The Right is detected
    {
       drivePID(180,140); //drive left
    }
    else if(lineLeft == true) //The Left is detected
    {
       drivePID(0,140); //drive Right
    } 
    else  //no line
    {
      drivePID(0,0);
    }
  }
  else
  {
    state = FORWARD_BALL_PURSUIT;
  }
}
//////////////
//State Machine Functions - Defense Strategic States//
//////////////
void defenseMoveAndBlock()
{
  lcdLine1 = "Block";
  //lcdLine2 = "Forward";
  digitalWrite(LED1, HIGH);
  digitalWrite(LED2, LOW);
  digitalWrite(LED4, LOW);

  //Make Sure Kicker is off
  digitalWrite(KICKER, LOW);
  rotOffset = 0;

  //Turn on absolute mode
  driveAbsoluteMode = true;

  if(digitalRead(B3) == HIGH)
  {
      state = START_DEFENSE;
      motor(M1, 0);
      motor(M2, 0);
      motor(M3, 0);
      motor(M4, 0);
      delay(500);
  } 
  float ballAngle = readIR();
  //Handling block cases
  if(ballProximity != 0) //Only run if the IR sensors see the ball
  {
    if(ballAngle > 80 && ballAngle < 100)
    {
      drivePID(0,0);
    }
    else if(ballAngle > 60 && ballAngle < 80)
    {
      drivePID(0,80);
    }
    else if(ballAngle > 100 && ballAngle < 120)
    {
      drivePID(180,80);
    }
    else if(ballAngle > 0 && ballAngle < 60)
    {
      drivePID(0,100);
    }
    else if(ballAngle > 120 && ballAngle < 180)
    {
      drivePID(180,100);
    }
    else
    {
      drivePID(0,0);
    }
  }
  else
  {
      //digitalWrite(LED3, HIGH);
      motor(M1, 0);
      motor(M2, 0);
      motor(M3, 0);
      motor(M4, 0);
  }
  //Back up if the sonar is too far away
  int US5_val = readSonar(US5);
  /*
  if(US5_val > 34)
  {
    drivePID(270,90);
  }
  */
  ////
  //State Transitions
  ////
  //See if the ball was captured using light gate and IR sensors
  bool LG_Status = readLightGate();
  if(LG_Status == true && ballAngle > 80 && ballAngle < 100 && ballProximity == 4)
  {
    state = DEFENSE_HAS_BALL; //Transition to defense has ball state
    startKickTime = millis();
  }
  //See if the robot is out of the box using ground sensor
  readGroundSensor();
  bool blackLineSeen = false;
  for(int i=0; i<15; i++)
  {
    if(groundSensor[i] < GS_THRESHOLD_BLACK)
    {
      blackLineSeen = true;
    }
  }
  if(blackLineSeen == true) //Start the transition
  {
       //brakeMotors();
       lineLeft = false;
       lineRight = false;
       lineFront = false;
       lineBack = false;
        if(groundSensor[GS0] < GS_THRESHOLD_BLACK || groundSensor[GS1] < GS_THRESHOLD_BLACK || groundSensor[GS2] < GS_THRESHOLD_BLACK)
        {
          blackLineFront = true;
        }
        if(groundSensor[GS3] < GS_THRESHOLD_BLACK || groundSensor[GS4] < GS_THRESHOLD_BLACK || groundSensor[GS5] < GS_THRESHOLD_BLACK || groundSensor[GS6] < GS_THRESHOLD_BLACK)
        {
          blackLineRight = true;
        }
        if(groundSensor[GS7] < GS_THRESHOLD_BLACK || groundSensor[GS8] < GS_THRESHOLD_BLACK || groundSensor[GS9] < GS_THRESHOLD_BLACK || groundSensor[GS10] < GS_THRESHOLD_BLACK)
        {
          blackLineLeft = true;
        }
        if(groundSensor[GS11] < GS_THRESHOLD_BLACK || groundSensor[GS12] < GS_THRESHOLD_BLACK || groundSensor[GS13] < GS_THRESHOLD_BLACK || groundSensor[GS14] < GS_THRESHOLD_BLACK)
        {
          blackLineBack = true;
        }
      //state = DEFENSE_AVOID_OUTOFBOX; //Transition to avoid out of bounds state
      //startAvoidOutTime = millis();
  }
  //See if the robot is out of bounds using ground sensor
  readGroundSensor();
  bool whiteLineSeen = false;
  for(int i=0; i<15; i++)
  {
    if(groundSensor[i] > GS_THRESHOLD_WHITE)
    {
      whiteLineSeen = true;
    }
  }
  if(whiteLineSeen == true) //Start the transition
  {
       brakeMotors();
       lineLeft = false;
       lineRight = false;
       lineFront = false;
       lineBack = false;
        if(groundSensor[GS0] > GS_THRESHOLD_WHITE || groundSensor[GS1] > GS_THRESHOLD_WHITE || groundSensor[GS2] > GS_THRESHOLD_WHITE)
        {
          lineFront = true;
          //Corner Checks - Front left and front right corners
          if(readSonar(US1) < 36 && readSonar(US1) > 22 && readSonar(US6) < 36 && readSonar(US6) > 22) //Front left corner check
          {
            lineLeft = true;
          }
          if(readSonar(US3) < 36 && readSonar(US3) > 22 && readSonar(US4) < 36 && readSonar(US4) > 22) //Front right corner check
          {
            lineRight = true;
          }
        }
        if(groundSensor[GS3] > GS_THRESHOLD_WHITE || groundSensor[GS4] > GS_THRESHOLD_WHITE || groundSensor[GS5] > GS_THRESHOLD_WHITE || groundSensor[GS6] > GS_THRESHOLD_WHITE)
        {
          lineRight = true;
          //Corner Checks - Front right and back right corners
          if(readSonar(US3) < 36 && readSonar(US3) > 22 && readSonar(US4) < 36 && readSonar(US4) > 22) //Front right corner check
          {
            lineFront = true;
          }
          if(readSonar(US4) < 36 && readSonar(US4) > 22 && readSonar(US5) < 36 && readSonar(US5) > 22) //Back right Corner Check
          {
            lineBack = true;
          }
        }
        if(groundSensor[GS7] > GS_THRESHOLD_WHITE || groundSensor[GS8] > GS_THRESHOLD_WHITE || groundSensor[GS9] > GS_THRESHOLD_WHITE || groundSensor[GS10] > GS_THRESHOLD_WHITE)
        {
          lineLeft = true;
          //Corner Checks - Front left and Back left corners
          if(readSonar(US1) < 36 && readSonar(US1) > 22 && readSonar(US6) < 36 && readSonar(US6) > 22) //Front left corner check
          {
            lineFront = true;
          }
          if(readSonar(US5) < 36 && readSonar(US5) > 22 && readSonar(US6) < 36 && readSonar(US6) > 22) //Back left Corner Check
          {
            lineBack = true;
          }
        }
        if(groundSensor[GS11] > GS_THRESHOLD_WHITE || groundSensor[GS12] > GS_THRESHOLD_WHITE || groundSensor[GS13] > GS_THRESHOLD_WHITE || groundSensor[GS14] > GS_THRESHOLD_WHITE)
        {
          lineBack = true;
          if(readSonar(US5) < 36 && readSonar(US5) > 22 && readSonar(US6) < 36 && readSonar(US6) > 22) //Back left corner check
          {
            lineLeft = true;
          }
          if(readSonar(US4) < 36 && readSonar(US4) > 22 && readSonar(US5) < 36 && readSonar(US5) > 22) //Back right Corner Check
          {
            lineRight = true;
          }
        }
      state = DEFENSE_AVOID_OUTOFBOUNDS; //Transition to avoid out of bounds state
      startAvoidOutTime = millis();
  }
}
void defenseHasBall()
{
  lcdLine1 = "HasBall";
  //lcdLine2 = "Defense";
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, HIGH);
  digitalWrite(LED4, LOW);
  //Turn on absolute mode
  driveAbsoluteMode = true;

  //Timeline
  long currentTime = millis();
  if((currentTime - startKickTime) < 220) 
  {
    rotOffset = 50;
    drivePID(90, 180); //Drive Straight Ahead
  }
  else if((currentTime - startKickTime) < 300)
  {
    rotOffset = 50;
    digitalWrite(KICKER, HIGH); //Kick
    drivePID(90, 190);
  }
  else if((currentTime - startKickTime) < 600)
  {
    ///Back up and Kicker Off
    digitalWrite(KICKER, LOW);
    drivePID(270, 180);
  }
  else
  {
    rotOffset = 0;
    state = DEFENSE_MOVE_AND_BLOCK; //Transistion back to move and block
  }
   ///State Transitions

  //See if the robot is out of bounds using ground sensor
  readGroundSensor();
  bool lineSeen = false;
  for(int i=0; i<15; i++)
  {
    if(groundSensor[i] > GS_THRESHOLD_WHITE)
    {
      lineSeen = true;
    }
  }
  if(lineSeen == true) //Start the transition
  {
       brakeMotors();
       digitalWrite(KICKER, LOW); //Turn off Kicker - This fixes the spinning problem
       rotOffset = 0;

       lineLeft = false;
       lineRight = false;
       lineFront = false;
       lineBack = false;
        if(groundSensor[GS0] > GS_THRESHOLD_WHITE || groundSensor[GS1] > GS_THRESHOLD_WHITE || groundSensor[GS2] > GS_THRESHOLD_WHITE)
        {
          lineFront = true;
          //Corner Checks - Front left and front right corners
          if(readSonar(US1) < 36 && readSonar(US6) < 36) //Front left corner check
          {
            lineLeft = true;
          }
          if(readSonar(US3) < 36 && readSonar(US4) < 36) //Front right corner check
          {
            lineRight = true;
          }
        }
        if(groundSensor[GS3] > GS_THRESHOLD_WHITE || groundSensor[GS4] > GS_THRESHOLD_WHITE || groundSensor[GS5] > GS_THRESHOLD_WHITE || groundSensor[GS6] > GS_THRESHOLD_WHITE)
        {
          lineRight = true;
          //Corner Checks - Front right and back right corners
          if(readSonar(US3) < 36 && readSonar(US4) < 36) //Front right corner check
          {
            lineFront = true;
          }
          if(readSonar(US4) < 36 && readSonar(US5) < 36) //Back right Corner Check
          {
            lineBack = true;
          }
        }
        if(groundSensor[GS7] > GS_THRESHOLD_WHITE || groundSensor[GS8] > GS_THRESHOLD_WHITE || groundSensor[GS9] > GS_THRESHOLD_WHITE || groundSensor[GS10] > GS_THRESHOLD_WHITE)
        {
          lineLeft = true;
          //Corner Checks - Front left and Back left corners
          if(readSonar(US1) < 36 && readSonar(US6) < 36) //Front left corner check
          {
            lineFront = true;
          }
          if(readSonar(US5) < 36 && readSonar(US6) < 36) //Back left Corner Check
          {
            lineBack = true;
          }
        }
        if(groundSensor[GS11] > GS_THRESHOLD_WHITE || groundSensor[GS12] > GS_THRESHOLD_WHITE || groundSensor[GS13] > GS_THRESHOLD_WHITE || groundSensor[GS14] > GS_THRESHOLD_WHITE)
        {
          lineBack = true;
          if(readSonar(US5) < 36 && readSonar(US6) < 36) //Back left corner check
          {
            lineLeft = true;
          }
          if(readSonar(US4) < 36 && readSonar(US5) < 36) //Back right Corner Check
          {
            lineRight = true;
          }
        }
      state = DEFENSE_AVOID_OUTOFBOUNDS; //Transition to avoid out of bounds state
      startAvoidOutTime = millis();
  }
}
void defenseAvoidOutOfBox()
{
  lcdLine1 = "AvoidBox";
  //lcdLine2 = "Forward";
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);
  digitalWrite(LED4, HIGH);
  //Make Sure Kicker is off
  digitalWrite(KICKER, LOW);
  rotOffset = 0;

  //Turn off absolute mode
  driveAbsoluteMode = false;

  //Only react to new sensor data once the robot has reacted to the initial data and is not sliding out of bounds (after 200ms)
  long currentTime = millis();
  if(currentTime - startAvoidOutTime > 300) 
  {
    blackLineLeft = false;
    blackLineRight = false;
    blackLineFront = false;
    blackLineBack = false;
    readGroundSensor();
    if(groundSensor[GS0] < GS_THRESHOLD_BLACK || groundSensor[GS1] < GS_THRESHOLD_BLACK || groundSensor[GS2] < GS_THRESHOLD_BLACK)
    {
      blackLineFront = true;
    }
    if(groundSensor[GS3] < GS_THRESHOLD_BLACK || groundSensor[GS4] < GS_THRESHOLD_BLACK || groundSensor[GS5] < GS_THRESHOLD_BLACK || groundSensor[GS6] < GS_THRESHOLD_BLACK)
    {
      blackLineRight = true;
    }
    if(groundSensor[GS7] < GS_THRESHOLD_BLACK || groundSensor[GS8] < GS_THRESHOLD_BLACK || groundSensor[GS9] < GS_THRESHOLD_BLACK || groundSensor[GS10] < GS_THRESHOLD_BLACK)
    {
      blackLineLeft = true;
    }
    if(groundSensor[GS11] < GS_THRESHOLD_BLACK || groundSensor[GS12] < GS_THRESHOLD_BLACK || groundSensor[GS13] < GS_THRESHOLD_BLACK || groundSensor[GS14] < GS_THRESHOLD_BLACK)
    {
      blackLineBack = true;
    }
  }
  ////////
  //State Transitions
  ////////
  if(currentTime - startAvoidOutTime < 500)
  {
    //Handling Out of bonds cases
    if(blackLineFront == true && blackLineBack == true && blackLineLeft == true && blackLineRight == true) //All sides detected - this should never happen
    {
      drivePID(0,0); //Stop
    }
    else if(blackLineFront == true && blackLineLeft == true && blackLineRight == true) //The front, left and right are all detected
    {
      drivePID(270,90); //drive back
    }
    else if(blackLineBack == true && blackLineLeft == true && blackLineRight == true) //The back, left and right are all detected
    {
      drivePID(270,90); //drive back
    }
    else if(blackLineBack == true && blackLineFront == true && blackLineRight == true) //The right, front and back are all detected
    {
      drivePID(180,90); //drive left
    }
    else if(blackLineBack == true && blackLineFront == true && blackLineLeft == true) //The left, front and back are all detected
    {
      drivePID(0,90); //drive right
    } 
    else if(blackLineFront == true && blackLineRight == true) //The Front and Right are detected (corner)
    {
       drivePID(225,90); //drive back and left
    }
    else if(blackLineFront == true && blackLineLeft == true) //The Front and Left are detected (corner)
    {
       drivePID(315,90); //drive back and right
    }
    else if(blackLineBack == true && blackLineRight == true) //The Back and Right are detected (corner)
    {
       drivePID(315,90); //drive back and right
    }
    else if(blackLineBack == true && blackLineLeft == true) //The Back and Left are detected (corner)
    {
       drivePID(225,90); //drive back and left
    }
    else if(blackLineFront == true) //The front is detected
    {
       drivePID(270,90); //drive back
    }
    else if(blackLineBack == true) //The Back is detected
    {
       drivePID(270,90); //drive back
    }
    else if(blackLineRight == true) //The Right is detected
    {
       drivePID(180,90); //drive left
    }
    else if(blackLineLeft == true) //The Left is detected
    {
       drivePID(0,90); //drive Right
    } 
    else  //no line
    {
      drivePID(0,0);
    }
  }
  else if(currentTime - startAvoidOutTime < 650)
  {
    drivePID(0,0);
  } 
  else
  {
    state = DEFENSE_MOVE_AND_BLOCK;
  }
}
void defenseAvoidOutOfBounds()
{
  lcdLine1 = "AvoidOut";
  //lcdLine2 = "Forward";
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);
  digitalWrite(LED4, HIGH);
  //Make Sure Kicker is off
  digitalWrite(KICKER, LOW);
  rotOffset = 0;
  
  //Turn off absolute mode
  driveAbsoluteMode = false;

  //Only react to new sensor data once the robot has reacted to the initial data and is not sliding out of bounds (after 200ms)
  long currentTime = millis();
  if(currentTime - startAvoidOutTime > 300) 
  {
    lineLeft = false;
    lineRight = false;
    lineFront = false;
    lineBack = false;
    readGroundSensor();
    if(groundSensor[GS0] > GS_THRESHOLD_WHITE || groundSensor[GS1] > GS_THRESHOLD_WHITE || groundSensor[GS2] > GS_THRESHOLD_WHITE)
    {
        lineFront = true;
    }
    if(groundSensor[GS3] > GS_THRESHOLD_WHITE || groundSensor[GS4] > GS_THRESHOLD_WHITE || groundSensor[GS5] > GS_THRESHOLD_WHITE || groundSensor[GS6] > GS_THRESHOLD_WHITE)
    {
        lineRight = true;
    }
    if(groundSensor[GS7] > GS_THRESHOLD_WHITE || groundSensor[GS8] > GS_THRESHOLD_WHITE || groundSensor[GS9] > GS_THRESHOLD_WHITE || groundSensor[GS10] > GS_THRESHOLD_WHITE)
    {
        lineLeft = true;
    }
    if(groundSensor[GS11] > GS_THRESHOLD_WHITE || groundSensor[GS12] > GS_THRESHOLD_WHITE || groundSensor[GS13] > GS_THRESHOLD_WHITE || groundSensor[GS14] > GS_THRESHOLD_WHITE)
    {
        lineBack = true;
    }
  }

  if(currentTime - startAvoidOutTime < 500)
  {
    //Handling Out of bounds cases
    if(lineFront == true && lineBack == true && lineLeft == true && lineRight == true) //All sides detected - this should never happen
    {
      drivePID(0,0); //Stop
    }
    else if(lineFront == true && lineLeft == true && lineRight == true) //The front, left and right are all detected
    {
      drivePID(270,90); //drive back
    }
    else if(lineBack == true && lineLeft == true && lineRight == true) //The back, left and right are all detected
    {
      drivePID(90,90); //drive forward
    }
    else if(lineBack == true && lineFront == true && lineRight == true) //The right, front and back are all detected
    {
      drivePID(180,90); //drive left
    }
    else if(lineBack == true && lineFront == true && lineLeft == true) //The left, front and back are all detected
    {
      drivePID(0,90); //drive right
    } 
    else if(lineFront == true && lineRight == true) //The Front and Right are detected (corner)
    {
       drivePID(225,90); //drive back and left
    }
    else if(lineFront == true && lineLeft == true) //The Front and Left are detected (corner)
    {
       drivePID(315,90); //drive back and Right
    }
    else if(lineBack == true && lineRight == true) //The Back and Right are detected (corner)
    {
       drivePID(135,90); //drive forward and left
    }
    else if(lineBack == true && lineLeft == true) //The Back and Left are detected (corner)
    {
       drivePID(45,90); //drive forward and Right
    }
    else if(lineFront == true) //The front is detected
    {
       drivePID(270,90); //drive back
    }
    else if(lineBack == true) //The Back is detected
    {
       drivePID(90,90); //drive forward
    }
    else if(lineRight == true) //The Right is detected
    {
       drivePID(180,90); //drive left
    }
    else if(lineLeft == true) //The Left is detected
    {
       drivePID(0,90); //drive Right
    } 
    else  //no line
    {
      drivePID(0,0);
    }
  }
  else if(currentTime - startAvoidOutTime < 650)
  {
    drivePID(0,0);
  } 
  else
  {
    state = DEFENSE_MOVE_AND_BLOCK;
  }
}


/////////////
//Functions for Controlling Robot Drive//
//////////////
float kp = 4.0;  
float kd  = 0.00; 
float lastErr = 0.0;

void drivePID(int dir, int PWR)
{
  float targetVal = fieldNorth + rotOffset; //make the setpoint field north

  //float currentVal = readCompass();
  float currentVal = readCompass();
  lcdLine2 = String(currentVal);
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

  if(PWR != 0 && compassWorking == true && currentVal != -1) //the robot is driving
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
    //adjust heading based on rotational error if Absolute Mode is on
    int correctedDir;
    if(driveAbsoluteMode)
    {
      correctedDir = dir-pErr;
    }
    else
    {
      correctedDir = dir;
    }
    drive(correctedDir,-totalRotGain, PWR);
  }
  else if(compassWorking == true &&  currentVal != -1) //the robot is standing in place (still do rotational corrections)
  {
    if((totalRotGain > 24 || totalRotGain < -24) && currentVal != -1) //20
    {  
      //Serial.print(" Total Gain:");
      //Serial.println(totalRotGain);
      totalRotGain = constrain(totalRotGain,-60,60);
      //digitalWrite(LED2, LOW);
      //digitalWrite(LED4, HIGH);
      drive(dir,-totalRotGain, 0);
    }
    else
    {
      //Serial.print(" Total Gain:");
      //Serial.println(totalRotGain);
      //digitalWrite(LED2, HIGH);
      //digitalWrite(LED4, LOW);
      
       motor(M1, 0);
       motor(M2, 0);
       motor(M3, 0);
       motor(M4, 0);
       //brakeMotors();
    }
  }
  else //compass not working, stop robot
  {
    motor(M1, 0);
    motor(M2, 0);
    motor(M3, 0);
    motor(M4, 0);
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
  //Handling Inverted motor control for second robot
  if(INVERT)
  {
    if(motorDirection == true)
    {
      motorDirection = false;
    }
    else
    {
      motorDirection = true;
    }
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
float readCompass() //8 bit mode to avoid errors
{
  reset:
  Serial1.write(compassCommand);
  long waitStartTime = millis();
  while(Serial1.available() < 1)
  {
    long currentTime = millis();
    if((currentTime - waitStartTime) > 400)
    {
      Serial.println("No Compass Plugged In");
      compassWorking = false;
      return -1;
    }
  }
  if(compassWorking == false) //The compass wasn't working previously but it came back on
  {
    //Reset Serial communication
    Serial.println("Compass Reset");
    Serial1.end();
    Serial1.begin(9600);
    compassWorking = true;
    //Serial.print("Number of Bytes Available: ");
    //Serial.println(Serial1.available());
    goto reset; //restart function
  }
  byte compass_raw = Serial1.read();
  int mappedVal = map(compass_raw, 0,255, 0,3600); //maps the 8bit angle to 0-3600
  float processedVal = ((float)mappedVal)/(float)10;
  return processedVal;
}
////Reading Light Gate Sensor
bool readLightGate()
{
  int lightGateVal = readGSMux(LG);
  //Serial.print("Raw LG: "); Serial.println(lightGateVal);
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

////Read Sonar Sensors
int readSonar(int sonarPin)
{
  int US_raw = analogRead(sonarPin);
  int US_val = US_raw/0.782; 
  //Serial.print("US: ");
  //Serial.print(US);
  //Serial.println("cm");
  return US_val;
}
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

  //Serial.print("IR Sorted 0: "); Serial.println(IR_sorted[0]);
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
    groundSensor[i] = readGSMux(i);
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
  //digitalWrite(LED3, HIGH);
  lcd.setCursor(0,0);
  lcd.print(lcdLine1);
  lcd.setCursor(0,1);
  lcd.print(lcdLine2);
  //digitalWrite(LED3, LOW);
}

//////////////
//Math/Misc Functions//
//////////////
float angleDif(float x, float y) //used to calculate the difference between actual heading and the target heading in drivePID()
{
  float radianDif = ((x-y)*(float)71)/((float)4068); // to radians
  float a = (float) atan2(sin(radianDif), cos(radianDif));
  a =  (a * (float)4068) / (float)71; //to degrees 

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
