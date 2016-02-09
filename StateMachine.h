//StateMachine.h

//Finite state machine state IDs.

#define START_FORWARD 1    //1 - Start Menu / Forward 
#define START_DEFENSE 2    //2 - Start Menu / Defense
#define IR_TEST 3          //3 - IR Test
#define CMPS_TEST 4        //4 - Compass Test
#define CMPS_CALIBRATION 5 //5 - Compass Calibration
#define GS_TEST 6          //6 - GS Test
#define GS_CALIBRATION 7   //7 - GS Calibration
#define LG_TEST 8          //8 - Light Gate Test
#define LG_CALIBRATION 9   //9 - Light Gate Calibration
#define KICKER_TEST 10     //10 - Kicker Test

#define FORWARD_BALL_PURSUIT 100       //100 - Forward Ball Pursuit
#define FORWARD_HAS_BALL 101           //101 - Forward Has Ball
#define FORWARD_AVOID_OUTOFBOUNDS 102  //102 - Forward Avoid Out of Bounds

#define DEFENSE_MOVE_AND_BLOCK 200     //200 - Defense Move and Block
#define DEFENSE_HAS_BALL 201           //201 - Defense Has Ball
#define DEFENSE_AVOID_OUTOFBOX 202     //202 - Defense Avoid Out of Box
#define DEFENSE_AVOID_OUTOFBOUNDS 203  //203 - Defense Avoid Out of Bounds 

#define E_STOP 0 //0 - Emergency Stop