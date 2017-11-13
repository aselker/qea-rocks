// This code should help you get started with your balancing robot.
// The code performs the following steps
// Calibration phase:
//  In this phase the robot should be stationary lying on the ground.
//  The code will record the gyro data for a couple of seconds to zero
//  out any gyro drift.
//
// Waiting phase:
//  The robot will now start to integrate the gyro over time to estimate
//  the angle.  Once the angle gets within +/- 3 degrees of vertical,
//  we transition into the armed phase.  A buzzer will sound to indicate
//  this transition.
//
// Armed phase:
//  The robot is ready to go, however, it will not start executing its control
//  loop until the angle leaves the region of [-3 degrees, 3 degrees].  This
//  allows you to let go of your robot, and it won't start moving until it's started
//  to fall just a little bit.  Once it leaves the region around vertical, it enters
//  the controlled phase.
//
// Controlled phase:
//  Here you can implement your control logic to do your balancing (or any of the
//  other Olympic events.


#include <Balboa32U4.h>
#include <Wire.h>
#include <LSM6.h>
#include "Balance.h"

#define METERS_PER_CLICK 3.141592*80.0*(1/1000.0)/12.0/(162.5)
#define MOTOR_MAX 300
#define MAX_SPEED 0.75  // m/s
#define FORTY_FIVE_DEGREES_IN_RADIANS 0.78
#define ANGLE_CORRECTION (0.092)


extern int32_t angle_accum;
extern int32_t speedLeft, speedRight;
extern int32_t driveLeft, driveRight;
extern int32_t distanceLeft, distanceRight;

void balanceDoDriveTicks();

extern int32_t displacement;
int32_t prev_displacement=0;

LSM6 imu;
Balboa32U4Motors motors;
Balboa32U4Encoders encoders;
Balboa32U4Buzzer buzzer;
Balboa32U4ButtonA buttonA;
Balboa32U4ButtonB buttonB;
Balboa32U4ButtonC buttonC;


uint32_t prev_time = 0; //The last time the balancing loop (not void loop()! ) ran
uint32_t prev_print_time = 0; //The last time we printed over serial

void setup()
{
  Serial.begin(9600);

  angle_accum = 0; //Can't initialize this the normal way 'cause it's used in several files

  ledYellow(0);
  ledRed(1);
  balanceSetup();
  ledRed(0);
  ledGreen(0);
  ledYellow(0);
}

bool moveMode = false;
bool spinMode = false;

float start_time = 0;
extern int16_t angle_prev;
bool start_flag = false, armed_flag = false; 
int16_t start_counter = 0;
void lyingDown();
extern bool isBalancingStatus;
extern bool balanceUpdateDelayedStatus;
const float POS_METERS = 1.0;

/*
// These work for a robot with small weights up top.
const float kpCruise = 260, kiCruise = 4000, kdCruise = 3;
const float kpAngle = 3, kiAngle = 25, kdAngle = 0.3;
const float kpPos = 0.00007, kiPos = 0.000000012, kdPos = 0.016;
*/

// These work with a robot with large weights
const float kpCruise = 400, kiCruise = 4000, kdCruise = 2.5;
const float kpAngle = 3, kiAngle = 13, kdAngle = 0.14;
const float kpPos = 0.00004, kiPos = 0.000000009, kdPos = 0.016;

const int deadSpot = 13;

// Error functions
float posDesired = 0;
float spinDesired = 0;
float vDesired = 0; //The speed goal output by the angle PI loop
float errL = 0, errR = 0;
float intErrL = 0, intErrR = 0; //"int" means "integral", not "integer"
float vLLast = 0, vRLast = 0; //For the D term
float angleError, lastAngleError = 0;
float angleGoalAdjust; //How much to adjust "up" so we stay put
float posIntegral = 0;


void newBalanceUpdate()
{
  static uint32_t lastMillis;
  uint32_t ms = millis();

  if ((uint32_t)(ms - lastMillis) < UPDATE_TIME_MS) { return; }
  balanceUpdateDelayedStatus = ms - lastMillis > UPDATE_TIME_MS + 1;
  lastMillis = ms;

  balanceUpdateSensors(); // call functions to integrate encoders and gyros
 
   if (imu.a.x < 0)
  {
    lyingDown();
    isBalancingStatus = false;
  }
  else
  {
    isBalancingStatus = true;
  }
}


void loop()
{
  // Use the buttons to select the mode
  if (buttonA.getSingleDebouncedPress())
  {
    moveMode = false;
    spinMode = false;
  }
  else if (buttonB.getSingleDebouncedPress())
  {
    moveMode = true;
    spinMode = false;
  }
  else if (buttonC.getSingleDebouncedPress())
  {
    moveMode = false;
    spinMode = true;
  }


  
  uint32_t cur_time = millis();

  newBalanceUpdate(); // run the sensor updates. Note that this function checks whether it's been at least 10ms since it was last run.
  
  if(angle > 3000 || angle < -3000)  start_counter = 0; // If angle is not within +- 3 degrees, reset counter that waits for start

  if((cur_time - prev_print_time) > 105) { //do the printing every 105 ms. Don't want to do it for an integer multiple of 10ms to not hog the processor
//    Serial.println("Desired speed: " + String(vDesired) + "\tAngle error: " + String(angleError) + "\tAngle: " + String(angle) + "\tLeft wheel speed: " + String(speedLeft) + "\tAngle accumulator: " + String(angle_accum) + "\tAngle goal adjustment: " + String(angleGoalAdjust));
    prev_print_time = cur_time;
  }

  float delta_t = (cur_time - prev_time)/1000.0;

  if (prev_time == 0) delta_t = 0.01; // handle the case where this is the first time through the loop
  
  // every UPDATE_TIME_MS, check if angle is within +- 3 degrees and we haven't set the start flag yet
  if(cur_time - prev_time > UPDATE_TIME_MS && angle > -3000 && angle < 3000 && !armed_flag && !start_flag) {
    start_counter++; // Counts how long we've been withing a 3-deg range
    if(start_counter > 30) //If the start counter is greater than 30, this means that the angle has been within +- 3 degrees for 0.3 seconds, then set the start_flag
    {
      start_time = millis();
      angle_accum = 0;
      posDesired = 0;
      armed_flag = true;
      buzzer.playFrequency(DIV_BY_10 | 445, 200, 15);
    }
  }

  // only start when the angle falls outside of the 3.0 degree band around 0.  This allows you to let go of the robot before it starts balancing
  if(cur_time - prev_time > UPDATE_TIME_MS && (angle < -3000 || angle > 3000) && armed_flag)   
  {
    start_flag = true;
    armed_flag = false;
  }

  if(cur_time - prev_time > UPDATE_TIME_MS && start_flag) //every UPDATE_TIME_MS, if the start_flag has been set, do the balancing
  { 

    prev_time = cur_time; // set the previous time to the current time for the next run through the loop
    if (moveMode) { // If we're moving, start moving the setpoint
      posDesired = (cur_time - start_time) * 2;   
    } else {
      posDesired = 0;
    }

    // Uncomment this for SPIN
    if (spinMode) {
      spinDesired = 0.17;
    }

    float pos = (float(distanceLeft) + float(distanceRight)) / 2.0;
    Serial.print(pos);
    Serial.print("\t");
    Serial.print(POS_METERS/ METERS_PER_CLICK);
    Serial.print("\t");
    Serial.println(posDesired);

    posIntegral += (pos - posDesired);

    angleGoalAdjust = kpPos * (pos - posDesired)  + kiPos * posIntegral + kdPos * (float(speedLeft) + float(speedRight))/2.0; //These are set elsewhere.  Nonlocality! =D

    angleError = ((float)angle)/1000/180*3.14159 - ANGLE_CORRECTION; //This finds the angle error, in radians

    angle_accum += (angleError + angleGoalAdjust) * delta_t;
    vDesired = (angleError + angleGoalAdjust) * kpAngle + angle_accum * kiAngle + (angleError - lastAngleError) * kdAngle / delta_t;

    lastAngleError = angleError;
    
    // speedLeft and speedRight are just the change in the encoder readings
    // wee need to do some math to get them into m/s
    float vL = (METERS_PER_CLICK*speedLeft/delta_t) + spinDesired;
    float vR = (METERS_PER_CLICK*speedRight/delta_t) - spinDesired;

    errL = vDesired-vL;
    errR = vDesired-vR;

    intErrL += errL*delta_t;
    intErrR += errR*delta_t;

    float PWM_left = errL*kpCruise + intErrL*kiCruise - (vL - vLLast)*kdCruise / delta_t;
    float PWM_right = errR*kpCruise + intErrR*kiCruise - (vR - vRLast)*kdCruise / delta_t;

    //Dead spot compensation
    if (PWM_left > 0) PWM_left += deadSpot;
    else PWM_left -= deadSpot;
    if (PWM_right > 0) PWM_right += deadSpot;
    else PWM_right -= deadSpot;

    vLLast = vL;
    vRLast = vR;
    
    // if the robot is more than 45 degrees, shut down the motor
    if(start_flag && fabs(angleError + angleGoalAdjust) > FORTY_FIVE_DEGREES_IN_RADIANS) {
      start_flag = false;   /// wait for restart
      prev_time = 0;
      motors.setSpeeds(0, 0);
      balanceResetEncoders(); //Reset the encoder distances, to avoid messing with the angle adjustment
      angleGoalAdjust = 0;
      posIntegral = 0; angle_accum = 0;//And reset the integrals
    } else if(start_flag) motors.setSpeeds((int)PWM_left, (int)PWM_right);
  }

  // kill switch
  if (buttonA.getSingleDebouncedPress())
  {
      motors.setSpeeds(0,0);
      while(!buttonA.getSingleDebouncedPress()) {}
  }
}
