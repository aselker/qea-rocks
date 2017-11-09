#include <Dhcp.h>
#include <Dns.h>
#include <Ethernet.h>
#include <EthernetClient.h>
#include <EthernetServer.h>
#include <EthernetUdp.h>

#include <Wire.h>
#include "Balance.h"

int32_t gYZero;
int32_t angle; // millidegrees
int32_t angleRate; // degrees/s
int32_t distanceLeft;
int32_t speedLeft;
int32_t driveLeft;
int32_t distanceRight;
int32_t displacement;

int32_t speedRight;
int32_t driveRight;
int16_t motorSpeed;
int32_t  angle_accum;
int16_t angle_prev;
bool isBalancingStatus;
bool balanceUpdateDelayedStatus;
extern int16_t testSpeed;          // this is the motor speed 


void balanceSetup()
{
  // Initialize IMU.
  Wire.begin();
  if (!imu.init())
  {
    while(true)
    {
      Serial.println("Failed to detect and initialize IMU!");
      delay(200);
    }
  }
  imu.enableDefault();
  imu.writeReg(LSM6::CTRL2_G, 0b01011000); // 208 Hz, 1000 deg/s

  // Wait for IMU readings to stabilize.
  delay(1000);

  // Calibrate the gyro.
  int32_t total = 0;
  for (int i = 0; i < CALIBRATION_ITERATIONS; i++)
  {
    imu.read();
    total += imu.g.y;
    delay(1);
  }

  gYZero = total / CALIBRATION_ITERATIONS;
}

void lyingDown()
{
  // Reset things so it doesn't go crazy.
  motorSpeed = 0;
  distanceLeft = 0;
  distanceRight = 0;
  motors.setSpeeds(0, 0);

  if (angleRate > -6 && angleRate < 6)
  {
    // It's really calm, so we know the angles.
    if (imu.a.z > 0)
    {
      
      // this is based on coarse measurement of what I think the angle would be resting on the flat surface. 
      // this corresponds to 94.8 degrees
      angle = 94827;
  
    }
    else
    {
      // this is based on coarse measurement of what I think the angle would be resting on the flat surface. 
         angle = -94827; 
    }
    distanceLeft = 0;
    distanceRight = 0;
  }
}

void integrateGyro()
{
  // Convert from full-scale 1000 deg/s to deg/s.
  angleRate = (imu.g.y - gYZero) / 29;

  angle += angleRate * UPDATE_TIME_MS;
}

void integrateEncoders()
{
  static int16_t lastCountsLeft;
  int16_t countsLeft = encoders.getCountsLeft();
  speedLeft = (countsLeft - lastCountsLeft);
  distanceLeft += countsLeft - lastCountsLeft;
  lastCountsLeft = countsLeft;

  static int16_t lastCountsRight;
  int16_t countsRight = encoders.getCountsRight();
  speedRight = (countsRight - lastCountsRight);
  distanceRight += countsRight - lastCountsRight;
  lastCountsRight = countsRight;      
}

void balanceResetEncoders()
{
  distanceLeft = 0;
  distanceRight = 0;
}
uint32_t delta_ms = 0;
void balanceUpdateSensors()
{
  static uint32_t prev_ms = 0;
  uint32_t cur_ms = 0;

  cur_ms = millis();
  imu.read();
  integrateGyro();
  integrateEncoders();
  delta_ms = prev_ms - cur_ms;
  prev_ms = cur_ms;
}


