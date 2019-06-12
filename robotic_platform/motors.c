#include <stdio.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <softPwm.h>

#define RANGE	100
#define INITIAL_VALUE 0
#define M1_P 22//25
#define M1_N 21//23
#define M2_P 25//22
#define M2_N 23//21

void init_motors()
{
  wiringPiSetup();

  softPwmCreate(M1_P, INITIAL_VALUE, RANGE);
  softPwmCreate(M1_N, INITIAL_VALUE, RANGE);

  softPwmCreate(M2_P, INITIAL_VALUE, RANGE);
  softPwmCreate(M2_N, INITIAL_VALUE, RANGE);
}

void stop_motors()
{
  softPwmWrite(M2_P, 0);
  softPwmWrite(M2_N, 0);
  softPwmWrite(M1_P, 0);
  softPwmWrite(M1_N, 0);
}

double left_speed;
double right_speed;

void motors(double speed, double left_offset, double right_offset)
{
  left_speed = speed + left_offset;
  right_speed = speed + right_offset;

  // left motor
  if (left_speed < 0)  {
    softPwmWrite(M1_P, 90); // 70
    softPwmWrite(M1_N, 0);
  }
  else
  if (left_speed > 0)  {
    softPwmWrite(M1_N, 90);
    softPwmWrite(M1_P, 0);
  }

  // right motor
  if (right_speed < 0)  {
    softPwmWrite(M2_P, 90);
    softPwmWrite(M2_N, 0);
  }
  else
  if (right_speed > 0)  {
    softPwmWrite(M2_N, 90);
    softPwmWrite(M2_P, 0);
  }
}
