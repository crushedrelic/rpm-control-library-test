#include"pMotor.h"
#include"Arduino.h"


const float kp = 0.01, ki = 0.05, kd = 1.5;
 static int oldPosition = 0;
int count = 0 ,start = 0,timeChange,SampleTime=500,lastInput=0;
int output = 0;
unsigned long time_start = millis();
unsigned long time_stop;
float rpm = 0;
double error,dInput,errSum;


float pMotor::calculate_rpm( int new_Position) {
  if ((new_Position != oldPosition))
  
  {   
    if ((new_Position - oldPosition) >= 1920 || (new_Position - oldPosition) <= -1920)
    {
      count++;
      if (count != start)
      {
        time_stop = millis();
        rpm = (60000 / (time_stop - time_start));
        time_start = time_stop;

      }
      oldPosition = new_Position;

    }
  }
  return rpm;


}
float pMotor::calculate_PID(double set, double input,unsigned int  lastTime) {
     unsigned long now = millis();
    timeChange = now - lastTime;
   // Serial.println("burda");
  if (timeChange>=SampleTime) {
    
    error = set - input;
    errSum = errSum + error;
    dInput =input-lastInput;
    output = kp*error + ki*errSum - kd*dInput;
    if (output >= 1023)
    {
      output = 1023;
    }
    if (output <= -1023)
    {
      output = -1023;
    }
  }
  lastInput = input;
  lastTime = now;
 return output;

}


