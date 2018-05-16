#include<Encoder.h>
#include"pMotor.h"

Encoder myEnc(5,6);
pMotor moto;
int set=100; 
#define enA 9
#define in1 7
#define in2 8
void setup() {
  // put your setup code here, to run once:
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  
  // Set initial rotation direction
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  //
  Serial.begin(9600);
}
//int timeChange,SampleTime=200,lastInput;
unsigned int  lastTime=millis();
//const float kp = 0.01, ki = 0.05, kd = 1.5;
//float output;
//double error,dInput,errSum;
void loop() {
  
  // put your main code here, to run repeatedly:
  long newPosition = myEnc.read();
  int pos = moto.calculate_rpm(newPosition);
  Serial.print(pos); Serial.print("     ");
  int pid= moto.calculate_PID(set,pos,lastTime);
  Serial.println(pid);
  int pwmOutput = map(pid ,-1023, 1023, 60 , 255);
  analogWrite(enA,pwmOutput);
}
/*float calculate_PID(double set, double input,unsigned int  lastTime) {
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

}*/
