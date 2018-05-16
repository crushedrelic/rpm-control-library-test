
#ifndef pm
#define pm
class pMotor
{
public:
  static float calculate_rpm(int NewPosition);
  static float calculate_PID(double set, double input,unsigned int  lastTime);
  
private:
  
  
  
};

  

#endif // !pm


