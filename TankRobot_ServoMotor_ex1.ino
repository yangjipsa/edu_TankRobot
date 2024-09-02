/*
  YAHBOOM G1 Tank Robot (ver. Arduino) : LED example Code
  - File Name : TankRobot_ServoMotor_ex1.ino
  - Update : 2024.09.03 (by yangjipsa)
*/ 


int ServoPin = 3;

void setup()
{
  pinMode(ServoPin, OUTPUT);
}

void loop()
{
  for (int i=10 ; i<170 ; i++)
  {
    servo_pulse(ServoPin, i);
    delay(2); // 2ms * 160 = 3200ms
  }

  for (int i=10 ; i<170 ; i++)
  {
    servo_pulse(ServoPin, 180 - i);
    delay(2); // 2ms * 160 = 3200ms
  }
}

void servo_pulse(int ServoPin, int myangle)
{
  int PulseWidth;
//Convert the Angle to 500-2480 pulse width
//  PulseWidth = (myangle * 11) + 500;
  PulseWidth = map(myangle, 0, 180, 1000, 2000);
  digitalWrite(ServoPin, HIGH);      
  delayMicroseconds(PulseWidth);     
  digitalWrite(ServoPin, LOW);       
//  delay(20 - PulseWidth / 1000);
  delayMicroseconds(20000 - PulseWidth);
}




