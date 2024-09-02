/*
  YAHBOOM G1 Tank Robot (ver. Arduino) : LED example Code
  - File Name : TankRobot_rgbLED_ex2.ino
  - Update : 2024.09.03 (by yangjipsa)
*/ 

int LED_R = 11;
int LED_G = 10;
int LED_B = 9;

void setup() { }

void loop()
{
  for (int i=0 ; i <= 255 ; i++)
  {
    analogWrite(LED_R, i);
    delay(10); // 256 * 10ms = 2,560ms
  }    
}

