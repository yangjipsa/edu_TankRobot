/*
  YAHBOOM G1 Tank Robot (ver. Arduino) : LED example Code
  - File Name : TankRobot_rgbLED_ex1.ino
  - Update : 2024.09.03 (by yangjipsa)
*/ 

int LED_R = 11;
int LED_G = 10;
int LED_B = 9;

void setup()
{
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
}

void loop()
{
  digitalWrite(LED_R, HIGH);
  digitalWrite(LED_G, LOW);
  digitalWrite(LED_B, LOW);
  delay(1000);

  digitalWrite(LED_R, LOW);
  digitalWrite(LED_G, HIGH);
  digitalWrite(LED_B, LOW);
  delay(1000);

  digitalWrite(LED_R, LOW);
  digitalWrite(LED_G, LOW);
  digitalWrite(LED_B, HIGH);
  delay(1000);
}





