/*
  YAHBOOM G1 Tank Robot (ver. Arduino) : LED example Code
  - File Name : TankRobot_rgbLED_ex3.ino
  - Update : 2024.09.03 (by yangjipsa)
*/ 

int LED_R = 11;
int LED_G = 10;
int LED_B = 9;

void setup() { }

void loop()
{
  int r = random(0, 256); // 0부터 255 사이의 랜덤 값 생성
  int g = random(0, 256); 
  int b = random(0, 256); 

  color_led_pwm(r, g, b); // 랜덤 색상을 RGB LED에 적용
}

void color_led_pwm(int v_iRed, int v_iGreen, int v_iBlue)
{
  analogWrite(LED_R, v_iRed);
  analogWrite(LED_G, v_iGreen);
  analogWrite(LED_B, v_iBlue);
  delay(100); // 100ms 지연 후 색상 변경
}

