/*
  YAHBOOM G1 Tank Robot (ver. Arduino) : LED example Code
  - File Name : TankRobot_DCmotor_ex1.ino
  - Update : 2024.09.03 (by yangjipsa)
*/ 

int Left_motor_go = 8;    
int Left_motor_back = 7;  
int Right_motor_go = 2;   
int Right_motor_back = 4; 
int Left_motor_pwm = 6;   
int Right_motor_pwm = 5; 

int CarSpeedControl = 150;

void setup()
{
  pinMode(Left_motor_go, OUTPUT);
  pinMode(Left_motor_back, OUTPUT);
  pinMode(Right_motor_go, OUTPUT);
  pinMode(Right_motor_back, OUTPUT);
}

void loop()
{
  //go forward (LEFT, RIGHT)
  digitalWrite(Left_motor_go, HIGH); digitalWrite(Left_motor_back, LOW); 
  analogWrite(Left_motor_pwm, CarSpeedControl);
  digitalWrite(Right_motor_go, HIGH); digitalWrite(Right_motor_back, LOW); 
  analogWrite(Right_motor_pwm, CarSpeedControl);

  //stop (LEFT, RIGHT)
  digitalWrite(Left_motor_go, LOW); digitalWrite(Left_motor_back, LOW);
  digitalWrite(Right_motor_go, LOW); digitalWrite(Right_motor_back, LOW);

  //spin right (LEFT, RIGHT)
  digitalWrite(Left_motor_go, HIGH); digitalWrite(Left_motor_back, LOW);   
  analogWrite(Left_motor_pwm, CarSpeedControl);
  digitalWrite(Right_motor_go, LOW); digitalWrite(Right_motor_back, HIGH); 
  analogWrite(Right_motor_pwm, CarSpeedControl);

  //stop (LEFT, RIGHT)
  digitalWrite(Left_motor_go, LOW); digitalWrite(Left_motor_back, LOW);
  digitalWrite(Right_motor_go, LOW); digitalWrite(Right_motor_back, LOW);
}





