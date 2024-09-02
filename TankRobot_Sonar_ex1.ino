/*
  YAHBOOM G1 Tank Robot (ver. Arduino) : Sonar sensor example Code
  - File Name : TankRobot_Sonor_ex1.ino
  - Update : 2024.09.03 (by yangjipsa)
*/ 

int EchoPin = 12;
int TrigPin = 13;
float distance = 0;

void setup()
{
  Serial.begin(9600);

  pinMode(EchoPin, INPUT);
  pinMode(TrigPin, OUTPUT);
}

void loop()
{
  Distance_test();
  Serial.print("Distance : ");
  Serial.print(distance);
  Serial.println("cm");
  delay(100);
}

void Distance_test()
{
  digitalWrite(TrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(TrigPin, HIGH);
  delayMicroseconds(15);
  digitalWrite(TrigPin, LOW);
  float Fdistance = pulseIn(EchoPin, HIGH); 
  // Vsound = 34300cm/s
  Fdistance = (Fdistance / 2) * 0.0343; 
  distance = Fdistance;
  return;
}


