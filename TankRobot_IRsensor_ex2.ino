/*
  YAHBOOM G1 Tank Robot (ver. Arduino) : IR sensor example Code
  - File Name : TankRobot_IRsensor_ex1.ino
  - Update : 2024.09.03 (by yangjipsa)
*/ 

const int TrackSensorLeftPin1  =  A2;
const int TrackSensorLeftPin2  =  A1;
const int TrackSensorRightPin1 =  A3;
const int TrackSensorRightPin2 =  A4;

int TrackSensorLeftValue1;
int TrackSensorLeftValue2;
int TrackSensorRightValue1;
int TrackSensorRightValue2;

void setup()
{
  Serial.begin(9600);

  // pinMode(TrackSensorLeftPin1, INPUT);
  // pinMode(TrackSensorLeftPin2, INPUT);
  // pinMode(TrackSensorRightPin1, INPUT);
  // pinMode(TrackSensorRightPin2, INPUT);
}

void loop()
{
  TrackSensorLeftValue1  = analogRead(TrackSensorLeftPin1);
  TrackSensorLeftValue2  = analogRead(TrackSensorLeftPin2);
  TrackSensorRightValue1 = analogRead(TrackSensorRightPin1);
  TrackSensorRightValue2 = analogRead(TrackSensorRightPin2);

  Serial.print(TrackSensorLeftValue1); Serial.print(" | ");
  Serial.print(TrackSensorLeftValue2); Serial.print(" | ");
  Serial.print(TrackSensorRightValue1); Serial.print(" | ");
  Serial.println(TrackSensorRightValue2);
  delay(20);
}




