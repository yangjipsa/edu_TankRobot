// Source Code Site : http://www.yahboom.net/study/g1-t-ar
/**
* @par Copyright (C): 2010-2019, Shenzhen Yahboom Tech
* @file         bluetooth_control.c
* @author       Danny
* @version      V1.0
* @date         2017.07.25
* @brief        bluetooth_control
* @details
* @par History  
*
*/
#define run_car     '1'
#define back_car    '2'
#define left_car    '3'
#define right_car   '4'
#define stop_car    '0'

#define ON 1           
#define OFF 0          

/*Car running status enumeration*/
enum {
  enSTOP = 0,
  enRUN,
  enBACK,
  enLEFT,
  enRIGHT,
  enTLEFT,
  enTRIGHT
} enCarState;

/*Definition of Pin*/
int Left_motor_go = 8;    
int Left_motor_back = 7;  
int Right_motor_go = 2;   
int Right_motor_back = 4; 
int Left_motor_pwm = 6;   
int Right_motor_pwm = 5; 

//TrackSensorLeftPin1 TrackSensorLeftPin2 TrackSensorRightPin1 TrackSensorRightPin2
//      A2                  A1                  A3                   A4
const int TrackSensorLeftPin1  =  A2;  //The first tracking infrared sensor pin on the left is connected to  wiringPi port A2 of Arduino UNO
const int TrackSensorLeftPin2  =  A1;  //The second tracking infrared sensor pin on the left is connected to  wiringPi port A1 of Arduino UNO
const int TrackSensorRightPin1 =  A3;  //The first tracking infrared sensor pin on the right is connected to  wiringPi port A3 of Arduino UNO
const int TrackSensorRightPin2 =  A4;  //The second tracking infrared sensor pin on the right is connected to  wiringPi port A4 of Arduino UNO

///Define variables to store data collected by each tracking infrared pin
bool TrackSensorLeftValue1;
bool TrackSensorLeftValue2;
bool TrackSensorRightValue1;
bool TrackSensorRightValue2;

String infrared_track_value = "0000";

int buzzer = A0;                //Buzzer connects to wiringPi port 10 of Arduino UNO 


int position = 0; 

/*Voltage pins and their variable*/
int VoltagePin = A0;
int VoltageValue = 0;

int CarSpeedControl = 150;

int ServoPin = 3;

int EchoPin = 12;         //Define the EchoPin connect to port 12 of Arduino UNO
int TrigPin = 13;         //Define the TrigPin connect to port 13 of Arduino UNO
float distance = 0;

int LED_R = 11;           //LED_R is connected to  wiringPi port 3 of Raspberry pi
int LED_G = 10;           //LED_G is connected to  wiringPi port 2 of Raspberry pi
int LED_B = 9;            //LED_B is connected to  wiringPi port 5 of Raspberry pi

int OutfirePin = A5;

int red, green, blue;

int time = 20000;
int count = 10;

/*Serial port data Settings*/
int IncomingByte = 0;            
String InputString = "";         //This array is used to store received content
boolean NewLineReceived = false; 
boolean StartBit  = false;       
String ReturnTemp = "";          

int g_CarState = enSTOP;        //1run 2back 3left 4right 0stop
int g_modeSelect = 0;           //0: default;  
                                //2:tracking mode 3:Ultrasonic obstacle avoidance mode
			        //4: color_led mode 5: light_follow mode 6: infrared_follow mode
boolean g_motor = false;

float voltage_table[21][2] =
{
  {6.46, 676}, {6.51, 678}, {6.61, 683}, {6.72, 687}, {6.82, 691}, {6.91, 695}, {7.01, 700}, {7.11, 703},
  {7.20, 707}, {7.31, 712}, {7.4, 715}, {7.5, 719}, {7.6, 723}, {7.7, 728}, {7.81, 733}, {7.91, 740},
  {8.02, 741}, {8.1, 745}, {8.22, 749}, {8.30, 753}, {8.4, 758}
};

int serial_putc( char c, struct __file * )
{
  Serial.write( c );
  return c;
}
void printf_begin(void)
{
  fdevopen( &serial_putc, 0 );
}

/**
* Function       setup
* @author        Danny
* @date          2017.07.25
* @brief         Initialization configure
* @param[in]     void
* @retval        void
* @par History   
*/
void setup()
{
  Serial.begin(9600);
  printf_begin();
  //Initialize the motor drive IO as the output mode
  pinMode(Left_motor_go, OUTPUT);
  pinMode(Left_motor_back, OUTPUT);
  pinMode(Right_motor_go, OUTPUT);
  pinMode(Right_motor_back, OUTPUT);

  //Initialize the tracksensor IO as the input mode
  pinMode(TrackSensorLeftPin1, INPUT);
  pinMode(TrackSensorLeftPin2, INPUT);
  pinMode(TrackSensorRightPin1, INPUT);
  pinMode(TrackSensorRightPin2, INPUT);

  //Initialize the buzzer IO as the output mode
  pinMode(buzzer, OUTPUT);
  digitalWrite(buzzer, HIGH);

  pinMode(EchoPin, INPUT);   //Initialize the EchoPin as the input mode
  pinMode(TrigPin, OUTPUT);  //Initialize the TrigPin as the output mode

  //Initialize the outfire IO as the output mode
  pinMode(OutfirePin, OUTPUT);
  digitalWrite(OutfirePin, HIGH);

  //Initialize the RGB IO as the output mode
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
  digitalWrite(LED_R, LOW);
  digitalWrite(LED_G, LOW);
  digitalWrite(LED_B, LOW);

  pinMode(ServoPin, OUTPUT);
  
  randomSeed(analogRead(0)); 
}

/**
* Function       servo_pulse
* @author        Danny
* @date          2017.07.26
* @brief         Define a pulse function to generate the PWM value in the analog mode. 
*                The base pulse is 20ms, 
*                and the high level of the pulse is controlled at 0-180 degrees in 0.5-2.5ms.
* @param[in1]    ServPin
* @param[in2]    myangle
* @param[out]    void
* @retval        void
* @par History   
*/
void servo_pulse(int ServoPin, int myangle)
{
  int PulseWidth;                    //Define the pulse width variable
  PulseWidth = (myangle * 11) + 500; //Convert the Angle to 500-2480 pulse width
  digitalWrite(ServoPin, HIGH);      
  delayMicroseconds(PulseWidth);     
  digitalWrite(ServoPin, LOW);       
  delay(20 - PulseWidth / 1000);     //Delay remaining time 
  return;
}

/**
* Function       Distance_test
* @author        Danny
* @date          2017.07.26
* @brief         measure the distance by Ultrasonic
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
void Distance_test()
{
  digitalWrite(TrigPin, LOW);               //Input a low level of 2 us to the Trig pin
  delayMicroseconds(2);
  digitalWrite(TrigPin, HIGH);              //Input a high level of at least 10 US to the Trig pin
  delayMicroseconds(15);
  digitalWrite(TrigPin, LOW);
  float Fdistance = pulseIn(EchoPin, HIGH); 
  Fdistance = Fdistance / 58;
  //  Serial.print("Distance:");            
  //  Serial.print(Fdistance);              
  //  Serial.println("cm");
  distance = Fdistance;
  return;
}

/**
* Function       voltage_test
* @author        Danny
* @date          2017.07.26
* @brief         电池电压引脚检测
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   无
*/
float voltage_test()
{
  pinMode(VoltagePin, INPUT);           
  VoltageValue = analogRead(VoltagePin);
  //Serial.println(VoltageValue);
  float voltage = (VoltageValue * 5 * 2.7)/1024.0f  ;  
  pinMode(VoltagePin, OUTPUT);
  digitalWrite(buzzer, HIGH);
  return voltage;
}



/**
* Function       track_test
* @author        Danny
* @date          2017.07.26
* @brief         track_test
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
void track_test()
{
  // When the black line is detected, the corresponding indicator of the tracking module is on, and the port level is LOW.
  // When the black line is not detected, the corresponding indicator of the tracking module is off, and the port level is HIGH.
  TrackSensorLeftValue1 = digitalRead(TrackSensorLeftPin1);
  TrackSensorLeftValue2 = digitalRead(TrackSensorLeftPin2);
  TrackSensorRightValue1 = digitalRead(TrackSensorRightPin1);
  TrackSensorRightValue2 = digitalRead(TrackSensorRightPin2);

  (TrackSensorLeftValue1 == LOW) ? infrared_track_value[0] = '1' : infrared_track_value[0] = '0';
  (TrackSensorLeftValue2 == LOW) ? infrared_track_value[1] = '1' : infrared_track_value[1] = '0';
  (TrackSensorRightValue1 == LOW) ? infrared_track_value[2] = '1' : infrared_track_value[2] = '0';
  (TrackSensorRightValue2 == LOW) ? infrared_track_value[3] = '1' : infrared_track_value[3] = '0';
  //infrared_track_value = "0000";
  return;
}
/**
* Function       track_get_value
* @author        liusen
* @date          2017.07.26
* @brief         track_get_value
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
void track_get_value()
{
  TrackSensorLeftValue1  = digitalRead(TrackSensorLeftPin1);
  TrackSensorLeftValue2  = digitalRead(TrackSensorLeftPin2);
  TrackSensorRightValue1 = digitalRead(TrackSensorRightPin1);
  TrackSensorRightValue2 = digitalRead(TrackSensorRightPin2);
}

/**
* Function       run
* @author        Danny
* @date          2017.07.25
* @brief         advance
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
void run()
{
  //Left motor advance
  digitalWrite(Left_motor_go, HIGH);  
  digitalWrite(Left_motor_back, LOW); 
  analogWrite(Left_motor_pwm, CarSpeedControl);

  //Right motor advance
  digitalWrite(Right_motor_go, HIGH);  
  digitalWrite(Right_motor_back, LOW); 
  analogWrite(Right_motor_pwm, CarSpeedControl);
}

/**
* Function       brake
* @author        Danny
* @date          2017.07.25
* @brief         brake
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
void brake()
{
  digitalWrite(Left_motor_go, LOW);
  digitalWrite(Left_motor_back, LOW);
  digitalWrite(Right_motor_go, LOW);
  digitalWrite(Right_motor_back, LOW);
}

/**
* Function       left
* @author        Danny
* @date          2017.07.25
* @brief         turn left(left wheel stop,right wheel advance)
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
void left()
{
  //Left motor stop
  digitalWrite(Left_motor_go, LOW);     
  digitalWrite(Left_motor_back, LOW);   
  analogWrite(Left_motor_pwm, 0);

  //Right motor stop
  digitalWrite(Right_motor_go, HIGH);  
  digitalWrite(Right_motor_back, LOW); 
  analogWrite(Right_motor_pwm, CarSpeedControl);
}

/**
* Function       spin_left
* @author        Danny
* @date          2017.07.25
* @brief         turn left in place(left wheel back,right wheel advance)
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
void spin_left()
{
  //Left motor back
  digitalWrite(Left_motor_go, LOW);     
  digitalWrite(Left_motor_back, HIGH);  
  analogWrite(Left_motor_pwm, CarSpeedControl);

  //Right motor advance
  digitalWrite(Right_motor_go, HIGH);  
  digitalWrite(Right_motor_back, LOW); 
  analogWrite(Right_motor_pwm, CarSpeedControl);
}

/**
* Function       right
* @author        Danny
* @date          2017.07.25
* @brief         turn right(left advance,right wheel stop)
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
void right()
{
  //Left motor advance
  digitalWrite(Left_motor_go, HIGH);    
  digitalWrite(Left_motor_back, LOW);  
  analogWrite(Left_motor_pwm, CarSpeedControl);

  //Right motor stop
  digitalWrite(Right_motor_go, LOW);    
  digitalWrite(Right_motor_back, LOW);  
  analogWrite(Right_motor_pwm, 0);
}

/**
* Function       spin_right
* @author        Danny
* @date          2017.07.25
* @brief         turn right (right wheel back,left wheel advance)
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
void spin_right()
{
  //Left motor advance
  digitalWrite(Left_motor_go, HIGH);    
  digitalWrite(Left_motor_back, LOW);   
  analogWrite(Left_motor_pwm, CarSpeedControl);

  //Right motor back
  digitalWrite(Right_motor_go, LOW);    
  digitalWrite(Right_motor_back, HIGH); 
  analogWrite(Right_motor_pwm, CarSpeedControl);
}

/**
* Function       back
* @author        Danny
* @date          2017.07.25
* @brief         back
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
void back()
{
  //Left motor back
  digitalWrite(Left_motor_go, LOW);     
  digitalWrite(Left_motor_back, HIGH);  
  analogWrite(Left_motor_pwm, CarSpeedControl);

  //Right motor back
  digitalWrite(Right_motor_go, LOW);    
  digitalWrite(Right_motor_back, HIGH); 
  analogWrite(Right_motor_pwm, CarSpeedControl);
}

/**
* Function       whistle
* @author        Danny
* @date          2017.07.25
* @brief         whistle
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
void whistle()
{
  pinMode(buzzer, OUTPUT);
  digitalWrite(buzzer, LOW);   //sound
  delay(100);                  
  digitalWrite(buzzer, HIGH);  //no sound
  delay(1);                   

  digitalWrite(buzzer, LOW);   //sound
  delay(200);                 
  digitalWrite(buzzer, HIGH);  //no sound
  delay(2);                  
  return;
}

/**
* Function       servo_appointed_detection
* @author        Danny
* @date          2017.07.25
* @brief         The servo rotates to the specified angle
* @param[in]     pos
* @param[out]    void
* @retval        void
* @par History   
*/
void servo_appointed_detection(int pos)
{
  int i = 0;
  for (i = 0; i <= 30; i++)    //Generate PWM, equivalent delay to ensure that it can be turned to the response angle
  {
    servo_pulse(ServoPin, pos); //Generate the PWM in the analog mode
  }
}

/**
* Function       color_led_pwm
* @author        Danny
* @date          2017.07.25
* @brief         Color_LED light the specified color
* @param[in1]    v_iRed（0-255）
* @param[in2]    v_iGreen（0-255）
* @param[in3]    v_iBlue（0-255）
* @param[out]    void
* @retval        void
* @par History   
*/
void color_led_pwm(int v_iRed, int v_iGreen, int v_iBlue)
{
  analogWrite(LED_R, v_iRed);
  analogWrite(LED_G, v_iGreen);
  analogWrite(LED_B, v_iBlue);
  delay(100);
  return;
}
/**
* Function       color_led
* @author        Danny
* @date          2017.07.25
* @brief         7 different colors formed by different combinations of R,G and B
* @param[in1]    Red
* @param[in2]    Green
* @param[in3]    Blue
* @retval        void
* @par History   
*/

void corlor_led(int v_iRed, int v_iGreen, int v_iBlue)
{
  if (v_iRed == ON)
  {
    digitalWrite(LED_R, HIGH);
  }
  else
  {
    digitalWrite(LED_R, LOW);
  }

  if (v_iGreen == ON)
  {
    digitalWrite(LED_G, HIGH);
  }
  else
  {
    digitalWrite(LED_G, LOW);
  }

  if (v_iBlue == ON)
  {
    digitalWrite(LED_B, HIGH);
  }
  else
  {
    digitalWrite(LED_B, LOW);
  }
}
/********************************************************************************************************/
/*Mode2 Tracking*/
/**
* Function       Tracking_Mode
* @author        Danny
* @date          2017.07.25
* @brief         Tracking
* @param[in1]    void
* @param[out]    void
* @retval        void
* @par History   
*/
void Tracking_Mode()
{
  // When the black line is detected, the corresponding indicator of the tracking module is on, and the port level is LOW.
  // When the black line is not detected, the corresponding indicator of the tracking module is off, and the port level is HIGH.
  TrackSensorLeftValue1  = digitalRead(TrackSensorLeftPin1);
  TrackSensorLeftValue2  = digitalRead(TrackSensorLeftPin2);
  TrackSensorRightValue1 = digitalRead(TrackSensorRightPin1);
  TrackSensorRightValue2 = digitalRead(TrackSensorRightPin2);
  
  //4 tracking pins level status
  // 0 0 X 0
  // 1 0 X 0
  // 0 1 X 0
   //Turn right in place,
   //Handle right acute angle and right right angle 
  if ( (TrackSensorLeftValue1 == LOW || TrackSensorLeftValue2 == LOW) &&  TrackSensorRightValue2 == LOW)
  {
    CarSpeedControl = 145;
    spin_right();
    delay(80);
  }
  //4 tracking pins level status
  // 0 X 0 0
  // 0 X 0 1
  // 0 X 1 0
  else if ( TrackSensorLeftValue1 == LOW && (TrackSensorRightValue1 == LOW ||  TrackSensorRightValue2 == LOW))
  {
    CarSpeedControl = 145;
    spin_left();
    delay(80);
  }
  // 0 X X X
  //Left_sensor1 detected black line
  else if ( TrackSensorLeftValue1 == LOW)
  {
    CarSpeedControl = 110;
    spin_left();
    delay(10);
  }
  // X X X 0
  //Right_sensor2 detected black line
  else if ( TrackSensorRightValue2 == LOW )
  {
    CarSpeedControl = 110;
    spin_right();
    delay(10);
  }
  //4 tracking pins level status
  // X 0 1 X
  else if ( TrackSensorLeftValue2 == LOW && TrackSensorRightValue1 == HIGH)
  {
    CarSpeedControl = 110;
    spin_left();
  }
  //4 tracking pins level status
  // X 1 0 X
  else if (TrackSensorLeftValue2 == HIGH && TrackSensorRightValue1 == LOW)
  {
    CarSpeedControl = 110;
    spin_right();
  }
  //4 tracking pins level status
  // X 0 0 X
  else if (TrackSensorLeftValue2 == LOW && TrackSensorRightValue1 == LOW)
  {
    CarSpeedControl = 150;
    run();
  }
}
/********************************************************************************************************/
/*Mode3: ultrasonic obstacle avoidance mode*/
/**
* Function       servo_color_carstate
* @author        Danny
* @date          2017.07.26
* @brief         servo_color_carstate
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
void servo_color_carstate()
{
  int iServoPos = 0;
  int LeftDistance = 0;   
  int RightDistance = 0;   
  int FrontDistance = 0;   
  corlor_led(ON, OFF, OFF);
  CarSpeedControl= 90;
  back();                //Avoid stop suddenly
  delay(80);
  brake();

  //The servo rotates to 0°(right)to measure distance
  servo_appointed_detection(20);
  delay(500);
  Distance();        
  RightDistance = distance;

  //The servo rotates to 180°(left)to measure distance
  servo_appointed_detection(160);
  delay(500);
  Distance();        
  LeftDistance = distance;

  //The servo rotates to 90°(front)to measure distance
  servo_appointed_detection(90);
  delay(500);

  if (LeftDistance < 20 && RightDistance < 20  )
  {
    corlor_led(OFF, ON, ON);
    CarSpeedControl= 150;
    spin_right();
    delay(1100);
    brake();
  }
  else if ( LeftDistance >= RightDistance) 
  {
    corlor_led(OFF, OFF, ON);
    CarSpeedControl= 150;
    spin_left();
    delay(550);
    brake();
  }
  else if (LeftDistance < RightDistance ) 
  {
    corlor_led(ON, OFF, ON);
    CarSpeedControl= 150;
    spin_right();
    delay(550);
    brake();
  }
}
/**
* Function       bubble
* @author        Danny
* @date          2017.07.26
* @brief         Bubble sorting
* @param[in1]    a:Ultrasonic array first address
* @param[in2]    n:Size of Ultrasonic array 
* @param[out]    void
* @retval        void
* @par History   
*/
void bubble(unsigned long *a, int n)

{
  int i, j, temp;
  for (i = 0; i < n - 1; i++)
  {
    for (j = i + 1; j < n; j++)
    {
      if (a[i] > a[j])
      {
        temp = a[i];
        a[i] = a[j];
        a[j] = temp;
      }
    }
  }
}
/**
* Function       Distance
* @author        Danny
* @date          2017.07.26
* @brief         Remove the maximum, minimum of the 5 datas, and get average values of 3 datas to improve accuracy of test
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
void Distance()
{
  unsigned long ultrasonic[5] = {0};
  int num = 0;
  while (num < 5)
  {
    Distance_test();
    while (distance >= 600 || distance == 0)
    {
      brake();
      Distance_test();
    }
    ultrasonic[num] = distance;
    //printf("L%d:%d\r\n", num, (int)distance);
    num++;
    delay(10);
  }
  num = 0;
  bubble(ultrasonic, 5);
  distance = (ultrasonic[1] + ultrasonic[2] + ultrasonic[3]) / 3;
  return;
}
/**
* Function       Ultrasonic_avoidMode
* @author        Danny
* @date          2017.07.26
* @brief         ultrasonic obstacle avoidance mode
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   无
*/
void Ultrasonic_avoidMode()
{
  Distance();        
   //printf("D:%d\r\n", (int)distance);
  if (distance > 35  )    
  {  
    CarSpeedControl= 150;
    run();
    corlor_led(OFF, ON, OFF);
  }
  else if ((distance >= 25 && distance <= 35))
  {
     CarSpeedControl= 100;
     run();
    
  }
  else if (  distance < 25  )
  {
    servo_color_carstate();
  }
}
/********************************************************************************************************/
/*Mode:4  ColorLED*/

void FindColor_Mode()
{

     servo_appointed_detection(position);
     color_led_pwm( random(0,255), random(0,255), random(0,255));
     position += 10;
     if(position > 180)
     {
       position = 0;
     }
}


/**
* Function       ModeBEEP
* @author        Danny
* @date          2017.08.17
* @brief         
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
void ModeBEEP(int mode)
{
  pinMode(buzzer, OUTPUT);
  for (int i = 0; i < mode + 1; i++)
  {
    digitalWrite(buzzer, LOW); 
    delay(100);
    digitalWrite(buzzer, HIGH); 
    delay(100);
  }
  delay(100);
  digitalWrite(buzzer, HIGH); 
}
/**
* Function       BeepOnOffMode
* @author        Danny
* @date          2017.08.17
* @brief         
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
void BeepOnOffMode()
{
  pinMode(buzzer, OUTPUT);
  digitalWrite(buzzer, LOW);   
  delay(1000);                  
  digitalWrite(buzzer, HIGH);  
}
/**
* Function       serial_data_parse
* @author        Danny
* @date          2017.07.25
* @brief         The serial port data is parsed and the corresponding action is specified
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
void serial_data_parse()
{

  if (InputString.indexOf("MODE") > 0 && InputString.indexOf("4WD") > 0)
  {
    if (InputString[10] == '0') 
    {
      brake();
      g_CarState = enSTOP;
      g_modeSelect = 0;
      //position = 0;
      BeepOnOffMode();
    }
    else
    {
      switch (InputString[9])
      {
        case '0': g_modeSelect = 0; ModeBEEP(0); break;
        case '1': g_modeSelect = 1; ModeBEEP(1); break;
        case '2': g_modeSelect = 2; ModeBEEP(2); break;
        case '3': g_modeSelect = 3; ModeBEEP(3); break;
        case '4': g_modeSelect = 4; ModeBEEP(4); break;
        case '5': g_modeSelect = 5; ModeBEEP(5); break;
        case '6': g_modeSelect = 6; ModeBEEP(6); break;
        default: g_modeSelect = 0; break;
      }
      delay(1000);
      BeepOnOffMode();
    }
    InputString = "";                     
    NewLineReceived = false;
    return;
  }

  if (g_modeSelect != 0) 
  {
    InputString = "";                    
    NewLineReceived = false;
    return;
  }

     //Analyze the control instructions of the servo sent by the host computer and execute corresponding operation
     //For exmaple:$4WD,PTZ180# servo turn 180°
  if (InputString.indexOf("PTZ") > 0)
  {
    int m_kp;
    int i = InputString.indexOf("PTZ"); 
    int ii = InputString.indexOf("#", i);
    if (ii > i)
    {
      String m_skp = InputString.substring(i + 3, ii);
      int m_kp = m_skp.toInt();       
      //      Serial.print("PTZ:");
      //      Serial.println(m_kp);
      servo_appointed_detection(180 - m_kp);
      InputString = "";                     
      NewLineReceived = false;
      return;
    }
  }

    //Analyze the control instructions sent by the host computer and execute corresponding operation
    //For exmaple:$4WD,CLR255,CLG0,CLB0# color_red
  else if (InputString.indexOf("CLR") > 0)
  {
    int m_kp;
    int i = InputString.indexOf("CLR");
    int ii = InputString.indexOf(",", i);
    if (ii > i)
    {
      String m_skp = InputString.substring(i + 3, ii);
      int m_kp = m_skp.toInt();
      //      Serial.print("CLR:");
      //      Serial.println(m_kp);
      red =   m_kp;
    }
    i = InputString.indexOf("CLG");
    ii = InputString.indexOf(",", i);
    if (ii > i)
    {
      String m_skp = InputString.substring(i + 3, ii);
      int m_kp = m_skp.toInt();
      //      Serial.print("CLG:");
      //      Serial.println(m_kp);
      green =   m_kp;
    }
    i = InputString.indexOf("CLB");
    ii = InputString.indexOf("#", i);
    if (ii > i)
    {
      String m_skp = InputString.substring(i + 3, ii);
      int m_kp = m_skp.toInt();
      //      Serial.print("CLB:");
      //      Serial.println(m_kp);
      blue =  m_kp;
      color_led_pwm(red, green, blue);
      InputString = "";               
      NewLineReceived = false;
      return;
    }
  }
  //Analyze the control instructions sent by the host computer and execute corresponding operation
  //For exmaple:$1,0,0,0,0,0,0,0,0,0#    advance
  if (InputString.indexOf("4WD") == -1)
  {
    //小车原地左旋右旋判断
    if (InputString[3] == '1')      
    {
      g_CarState = enTLEFT;
    }
    else if (InputString[3] == '2') 
    {
      g_CarState = enTRIGHT;
    }
    else
    {
      g_CarState = enSTOP;
    }


    if (InputString[5] == '1')     
    {
      whistle();
    }


    if (InputString[7] == '1')     
    {
      CarSpeedControl += 50;
      if (CarSpeedControl > 255)
      {
        CarSpeedControl = 255;
      }
    }
    if (InputString[7] == '2')
    {
      CarSpeedControl -= 50;
      if (CarSpeedControl < 50)
      {
        CarSpeedControl = 100;
      }
    }

    if (InputString[9] == '1') 
    {
      servo_appointed_detection(180);
    }
    if (InputString[9] == '2') 
    {
      servo_appointed_detection(0);
    }

    switch (InputString[13])
    {
      case '1': corlor_led(ON, ON, ON); break;
      case '2': corlor_led(ON, OFF, OFF); break;
      case '3': corlor_led(OFF, ON, OFF); break;
      case '4': corlor_led(OFF, OFF, ON); break;
      case '5': corlor_led(OFF, ON, ON); break;
      case '6': corlor_led(ON, OFF, ON); break;
      case '7': corlor_led(ON, ON, OFF); break;
      case '8': corlor_led(OFF, OFF, OFF); break;
    }


    if (InputString[17] == '1') 
    {
      servo_appointed_detection(90);
    }

    if (g_CarState != enTLEFT && g_CarState != enTRIGHT)
    {
      switch (InputString[1])
      {
        case run_car:   g_CarState = enRUN;  break;
        case back_car:  g_CarState = enBACK;  break;
        case left_car:  g_CarState = enLEFT;  break;
        case right_car: g_CarState = enRIGHT;  break;
        case stop_car:  g_CarState = enSTOP;  break;
        default: g_CarState = enSTOP; break;
      }
    }

    InputString = "";        
    NewLineReceived = false;

    switch (g_CarState)
    {
      case enSTOP: brake(); break;
      case enRUN: run(); break;
      case enLEFT: spin_left(); break;
      case enRIGHT: spin_right(); break;
      case enBACK: back(); break;
      case enTLEFT: spin_left(); break;
      case enTRIGHT: spin_right(); break;
      default: brake(); break;
    }
  }
}

/**
* Function       serial_data_postback
* @author        Danny
* @date          2017.07.25
* @brief         The collected sensor data is transmitted by the serial port to the host computer for display
* @param[in]     void
* @retval        void
* @par History   
*/
void serial_data_postback()
{
  //ultrasonic
  Distance_test();
  ReturnTemp = "$4WD,CSB" ;
  ReturnTemp.concat(distance);
  //voltage
  ReturnTemp += ",PV";
  ReturnTemp.concat( voltage_test());
  //gray level
  ReturnTemp += ",GS";
  ReturnTemp.concat("0");
  //tracking
  ReturnTemp += ",LF";
  track_test();
  ReturnTemp.concat(infrared_track_value);
  //infrared obstacle avoidance
  ReturnTemp += ",HW";
  //infrared_avoid_test();
  ReturnTemp.concat("00");
  //light-seeking
  ReturnTemp += ",GM";
  //follow_light_test();
  ReturnTemp.concat("00");
  ReturnTemp += "#";
  Serial.print(ReturnTemp);
  return;
}

/**
* Function       serialEvent
* @author        Danny
* @date          2017.07.25
* @brief         Serial port parsing data packet
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/

void serialEvent()
{
  while (Serial.available())
  {
    IncomingByte = Serial.read();
    if (IncomingByte == '$')
    {
      StartBit = true;
    }
    if (StartBit == true)
    {
      InputString += (char) IncomingByte;
    }
    if (IncomingByte == '#')
    {
      NewLineReceived = true;
      StartBit = false;
    }
  }
}

/**
* Function       loop
* @author        Danny
* @date          2017.07.25
* @brief         Analyze the control instructions sent by the host computer and execute corresponding operation
* @param[in]     void
* @retval        void
* @par History   
*/
void loop()
{
  if (NewLineReceived)
  {
    serial_data_parse();  
  }

  switch (g_modeSelect)
  {
    case 1: break; 
    case 2: Tracking_Mode(); break; 
    case 3: Ultrasonic_avoidMode();  break;  
    case 4: FindColor_Mode(); break;  
    
  }

  if (g_modeSelect == 0 && g_motor == false)
  {
    time--;
    if (time == 0)
    {
      count--;
      time = 20000;
      if (count == 0)
      {
        
        serial_data_postback();
        time = 20000;
        count = 10;
      }
    }
  }

}





