#include<Servo.h>

Servo Myservo;

#define echoPin 2 // attach pin D2 Arduino to pin Echo of HC-SR04
#define trigPin 1 //attach pin D3 Arduino to pin Trig of HC-SR04

#define speedPinR 9    //  RIGHT PWM pin connect MODEL-X ENA
#define RightMotorDirPin1  12    //Right Motor direction pin 1 to MODEL-X IN1 
#define RightMotorDirPin2  11    //Right Motor direction pin 2 to MODEL-X IN2
#define speedPinL 6    // Left PWM pin connect MODEL-X ENB
#define LeftMotorDirPin1  7    //Left Motor direction pin 1 to MODEL-X IN3 
#define LeftMotorDirPin2  8   //Left Motor direction pin 1 to MODEL-X IN4 

// defines variables
long duration; // variable for the duration of sound wave travel
int distance; // variable for the distance measurement

//unsigned long previousMillis = 0;
//unsigned long interval = 100;
unsigned long ServoInterval = 100;
unsigned long ServoPreviousMillis = 0;
unsigned char ServoAnglesIdx = 0;
int rotationSign = 1;
int minDistanceAngle[2] = {100, 0}; // {distanceToObstacle in cm, angle}
int distanceToObstacle = 10;
bool alignRobot = true;

int getDistance()
{
  // Clears the trigPin condition
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
  // Displays the distance on the Serial Monitor
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  return distance;
}

void stop_Stop()    //Stop
{
  digitalWrite(RightMotorDirPin1, LOW);
  digitalWrite(RightMotorDirPin2,LOW);
  digitalWrite(LeftMotorDirPin1,LOW);
  digitalWrite(LeftMotorDirPin2,LOW);
}

//Pins initialize
void init_GPIO()
{
  pinMode(RightMotorDirPin1, OUTPUT); 
  pinMode(RightMotorDirPin2, OUTPUT); 
  pinMode(speedPinL, OUTPUT);  
 
  pinMode(LeftMotorDirPin1, OUTPUT);
  pinMode(LeftMotorDirPin2, OUTPUT); 
  pinMode(speedPinR, OUTPUT); 
  //stop_Stop();
}

void go_Right(int t=0)  //Turn right
{
  digitalWrite(RightMotorDirPin1, LOW);
  digitalWrite(RightMotorDirPin2,HIGH);
  digitalWrite(LeftMotorDirPin1,HIGH);
  digitalWrite(LeftMotorDirPin2,LOW);
  analogWrite(speedPinL,200);
  analogWrite(speedPinR,200);
  delay(t);
}



void setup() {
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPin, INPUT); // Sets the echoPin as an INPUT
  Serial.begin(9600); // // Serial Communication is starting with 9600 of baudrate speed
  Serial.println("Ultrasonic Sensor HC-SR04 Test"); // print some text in Serial Monitor
  Serial.println("with Arduino UNO R3");

  Myservo.attach(13);
  Myservo.write(0); // Setup intial position of the servo
  
  init_GPIO();
  go_Right(2500);
  stop_Stop();//Stop
  
  delay(2000); // Wait for setup config to be applied
}

void loop() {
  // put your main code here, to run repeatedly:
  unsigned long currentMillis = millis();
  /*
  // Servo Control Timer
  if (currentMillis - ServoPreviousMillis >= ServoInterval)
  {
    ServoPreviousMillis = currentMillis;
    
    Serial.print("Timestamp: ");
    Serial.print(ServoPreviousMillis);
    Serial.println("ms =>");
    distanceToObstacle = getDistance();
    if (distanceToObstacle < minDistanceAngle[0])
    {
      minDistanceAngle[0] = distanceToObstacle;
      minDistanceAngle[1] = ServoAnglesIdx;
    }    

    Serial.print("Angle => ");
    Serial.println(ServoAnglesIdx);
    Myservo.write(ServoAnglesIdx);

    // Updating servo angle
    ServoAnglesIdx = ServoAnglesIdx + rotationSign * 5;
    // Changing rotation sign when needed
    if (ServoAnglesIdx >= 180  || ServoAnglesIdx <= 0)
    {
      rotationSign = -rotationSign;
      Serial.println("Done Scanning Full Front Range...");
      Serial.print("Minimum Detected Distance To Obstacle => ");
      Serial.println(minDistanceAngle[0]);
      Serial.print("Angle of Minimum Detected Distance => ");
      Serial.println(minDistanceAngle[1]);
    
      // Aligning Robot using Angle of Minimum Detected Distance
      if (alignRobot == true)
      {
        go_Right(500);
        stop_Stop();//Stop
        alignRobot = false;
      }
    /*  
      Serial.println("Starting The Scan in opposite direction...");
      minDistanceAngle[0] = 100;
    } 
  }*/ 



  
}
