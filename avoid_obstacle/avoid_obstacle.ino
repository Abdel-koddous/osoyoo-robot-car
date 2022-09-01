#include<Servo.h>

Servo Myservo;

#define echoPin 2 // attach pin D2 Arduino to pin Echo of HC-SR04
#define trigPin 1 //attach pin D3 Arduino to pin Trig of HC-SR04

// defines variables
long duration; // variable for the duration of sound wave travel
int distance; // variable for the distance measurement

unsigned long previousMillis = 0;
unsigned long interval = 200;
unsigned long ServoInterval = 200;
unsigned long ServoPreviousMillis = 0;
unsigned char ServoAnglesIdx = 0;
int numberOfAngles = 181;
//int servoAngles[numberOfAngles];

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
/*
void getServoAngles()
{
  for (int angle = 0; angle <= 180; angle++) {
    servoAngles[angle] = angle;
}
*/
void setup() {
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPin, INPUT); // Sets the echoPin as an INPUT
  Serial.begin(9600); // // Serial Communication is starting with 9600 of baudrate speed
  Serial.println("Ultrasonic Sensor HC-SR04 Test"); // print some text in Serial Monitor
  Serial.println("with Arduino UNO R3");

  Myservo.attach(13);
  Myservo.write(180); // Setup intial position of the servo
  //getServoAngles();
}

void loop() {
  // put your main code here, to run repeatedly:
  unsigned long currentMillis = millis();
  
  if (currentMillis - ServoPreviousMillis >= ServoInterval)
  {
    ServoPreviousMillis = currentMillis;
    
    Serial.print("Angle => ");
    Serial.println(ServoAnglesIdx);
    Serial.println(ServoAnglesIdx);
    //Serial.println(servoAngles[ServoAnglesIdx % numberOfAngles]);

    Myservo.write(ServoAnglesIdx);
    ServoAnglesIdx = ServoAnglesIdx + 5;
    if (ServoAnglesIdx > 180)
    {
      ServoAnglesIdx = 0;
    }
  }

  if (currentMillis - previousMillis >= interval) 
  {
    previousMillis = currentMillis;
    Serial.print("Timestamp: ");
    Serial.print(previousMillis);
    Serial.println("ms =>");
    getDistance();

    Serial.print("Servo ReadMicroseconds => ");
    Serial.println(Myservo.readMicroseconds());
  }



}
