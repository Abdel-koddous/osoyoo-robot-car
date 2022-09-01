#include<Servo.h>

Servo Myservo;

void setup()
{
Myservo.attach(13);
}









void scan_surroundings(){
  Myservo.write(180);
  
  delay(500);
  
  
  Myservo.write(135);
  
  delay(500);
  
  Myservo.write(90);
  
  delay(500);
  
  
  Myservo.write(45);
  
  delay(500);
  
  Myservo.write(0);
  
  delay(500);
  
}

void loop(){
  scan_surroundings();
}
