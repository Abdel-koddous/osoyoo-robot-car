/*  ___   ___  ___  _   _  ___   ___   ____ ___  ____  
 * / _ \ /___)/ _ \| | | |/ _ \ / _ \ / ___) _ \|    \ 
 *| |_| |___ | |_| | |_| | |_| | |_| ( (__| |_| | | | |
 * \___/(___/ \___/ \__  |\___/ \___(_)____)___/|_|_|_|
 *                  (____/ 
 * Osoyoo Wifi Arduino Robot Car project
 * USe WI-FI Udp protocol to control robot car
 * tutorial url: https://osoyoo.com/?p=32758
 */
/*Declare L298N Dual H-Bridge Motor Controller directly since there is not a library to load.*/
#include <WiFiEspUdp.h>
WiFiEspUDP Udp;
unsigned int localPort = 8888;  // local port to listen on

//Define L298N Dual H-Bridge Motor Controller Pins
#define RightDirectPin1  12    //Right Motor direction pin 1 to MODEL-X IN1 grey
#define RightDirectPin2  11    //Right Motor direction pin 1 to MODEL-X IN2 yellow
#define speedPinL 6    //Left PWM pin connect MODEL-X ENB brown
#define LeftDirectPin1  7    //Left Motor direction pin 1 to MODEL-X IN3 green
#define LeftDirectPin2  8   //Left Motor direction pin 1 to MODEL-X IN4 white
#define speedPinR 3    // RIGHT PWM pin connect MODEL-X ENA blue
#define SOFT_RX 4    // Softserial RX port
#define SOFT_TX 5    //Softserial TX port


/*From left to right, connect to D3,A1-A3 ,D10*/
#define LFSensor_0 A0  //OLD D3
#define LFSensor_1 A1
#define LFSensor_2 A2
#define LFSensor_3 A3
#define LFSensor_4 A4  //OLD D10

#define SERVO_PIN     9  //servo connect to D9

#define Echo_PIN   2 // Ultrasonic Echo pin connect to D2
#define Trig_PIN   10  // Ultrasonic Trig pin connect to D10
#define LEFT_TURN_TIME 300
#define RIGHT_TURN_TIME 300
#define AHEAD_TIME 300
#define BACK_TIME 500

#define BUZZ_PIN     13  //buzzer connect to D13
#define FAST_SPEED 180
#define MID_SPEED 130
int track_speed = 100 ;    //tracking speed
#define SPEED   120  //avoidance motor speed
#define SPEED_LEFT  255
#define SPEED_RIGHT  255
#define BACK_SPEED  200     //back speed
#define TURN_SPEED  200     //back speed

#define TRACK_SPEED   150  //line follow motor speed

int leftscanval, centerscanval, rightscanval, ldiagonalscanval, rdiagonalscanval;
int distancelimit = 30; //distance limit for obstacles in front           
int sidedistancelimit = 30; //minimum distance in cm to obstacles at both sides (the car will allow a shorter distance sideways)

const int turntime = 300; //Time the robot spends turning (miliseconds)
const int backtime = 300; //Time the robot spends turning (miliseconds)
int distance;
int numcycles = 0;

int thereis;
bool flag1=false;
bool stopFlag = true;
bool JogFlag = false;
uint16_t JogTimeCnt = 0;
uint32_t JogTime=0;

#define MAX_PACKETSIZE 32    //Serial receive buffer
char buffUART[MAX_PACKETSIZE];
unsigned int buffUARTIndex = 0;
unsigned long preUARTTick = 0;

enum DS
{
  MANUAL_DRIVE,
  AUTO_DRIVE_LF, //line follow
  AUTO_DRIVE_UO  //ultrasonic obstruction
}Drive_Status=MANUAL_DRIVE;

enum DN
{ 
  GO_ADVANCE, 
  GO_LEFT, 
  GO_RIGHT,
  GO_BACK,
  STOP_STOP,
  DEF
}Drive_Num=DEF;
String WorkMode="?";

//String toggleStr="<form action=\"/\" method=GET><input type=submit name=a value=L><input type=submit name=a value=U><input type=submit name=a value=D><input type=submit name=a value=R></form>";
#define DEBUG true
 
#include "WiFiEsp.h"
// Emulate Serial1 on pins 9/10 by default
// If you want to use Hard Serial1 in Mega2560 , please remove the wifi shield jumper cap on ESP8266 RX/TX PIN , CONNECT TX->D18 RX->D19
#ifndef HAVE_HWSERIAL1
#include "SoftwareSerial.h"
SoftwareSerial Serial1(SOFT_RX, SOFT_TX); // RX, TX
#endif

char ssid[] = "MAISON";   // replace *** with your router wifi SSID (name)
char pass[] = "tagharte@123";   // replace *** with your router wifi password
 char packetBuffer[5];      
int status = WL_IDLE_STATUS;     // the Wifi radio's status
int connectionId;

#include <Servo.h>
Servo head;
 

// use a ring buffer to increase speed and reduce memory allocation
RingBuffer buf(8);


void buzz_ON()   //open buzzer
{
   for(int i=0;i<100;i++)
  {
   digitalWrite(BUZZ_PIN,LOW);
   delay(2);//wait for 1ms
   digitalWrite(BUZZ_PIN,HIGH);
   delay(2);//wait for 1ms
  }
}
void buzz_OFF()  //close buzzer
{
  digitalWrite(BUZZ_PIN, HIGH);
}

void alarm(){
   buzz_ON();
 
   buzz_OFF();
}

void setup()
{
   
    pinMode(RightDirectPin1, OUTPUT); 
  pinMode(RightDirectPin2, OUTPUT); 
  pinMode(speedPinL, OUTPUT);  
  pinMode(LeftDirectPin1, OUTPUT);
  pinMode(LeftDirectPin2, OUTPUT); 
  pinMode(speedPinR, OUTPUT); 
  /*init HC-SR04*/
  pinMode(Trig_PIN, OUTPUT); 
  pinMode(Echo_PIN,INPUT); 
  /*init buzzer*/
  pinMode(BUZZ_PIN, OUTPUT);
  digitalWrite(BUZZ_PIN, HIGH);  
  buzz_OFF(); 

  digitalWrite(Trig_PIN,LOW);
  /*init servo*/
  
  head.attach(SERVO_PIN); 
  head.write(90);
 
   delay(2000);
   
   Serial.begin(9600);   // initialize serial for debugging
 
  Serial1.begin(115200);    // initialize serial for ESP module
  Serial1.print("AT+CIOBAUD=9600\r\n");
      Serial1.write("AT+RST\r\n");
   Serial1.begin(9600);    // initialize serial for ESP module
  
  WiFi.init(&Serial1);    // initialize ESP module

  // check for the presence of the shield
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    while (true); // don't continue
  }

 // Serial.print("Attempting to start AP ");
//  Serial.println(ssid);
  //AP mode
  //status = WiFi.beginAP(ssid, 10, "", 0);

//STA mode
   while ( status != WL_CONNECTED) {
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network
    status = WiFi.begin(ssid, pass);
  }
 

  Serial.println("You're connected to the network");
  Serial.println();



  printWifiStatus();
  
  Udp.begin(localPort);
  
  Serial.print("Listening on port ");
  Serial.println(localPort);
 
 
}
 
boolean flag=false;
void loop()
{ 
 
  
  int packetSize = Udp.parsePacket();
  if (packetSize) {                               // if you get a client,
     Serial.print("Received packet of size ");
    Serial.println(packetSize);
    int len = Udp.read(packetBuffer, 255);
    if (len > 0) {
      packetBuffer[len] = 0;
    }
      char c=packetBuffer[0];
            
    // Send return packet
    char reply[] = "Packet received!";
    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
    Udp.write(reply);
    Udp.endPacket();
  }

  Serial.println("Still nothing received...");
  delay(200);

 
 
} //end of loop
 
void printWifiStatus()
{
  // print your WiFi shield's IP address
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print where to go in the browser
  Serial.println();
  Serial.print("To see this page in action, connect to ");
  Serial.print(ssid);
  Serial.print(" and open a browser to http://");
  Serial.println(ip);
  Serial.println();
}