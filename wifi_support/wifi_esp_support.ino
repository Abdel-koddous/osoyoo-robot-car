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

#define SOFT_RX 4    // Softserial RX port
#define SOFT_TX 5    //Softserial TX port


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

void setup()
{
  
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

  Udp.begin(5005);

}
 
void loop()
{ 
  // Send return packet
  char reply[] = "This is the Robot Car speaking... Distance to obstacle => BlaBlaBla meters...";

  // IP ADDRESS THE UDP Packet is sent to => 192.168.100.139
  IPAddress ip_addr = IPAddress(192, 168, 100, 139);
  uint16_t remote_port = 5005;

  Serial.println("Sending UDP Packet...");
  Udp.beginPacket(ip_addr, remote_port);
  Udp.write(reply);
  Udp.endPacket();

  delay(1000);
} //end of loop
 
void printWifiStatus()
{
  // print your WiFi shield's IP address
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print where to go in the browser
  Serial.println();
  Serial.print("To validate the connection route, use the ping command with the address: ");
  Serial.println(ip);
  Serial.println();
}