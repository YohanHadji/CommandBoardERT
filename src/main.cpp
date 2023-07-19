// Complete Instructions to Get and Change ESP MAC Address: https://RandomNerdTutorials.com/get-change-esp32-esp8266-mac-address-arduino/
#include <Arduino.h>
#include <WiFi.h>

#define SERIAL_TO_PC USBSerial
 
void setup(){
  SERIAL_TO_PC.begin(115200);
  WiFi.mode(WIFI_MODE_STA);
  delay(10000);
  SERIAL_TO_PC.println(WiFi.macAddress());
}
 
void loop(){

}