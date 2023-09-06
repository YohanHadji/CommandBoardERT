#include <Arduino.h>
#include <Capsule.h>
#include <LoopbackStream.h>
#include <Adafruit_NeoPixel.h>
#include "../R2HomeTelemetryInterface/PacketDefinition.h"
#include "mcurses.h"
#include "config.h"

uint32_t colors[] = {
  0xFF0000, // Red
  0x00FF00, // Green
  0x0000FF, // Blue
  0x32A8A0, // Cyan
  0xFFEA00, // Yellow
  0xCF067C, // Purple
  0xFF0800  // Orange
}; 

static TelemetryPacket lastPacket;

void handleUartCapsule(uint8_t packetId, uint8_t *dataIn, uint32_t len);
CapsuleStatic UartCapsule(handleUartCapsule);

Adafruit_NeoPixel led(1, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800); // 1 led

static String cmdMemory;

void printOnSerial();
void putcharOnSerial(uint8_t c);
double distanceTo(double lat1, double lng1, double lat2, double lng2);

void setup() {
  // Initialize USBSerial Monitor
  SERIAL_TO_PC.begin(115200);
  SERIAL_TO_PC.setTxTimeoutMs(0);

  UART_PORT.begin(UART_BAUD, 134217756U, 9, 46);

  led.begin();
  led.fill(colors[1]);
  led.show();

  setFunction_putchar(putcharOnSerial);
  initscr(); 
  curs_set(0);

  printOnSerial();
}
 
void loop() {
  while (UART_PORT.available()) {
    UartCapsule.decode(UART_PORT.read());
  }

  const int cmdBufferSize = 256;
  static uint8_t buff[cmdBufferSize];
  static int buffIndex = 0;
  static bool parsingCmd = false;

  while(SERIAL_TO_PC.available()) {
    digitalWrite(13, HIGH);
    uint8_t a = SERIAL_TO_PC.read();
    if (a == 'l' and !parsingCmd) {
      memcpy(buff, "left", 4);
      size_t codedSize = UartCapsule.getCodedLen(buffIndex);
      byte* coded = UartCapsule.encode(CAPSULE_ID::DEVICE_TO_MOTHER, buff, 4);

      UART_PORT.write(coded, codedSize);
      delete[] coded;
    }
    else if (a == 'r' and !parsingCmd) {
      memcpy(buff, "right", 5);
      size_t codedSize = UartCapsule.getCodedLen(buffIndex);
      byte* coded = UartCapsule.encode(CAPSULE_ID::DEVICE_TO_MOTHER, buff, 5);

      UART_PORT.write(coded, codedSize);
      delete[] coded;
    }
    else if (a == '#') {
      parsingCmd = true;
      cmdMemory = "";
      for (int i = 0; i < cmdBufferSize; i++) {
        buff[i] = 0;
      }
    }
    else if (a != '/') {
      buff[buffIndex++] = a;
      cmdMemory += (char)a;
    }
    else {
      parsingCmd = false;
      cmdMemory = " - Command sent: "+cmdMemory+" - ";
      size_t codedSize = UartCapsule.getCodedLen(buffIndex);
      byte* coded = UartCapsule.encode(CAPSULE_ID::DEVICE_TO_MOTHER, buff, buffIndex);

      UART_PORT.write(coded, codedSize);
      buffIndex = 0;
      delete[] coded;
      for (int i = 0; i < cmdBufferSize; i++) {
        buff[i] = 0;
      }
    }
    printOnSerial();
  }
}



void handleUartCapsule(uint8_t packetId, uint8_t *dataIn, uint32_t len) {
  switch(packetId) {
    case CAPSULE_ID::MOTHER_TO_DEVICE:
    {
      memcpy(&lastPacket, dataIn, sizeof(lastPacket));
      uint32_t ledColor = colors[random(0,8)];
      led.fill(ledColor);
      led.show();
      printOnSerial();
    }
    break;
    default:
    break;
  }
}


void printOnSerial() {
  clear();

  int lineIndex = 5;
  int colIndex = 0;

  colIndex = 10; 

  move(++lineIndex, colIndex); 
  attrset((A_BOLD | F_WHITE | B_BLACK));
  addstr("Status: ");
  attrset((A_NORMAL | F_WHITE | B_BLACK));
  lineIndex++;
  move(++lineIndex, colIndex);
  addstr(("Time:        "+String(lastPacket.timeSecond)).c_str());  
  move(++lineIndex, colIndex);
  addstr(("Flight Mode: "+String(lastPacket.flightMode)).c_str());
  move(++lineIndex, colIndex);
  addstr(("Initialised: "+String((lastPacket.status & 0x01) ? 1 : 0)).c_str());
  move(++lineIndex, colIndex);
  addstr(("Separated:   "+String((lastPacket.status & 0x02) ? 1 : 0)).c_str());
  move(++lineIndex, colIndex);
  addstr(("Deployed:    "+String((lastPacket.status & 0x04) ? 1 : 0)).c_str());
  move(++lineIndex, colIndex);
  addstr(("Wing Opened: "+String((lastPacket.status & 0x08) ? 1 : 0)).c_str());
  move(++lineIndex, colIndex);
  addstr(("Sen Valid:   "+String((lastPacket.status & 0x10) ? 1 : 0)).c_str());
  move(++lineIndex, colIndex);
  addstr(("GPS Valid:   "+String((lastPacket.status & 0x20) ? 1 : 0)).c_str());
  move(++lineIndex, colIndex);
  addstr(("Time Valid:  "+String((lastPacket.status & 0x40) ? 1 : 0)).c_str());

  lineIndex = 5;

  colIndex = colIndex + 40;
  
  move(++lineIndex, colIndex);
  attrset((A_BOLD | F_WHITE | B_BLACK));
  addstr("Position: ");
  attrset((A_NORMAL | F_WHITE | B_BLACK));
  lineIndex++;
  move(++lineIndex, colIndex);
  addstr(("Latitude:  "+String(lastPacket.latitude,7)).c_str());
  move(++lineIndex, colIndex);
  addstr(("Longitude: "+String(lastPacket.longitude,7)).c_str());
  move(++lineIndex, colIndex);
  addstr(("Altitude:  "+String(lastPacket.altitude,1)).c_str());

  // lineIndex = lineIndex+5;

  lineIndex = 5;

  colIndex = colIndex + 40;

  move(++lineIndex, colIndex);
  attrset((A_BOLD | F_WHITE | B_BLACK));
  addstr("Sensor Misc: ");
  attrset((A_NORMAL | F_WHITE | B_BLACK));
  lineIndex++;
  move(++lineIndex, colIndex);
  addstr(("Yaw:       "+String(lastPacket.yaw,1)).c_str());
  move(++lineIndex, colIndex);
  addstr(("RotSpeed:  "+String(lastPacket.rotationSpeed,1)).c_str());
  move(++lineIndex, colIndex);
  addstr(("Z Speed:   "+String(lastPacket.zSpeed,1)).c_str());
  move(++lineIndex, colIndex);
  addstr(("2D Speed:  "+String(lastPacket.twoDSpeed,1)).c_str());
  move(++lineIndex, colIndex);
  addstr(("Temp:      "+String(lastPacket.temperature,1)).c_str());
  move(++lineIndex, colIndex);
  addstr(("Voltage:   "+String(lastPacket.voltage,1)).c_str());

  // lineIndex = lineIndex+5;

  lineIndex = 5;  

  colIndex = colIndex + 40;

  move(++lineIndex, colIndex);
  attrset((A_BOLD | F_WHITE | B_BLACK));
  addstr("Navigation: ");
  attrset((A_NORMAL | F_WHITE | B_BLACK));
  lineIndex++;
  move(++lineIndex, colIndex);
  addstr(("Waypoint Latitude:     "+String(lastPacket.waypointLatitude,7)).c_str());
  move(++lineIndex, colIndex);
  addstr(("Waypoint Longitude:    "+String(lastPacket.waypointLongitude,7)).c_str());
  move(++lineIndex, colIndex);
  addstr(("Waypoint Altitude:     "+String(lastPacket.waypointAltitude,1)).c_str());
  lineIndex++;
  move(++lineIndex, colIndex);
  addstr(("Trajectory Latitude:   "+String(lastPacket.trajectoryLatitude,7)).c_str());
  move(++lineIndex, colIndex);
  addstr(("Trajectory Longitude:  "+String(lastPacket.trajectoryLongitude,7)).c_str());
  lineIndex++;
  move(++lineIndex, colIndex);

  float distanceToTrajectory;
  float distanceToPosition;

  distanceToTrajectory = distanceTo(lastPacket.trajectoryLatitude, lastPacket.trajectoryLongitude, lastPacket.waypointLatitude, lastPacket.waypointLongitude);
  distanceToPosition = distanceTo(lastPacket.latitude, lastPacket.longitude, lastPacket.waypointLatitude, lastPacket.waypointLongitude);

  addstr(("Distance to Traj:      "+String(distanceToTrajectory,1)).c_str());
  move(++lineIndex, colIndex);
  addstr(("Distance to Position:  "+String(distanceToPosition,1)).c_str());
  double distanceRatio = distanceToTrajectory/(lastPacket.altitude-lastPacket.waypointAltitude);
  move(++lineIndex, colIndex);
  addstr(("Distance Ratio:        "+String(distanceRatio,1)).c_str());

  move(++lineIndex, colIndex);
  attrset((A_BOLD | F_WHITE | B_BLACK));
  addstr("Control: ");
  attrset((A_NORMAL | F_WHITE | B_BLACK));
  lineIndex++;
  move(++lineIndex, colIndex);
  addstr(("Relative Heading Setpoint: "+String(lastPacket.relativeHeadingSetpoint,1)).c_str());
  move(++lineIndex, colIndex);
  addstr(("Relative Heading:          "+String(lastPacket.yaw,1)).c_str());
  move(++lineIndex, colIndex);
  addstr(("Projected Ground Speed:    "+String(lastPacket.projectedGroundSpeed,1)).c_str());

  lineIndex = lineIndex+5;
  colIndex = 10;

  move(++lineIndex, colIndex);
  attrset((A_BOLD | F_WHITE | B_BLACK));
  addstr("Uplink: ");
  attrset((A_NORMAL | F_WHITE | B_BLACK));
  lineIndex++;
  move(++lineIndex, colIndex);
  addstr(("Command: "+cmdMemory).c_str());

}

void putcharOnSerial(uint8_t c) {
  SERIAL_TO_PC.write(c);
}

double distanceTo(double lat1, double lng1, double lat2, double lng2) {
  double R = 6371000;
  lat1 = lat1 * PI / 180.0;
  lng1 = lng1 * PI / 180.0;
  lat2 = lat2 * PI / 180.0;
  lng2 = lng2 * PI / 180.0;

  double dlat = lat2-lat1;
  double dlng = lng2-lng1;

  double a = sin(dlat/2) * sin(dlat/2) + cos(lat1) * cos(lat2) * sin(dlng/2) * sin(dlng/2);
  double c = 2 * atan2(sqrt(a), sqrt(1-a));
  double d = R * c;
  return d;
}

