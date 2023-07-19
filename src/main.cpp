#include <esp_now.h>
#include <WiFi.h>
#include <Capsule.h>
#include <LoopbackStream.h>
#include <Adafruit_NeoPixel.h>
#include "../ERT_RF_Protocol_Interface/PacketDefinition.h"
#include "../ERT_RF_Protocol_Interface/ParameterDefinition.h"

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

void handleEspNow(const uint8_t * mac, const uint8_t *incomingData, int len);
LoopbackStream ESPNowRxBuffer(1024);

void handleBinoculars(uint8_t packetId, uint8_t *dataIn, uint32_t len);
void handleUartCapsule(uint8_t packetId, uint8_t *dataIn, uint32_t len);

CapsuleStatic UartCapsule(handleUartCapsule);
CapsuleStatic Binoculars(handleBinoculars);

Adafruit_NeoPixel led(1, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800); // 1 led

void setup() {
  // Initialize USBSerial Monitor
  SERIAL_TO_PC.begin(115200);
  SERIAL_TO_PC.setTxTimeoutMs(0);

  UART_PORT.begin(UART_BAUD, 134217756U, 9, 46);

  led.begin();
  led.fill(colors[1]);
  led.show();
  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  esp_now_init();
  esp_now_register_recv_cb(handleEspNow);
}
 
void loop() {
  while(ESPNowRxBuffer.available()) {
    Binoculars.decode(ESPNowRxBuffer.read());
  }
}

// callback function that will be executed when data is received
void handleEspNow(const uint8_t * mac, const uint8_t *incomingData, int len) {
  for (int i = 0; i < len; i++) {
    ESPNowRxBuffer.write(incomingData[i]);
  }  
}

void handleBinoculars(uint8_t packetId, uint8_t *dataIn, uint32_t len) {
  switch (packetId) {
    case CAPSULE_ID::BINOC_GLOBAL_STATUS:
    case CAPSULE_ID::BINOC_ATTITUDE:
    case CAPSULE_ID::BINOC_POSITION:
    case CAPSULE_ID::BINOC_STATUS:
    {
      uint8_t* packetToSend = UartCapsule.encode(packetId,dataIn,len);
      UART_PORT.write(packetToSend,UartCapsule.getCodedLen(len));
      delete[] packetToSend;
    }
    break;
  }
}

void handleUartCapsule(uint8_t packetId, uint8_t *dataIn, uint32_t len) {

}