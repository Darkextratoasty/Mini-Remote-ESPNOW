#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include "Adafruit_seesaw.h"
#include "SPI.h"

//#define DEBUG_ENABLED
#define LOOP_RATE 10

Adafruit_seesaw ss(&Wire1);

#define BUTTON_START    16
#define BUTTON_SELECT    0
#define BUTTON_Y         2
#define BUTTON_X         6
#define BUTTON_A         5
#define BUTTON_B         1
uint32_t button_mask = (1UL << BUTTON_X) | (1UL << BUTTON_Y) | (1UL << BUTTON_START) |
                       (1UL << BUTTON_A) | (1UL << BUTTON_B) | (1UL << BUTTON_SELECT);

// Must match the receiver structure
typedef struct struct_message {
  int joyX;
  int joyY;
  char buttonState;
  char checksum;
} struct_message;

// REPLACE WITH YOUR RECEIVER MAC Address
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {}

void setup() {
#ifdef DEBUG_ENABLED
  Serial.begin(115200);
  while(!Serial) {
    delay(10);
  }
#endif
  
  if(!ss.begin(0x50)){
#ifdef DEBUG_ENABLED
    Serial.println("ERROR! seesaw not found");
#endif
    while(1) delay(1);
  }
  Wire1.setClock(400000);
  uint32_t version = ((ss.getVersion() >> 16) & 0xFFFF);
  if (version != 5743) {
#ifdef DEBUG_ENABLED
    Serial.print("Wrong firmware loaded? ");
    Serial.println(version);
#endif
    while(1) delay(10);
  }
  
  ss.pinModeBulk(button_mask, INPUT_PULLUP);
  ss.setGPIOInterrupts(button_mask, 1);

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
#ifdef DEBUG_ENABLED
    Serial.println("Error initializing ESP-NOW");
#endif
    return;
  }
  esp_now_register_send_cb(OnDataSent);
  
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
#ifdef DEBUG_ENABLED
    Serial.println("Failed to add peer");
#endif
    return;
  }
}

void loop() {
  static unsigned long loopStart = millis();
  
  while(millis() - loopStart < (1000/LOOP_RATE));
  loopStart = millis();
  
  struct_message data;

  unsigned long startTime = micros();
  // Reverse x/y values to match joystick orientation
  data.joyX = 1023 - ss.analogRead(14);
  data.joyY = 1023 - ss.analogRead(15);

  uint32_t buttons = ss.digitalReadBulk(button_mask);
  unsigned long endTime = micros();

  data.buttonState = 0b00000000;
  if (! (buttons & (1UL << BUTTON_START))) { data.buttonState = data.buttonState | 0b10000000;}
  if (! (buttons & (1UL << BUTTON_SELECT))) { data.buttonState = data.buttonState | 0b01000000;}
  if (! (buttons & (1UL << BUTTON_Y))) { data.buttonState = data.buttonState | 0b00100000;}
  if (! (buttons & (1UL << BUTTON_X))) { data.buttonState = data.buttonState | 0b00010000;}
  if (! (buttons & (1UL << BUTTON_A))) { data.buttonState = data.buttonState | 0b00001000;}
  if (! (buttons & (1UL << BUTTON_B))) { data.buttonState = data.buttonState | 0b00000100;}

  char XHB = (char)highByte(data.joyX);
  char XLB = (char)lowByte(data.joyX);
  char YHB = (char)highByte(data.joyY);
  char YLB = (char)lowByte(data.joyY);

  data.checksum = data.buttonState ^ XHB ^ XLB ^ YHB ^ YLB;

#ifdef DEBUG_ENABLED
  Serial.print("x: "); Serial.print(data.joyX); Serial.print(", "); Serial.print("y: "); Serial.print(data.joyY);
  Serial.print("  ");
  Serial.print(data.buttonState, BIN);
  Serial.print("  ");
  Serial.print(data.checksum, BIN);
  Serial.print("  ");
  Serial.println(endTime - startTime);
#endif

  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &data, sizeof(data));
   
  if (result != ESP_OK) Serial.println("Error sending the data");

}
