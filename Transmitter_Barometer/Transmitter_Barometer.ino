#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>

Adafruit_BMP280 bmp;

// REPLACE THIS with your SLAVE'S MAC address
uint8_t slaveAddress[] = {0x10, 0x51, 0xDB, 0x85, 0xC2, 0xD4}; 

typedef struct struct_message {
    float masterPressure;
} struct_message;

struct_message myData;
esp_now_peer_info_t peerInfo;

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  
  // Set WiFi Channel (Must be same as Slave)
  // esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE); 

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register slave
  memcpy(peerInfo.peer_addr, slaveAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  if (!bmp.begin(0x76)) { Serial.println("BMP fail"); while(1); }
  
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL, Adafruit_BMP280::SAMPLING_X16, 
                  Adafruit_BMP280::SAMPLING_X4, Adafruit_BMP280::FILTER_X4, // set this param for better accuracy
                  Adafruit_BMP280::STANDBY_MS_1);
}

void loop() {
  myData.masterPressure = bmp.readPressure() / 100.0F;
  
  esp_err_t result = esp_now_send(slaveAddress, (uint8_t *) &myData, sizeof(myData));
  
  if (result == ESP_OK) {
    Serial.print("Sent Master Pressure: ");
    Serial.println(myData.masterPressure);
  } else {
    Serial.println("Error sending the data");
  }
  delay(100); 
}