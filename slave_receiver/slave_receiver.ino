 ////////////////////////////////////// forward only (wokrs well with filter but introduces drift and slow respinse ) 
 #include <esp_now.h>
 #include <WiFi.h>
 #include <Wire.h>
 #include <Adafruit_BMP280.h>

 Adafruit_BMP280 bmp;

  --- CONFIGURATION ---
 const int BURST_SIZE = 300;    // Number of readings to average per "frame"
 volatile float masterP = 0;
 bool firstSyncReceived = false;
 float heightOffset = 0;      

 typedef struct struct_message {
     float masterPressure;
 } struct_message;
 struct_message incomingData;

 void OnDataRecv(const esp_now_recv_info *recv_info, const uint8_t *incoming, int len) {
   memcpy(&incomingData, incoming, sizeof(incomingData));
   masterP = incomingData.masterPressure;
   firstSyncReceived = true;
 }

 void setup() {
   Serial.begin(115200);
   WiFi.mode(WIFI_STA);
   esp_now_init();
   esp_now_register_recv_cb(OnDataRecv);

   if (!bmp.begin(0x76) && !bmp.begin(0x77)) { while(1); }
  
   // High-speed settings for burst sampling
   bmp.setSampling(Adafruit_BMP280::MODE_NORMAL, 
                   Adafruit_BMP280::SAMPLING_X2, 
                   Adafruit_BMP280::SAMPLING_X4, 
                   Adafruit_BMP280::FILTER_X4,  // change this param for accuracy
                   Adafruit_BMP280::STANDBY_MS_1);

   Serial.println("System Ready. Type 't' to Tare.");
 }

 void loop() {
   // 1. Handle Serial Tare
   if (Serial.available() > 0) {
     char cmd = Serial.read();
     if (cmd == 't' || cmd == 'z') {
        Calculate current absolute height to set as zero
       float sum = 0;
       for(int i=0; i<BURST_SIZE; i++) {
         sum += 44330.0 * (1.0 - pow((bmp.readPressure()/100.0) / masterP, 0.1903));
       }
       heightOffset = sum / BURST_SIZE;
       Serial.println(">>>> TARE PERFORMED <<<<");
     }
   }

   if (!firstSyncReceived) {
     Serial.println("Waiting for Master Node Sync...");
     delay(500);
     return;
   }

   // 2. BURST SAMPLING (No past readings used)
   float burstSum = 0;
   for (int i = 0; i < BURST_SIZE; i++) {
     float localP = bmp.readPressure() / 100.0F;
      Immediate calculation
     burstSum += 44330.0 * (1.0 - pow(localP / masterP, 0.1903));
      No delay here; read as fast as the I2C bus allows
   }

   // 3. Final Prediction for this moment
   float currentHeight = (burstSum / BURST_SIZE) - heightOffset;

   // 4. Precision Output (cm only)
   Serial.print("Current_Relative_Height:"); 
   Serial.print(currentHeight, 1); 
   Serial.println(" m");

    Small delay to make the Serial Monitor readable
   delay(100); 
 }