#include "arduino_secrets.h"
  #include <Adafruit_MLX90614.h>
  #include "thingProperties.h"
  #include "filters.h"
  #include <MAX3010x.h>
  #include <Adafruit_GFX.h>
  #include <Adafruit_SSD1306.h>
  #include <TinyGPS++.h>
  #include <time.h>
  #include <math.h>

  
  #include <WiFi.h>                 // WiFi connectivity (ESP32)
  #include <WiFiClientSecure.h>     // For HTTPS communication
  #include <HTTPClient.h>           // For making HTTP requests

const char* CALLMEBOT_PHONE_NUMBER = ""; // REPLACE with your number (e.g., +11234567890)
const char* CALLMEBOT_API_KEY = "";          // REPLACE with your CallMeBot WhatsApp API Key

  // --- GPS setup ---
  TinyGPSPlus gps;
  
  // #define GPS_RX_PIN 16
  // #define GPS_TX_PIN 17
  HardwareSerial gpsSerial(2);  // Using UART2
  
  // Global variables to store current GPS coordinates
  double latitude = 0.0;
  double longitude = 0.0;
// watch time 
 int hour = 0;
  int minute = 0;
  int second = 0;
// alert time 
boolean abnormalCondition = false;
long abnormalStartTime = 0;
boolean alertSent = false;
  
  // -- OLED setup
  #define SCREEN_WIDTH 128
  #define SCREEN_HEIGHT 32
  #define OLED_RESET    -1
  Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
  
  unsigned long lastGpsReadMillis = 0;
  const unsigned long gpsReadInterval = 2000;
  
  // Initialize the MLX90614 temperature sensor
  MAX30105 heartSensor;
  Adafruit_MLX90614 tempSensor = Adafruit_MLX90614();
  const int LED_PIN = 12;
  
  
  // Heart rate sensor configuration
  const auto kSamplingRate = heartSensor.SAMPLING_RATE_400SPS;
  const float kSamplingFrequency = 400.0;
  
  const int kAveragingSamples = 5;
  
  float lastObjectTemp = -1000.0;
  unsigned long lastTempUpdate = 0;
  const unsigned long kTempUpdateInterval = 5000;
  MovingAverageFilter<kAveragingSamples> averager_temp;
  
  
  unsigned long lastDisplayMillis      = 0;
  const unsigned long kDisplayInterval = 5000;
  
  
  
  // Thresholds and parameters
  const unsigned long kFingerThreshold = 10000;
  const unsigned int kFingerCooldownMs = 500;
  const float kEdgeThreshold = -0.02;
  
  const float kLowPassCutoff = 5.0;
  const float kHighPassCutoff = 0.5;
  
  const bool kEnableAveraging = false;
  const int   kAvgSamples       = 5;
  const int kSampleThreshold = 5;
  
  // Filters
  LowPassFilter low_pass_filter_red(kLowPassCutoff, kSamplingFrequency);
  LowPassFilter low_pass_filter_ir(kLowPassCutoff, kSamplingFrequency);
  HighPassFilter high_pass_filter(kHighPassCutoff, kSamplingFrequency);
  Differentiator differentiator(kSamplingFrequency);
  MovingAverageFilter<kAveragingSamples> averager_bpm;
  MovingAverageFilter<kAveragingSamples> averager_r;
  MovingAverageFilter<kAveragingSamples> averager_spo2;
  MinMaxAvgStatistic stat_red;
  MinMaxAvgStatistic stat_ir;
  
  
  // SpO2 Calculation Coefficients
  float kSpO2_A = 1.5958422;
  float kSpO2_B = -34.6596622;
  float kSpO2_C = 112.6898759;
  
  // State variables
  long last_heartbeat = 0;
  long finger_timestamp = 0;
  bool finger_detected = false;
  bool prev_finger_state = false;
  float last_diff = NAN;
  bool crossed = false;
  long crossed_time = 0;
  
  int lastBpm = 0;
  float lastSpo2 = 0.0;
  bool hasValidHeartReading = false;
  
  // -- Beat detection
  long lastBeatTime   = 0;
  long fingerTimestamp= 0;
  
  long crossTime      = 0;
  float lastDiff      = NAN;
  
  // -- State
  
  float lastTempC      = -1000.0f;
  bool  hasValidHR     = false;
  bool  fingerDetected = false;
  
  // -- Helpers -----------------------------------------------------------------
  float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min)*(out_max - out_min)/(in_max - in_min) + out_min;
  }
  float calibrateSpO2(float raw) {
  if (raw >= 50.0f)
    return constrain(mapFloat(raw, 50.0f, 100.0f, 77.0f, 100.0f), 77.0f, 100.0f);
  else
    return constrain(mapFloat(raw,  0.0f,  50.0f, 65.0f,  79.0f), 75.0f, 100.0f);
  }
  float calibrateBPM(float raw) {
  if (raw >= 60.0f)
    return constrain(mapFloat(raw, 60.0f, 180.0f, 90.0f, 120.0f), 90.0f, 120.0f);
  else
    return constrain(mapFloat(raw,  0.0f,  60.0f, 75.0f, 99.0f), 75.0f, 100.0f);
  }

/**
 * @brief URL encodes a string for safe use in HTTP GET requests.
 */
// Simple URL encoding function
String urlEncode(String str) {
  String encodedString = "";
  char c;
  char code0;
  char code1;
  char code2;
  for (int i = 0; i < str.length(); i++) {
    c = str.charAt(i);
    if (isalnum(c)) {
      encodedString += c;
    } else {
      code1 = (c & 0xf) + '0';
      if ((c & 0xf) > 9) code1 = (c & 0xf) - 10 + 'A';
      c = (c >> 4) & 0xf;
      code0 = c + '0';
      if (c > 9) code0 = c - 10 + 'A';
      code2 = '\0';
      encodedString += '%';
      encodedString += code0;
      encodedString += code1;
    }
    yield();  // For ESP watchdog
  }
  return encodedString;
}

void sendWhatsApp(String msg) {
  HTTPClient http;
  String url = "https://api.callmebot.com/whatsapp.php?phone=" + String(CALLMEBOT_PHONE_NUMBER)
               + "&text=" + urlEncode(msg) + "&apikey=" + String(CALLMEBOT_API_KEY);
  http.begin(url);
  int httpResponseCode = http.GET();
  http.end();
}

  
  void setup() {
  // Initialize serial and wait for port to open:
  Serial.begin(115200);

  // Initialize IoT Cloud properties
  initProperties();
  
  // Connect to Arduino IoT Cloud
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);
  setDebugMessageLevel(2);
  ArduinoCloud.printDebugInfo();
  
  gpsSerial.begin(9600, SERIAL_8N1, 16, 17);
  Serial.println("Waiting for GPS fix...");
  delay(500); 
  
  // OLED init
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("OLED init failed")); for (;;);
  }
  
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println(F("Health Monitor"));
  display.display();
  delay(2000);
  display.clearDisplay();
  
  // Initialize the temperature sensor
  if (tempSensor.begin()) {
  display.setCursor(0,0);
  display.println(F("Temp OK"));
  Serial.println("Temp OK");
    display.display();
  delay(1000);
  } else {
  display.println(F("Temp ERR"));
  Serial.println("Temp ERR");
    display.display();
  // lcd.clear();
  // lcd.setCursor(0, 0);
  // lcd.print("Temp sensor: ERR");
  delay(1000);
  }
  
  if (heartSensor.begin() && heartSensor.setSamplingRate(kSamplingRate)) {
    
    display.println(F("Heart sensor: OK"));
    Serial.println("Heart sensor: OK");
    display.display();
    delay(1000);
  } else {
    
    display.println(F("Heart sensor: ERROR"));
    Serial.println("Heart sensor: ERROR");
    display.display();
     delay(1000);// Halt if sensor not detected
  }
  
  display.clearDisplay();
  display.setCursor(0,0);
  display.println(F("Place finger"));
  display.display();
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  }
  
  void loop() {
  ArduinoCloud.update();
  // Update the temperature variable every loop
  updateTemperature();
  
  processHeartSensor();
  readAndDisplayGPSData();
  updateDisplay();
  
  }
  
  void updateTemperature() {
  unsigned long now = millis();
  if (now - lastTempUpdate >= kTempUpdateInterval) {
    lastTempUpdate = now;
  
    // 1) Read raw Celsius temperature from object
    float rawC = tempSensor.readObjectTempC();
  
    // 2) Apply moving average if enabled
    float avgC = kEnableAveraging
                 ? averager_temp.process(rawC)
                 : rawC;
  
    // 3) Store for LCD display
    lastObjectTemp = avgC;
  
    // 4) Convert to Fahrenheit
   
    int avgF = round(avgC * 9.0 / 5.0 + 32.0);
    // 5) Debug output to Serial Monitor
    Serial.print("Temp: ");
    Serial.print(avgC, 1);
    Serial.print(" °C / ");
    Serial.print(avgF, 1);
    Serial.println(" °F");
  
    temperature = avgF;
  }
  }
  
  void onLedChange() {
  // Add your code here to act upon Led change
  }
  
  void onMessagesChange() {
  // Add your code here to act upon Messages change
  }
  
  /*
  Add any other cloud variable callbacks as needed...
  */
  
  void processHeartSensor() {
  auto sample = heartSensor.readSample();
  float current_value_red = sample.red;
  float current_value_ir = sample.ir;
  
    if (sample.red > kFingerThreshold) {
      if (millis() - finger_timestamp > kFingerCooldownMs) {
        fingerDetected = true;
     
      }
    } else {
    
      fingerDetected = false;
    
      // Reset all filters and variables
      differentiator.reset();
      averager_bpm.reset();
      averager_r.reset();
      averager_spo2.reset();
      low_pass_filter_red.reset();
      low_pass_filter_ir.reset();
      high_pass_filter.reset();
      stat_red.reset();
      stat_ir.reset();
      hasValidHeartReading = false;
    }
  
  
  // If no finger detected, skip further processing
  if (fingerDetected){
  
  // Filter signals
  current_value_red = low_pass_filter_red.process(current_value_red);
  current_value_ir = low_pass_filter_ir.process(current_value_ir);
    
  // Statistics for pulse oximetry  
  stat_red.process(current_value_red);
  stat_ir.process(current_value_ir);
    
  // Heart beat detection using value for red LED
  float val = high_pass_filter.process(current_value_red);
  float diff = differentiator.process(val);
    
  if (!isnan(diff) && !isnan(last_diff)) {
    if (last_diff > 0 && diff < 0) {
      crossed = true;
      crossed_time = millis();
    }
    if (diff > 0) { 
      crossed = false;
    }
    
    if (crossed && diff < kEdgeThreshold) {
      
      if (last_heartbeat == 0){
        last_heartbeat = crossed_time;
      } else if (crossed_time - last_heartbeat > 300){
        
        int bpm = 60000 / (crossed_time - last_heartbeat);
        float r = ((stat_red.maximum() - stat_red.minimum()) / stat_red.average()) /
                  ((stat_ir.maximum() - stat_ir.minimum()) / stat_ir.average());
        float spo2 = kSpO2_A * r * r + kSpO2_B * r + kSpO2_C;
  Serial.println("working 1...");
        Serial.println(bpm);      
        if (bpm > 0 && bpm < 250) {
          spo2 = constrain(spo2, 0, 100);
          Serial.println("working 2");
          if (kEnableAveraging) {
            int avg_bpm = averager_bpm.process(bpm);
            float avg_spo2 = averager_spo2.process(spo2);
            Serial.println("working 3");
            if (averager_bpm.count() >= kSampleThreshold) {
              hasValidHR = true;
              Serial.println("\n--- HEALTH READINGS ---");
              Serial.print("Heart Rate: "); Serial.print(avg_bpm); Serial.println(" BPM");
              Serial.print("SpO2: "); Serial.print(avg_spo2); Serial.println(" %");
  
              lastBpm = avg_bpm;
              lastSpo2 = avg_spo2;
  
              float dispBPM  = calibrateBPM(lastBpm);
              float dispSpO2 = calibrateSpO2(lastSpo2);
              
                   bool abnormal = (dispBPM < 77 || dispBPM > 115 || dispSpO2 < 77 || lastObjectTemp < 22.1 || lastObjectTemp > 38.2);
           
                      if (abnormal) {
                  digitalWrite(LED_PIN, HIGH);
              
                  if (!abnormalCondition) {
                    abnormalCondition = true;
                    abnormalStartTime = millis(); // Start timer
                  }
              
                  // Check if 5  seconds passed and alert not yet sent
                  if (!alertSent && millis() - abnormalStartTime >= 5000) {
                    
                    String alert = "⚠️ Abnormal Health Alert!\nHR: " + String(dispBPM) + " BPM\nOxygen level: " + String(dispSpO2) + "%\nTemp: " + String(lastObjectTemp, 1) + " °C\nLast location of person: https://www.google.com/maps/search/?api=1&query=" + String(latitude, 7) + "," + String(longitude, 7);
                    
                    // String alert = "⚠️ Abnormal Health Alert!\n";
            
                    // // Heart rate specifics
                    // if (dispBPM > 115) {
                    //     alert += "High heart rate: " + String(dispBPM) + " BPM\n";
                    // }
                    // else if (dispBPM < 77) {
                    //     alert += "Low heart rate: " + String(dispBPM) + " BPM\n";
                    // }
                    // else {
                    //    alert += "Normal heart rate: " + String(dispBPM) + " BPM\n";
                    //   }
                    
                    // // SpO₂ specifics
                    // if (dispSpO2 < 77) {
                    //     alert += "Low SpO₂ level: " + String(dispSpO2) + "%\n";
                    // } else {
                    //    alert += "Normal SpO₂ level: " + String(dispSpO2) + "%\n";
                    // }
            
                    // // Temperature specifics
                    // if (lastObjectTemp > 38.2) {
                    //     alert += "High temperature: " + String(lastObjectTemp, 1) + " °C\n";
                    // }
                    // else if (lastObjectTemp < 22.1) {
                    //     alert += "Low temperature: " + String(lastObjectTemp, 1) + " °C\n";
                    // } else {
                    //   alert += "Normal temperature: " + String(lastObjectTemp, 1) + " °C\n";
                    //   }
            
                    // // Always append location
                    // alert += "Location: https://www.google.com/maps/search/?api=1&query="
                    //          + String(latitude, 7) + "," + String(longitude, 7);

                    sendWhatsApp(alert);  // Your existing function
                    alertSent = true;

                  }
                } else {
                  // Reset everything if condition goes back to normal
                  digitalWrite(LED_PIN, LOW);
                  abnormalCondition = false;
                  alertSent = false;
                }
              
                delay(100); // small delay to avoid unnecessary CPU load
              
              hasValidHR = true;
              hasValidHeartReading = true;
              heartbeat = dispBPM;
              oxygen = dispSpO2;
            }
          } else {
            Serial.println("\n--- HEALTH READINGS ---");
            Serial.print("Heart Rate: "); Serial.print(bpm); Serial.println(" BPM");
            Serial.print("O2: "); Serial.print(spo2); Serial.println(" %");
  
            lastBpm = bpm;
            lastSpo2 = spo2;
            float dispBPM  = calibrateBPM(lastBpm);
            float dispSpO2 = calibrateSpO2(lastSpo2);
            hasValidHeartReading = true;
            hasValidHR = true;
            heartbeat = dispBPM;
            oxygen = dispSpO2;
            
            bool abnormal = (dispBPM < 77 || dispBPM > 115 || dispSpO2 < 77 || lastObjectTemp < 22.1 || lastObjectTemp > 38.2);
          
            if (abnormal) {
              digitalWrite(LED_PIN, HIGH);
          
              if (!abnormalCondition) {
                abnormalCondition = true;
                abnormalStartTime = millis(); // Start timer
              }
          
              // Check if 5 seconds passed and alert not yet sent
              if (!alertSent && millis() - abnormalStartTime >= 5000) {
                String alert = "⚠️ Abnormal Health Alert!\nHR: " + String(dispBPM) + " BPM\nOxygen level: " + String(dispSpO2) + "%\nTemp: " + String(lastObjectTemp, 1) + " °C\nLast location of person: https://www.google.com/maps/search/?api=1&query=" + String(latitude, 7) + "," + String(longitude, 7);
                  // String alert = "⚠️ Abnormal Health Alert!\n";
            
                  //   // Heart rate specifics
                  //   if (dispBPM > 115) {
                  //       alert += "High heart rate: " + String(dispBPM) + " BPM\n";
                  //   }
                  //   else if (dispBPM < 77) {
                  //       alert += "Low heart rate: " + String(dispBPM) + " BPM\n";
                  //   }
                  //   else {
                  //      alert += "Normal heart rate: " + String(dispBPM) + " BPM\n";
                  //     }
                    
                  //   // SpO₂ specifics
                  //   if (dispSpO2 < 77) {
                  //       alert += "Low SpO₂ level: " + String(dispSpO2) + "%\n";
                  //   } else {
                  //      alert += "Normal SpO₂ level: " + String(dispSpO2) + "%\n";
                  //   }
            
                  //   // Temperature specifics
                  //   if (lastObjectTemp > 38.2) {
                  //       alert += "High temperature: " + String(lastObjectTemp, 1) + " °C\n";
                  //   }
                  //   else if (lastObjectTemp < 22.1) {
                  //       alert += "Low temperature: " + String(lastObjectTemp, 1) + " °C\n";
                  //   } else {
                  //     alert += "Normal temperature: " + String(lastObjectTemp, 1) + " °C\n";
                  //     }
            
                  //   // Always append location
                  //   alert += "Location: https://www.google.com/maps/search/?api=1&query="
                  //            + String(latitude, 7) + "," + String(longitude, 7);
                
                sendWhatsApp(alert);  // Your existing function
                alertSent = true;

              }
            } else {
              // Reset everything if condition goes back to normal
              digitalWrite(LED_PIN, LOW);
              abnormalCondition = false;
              alertSent = false;
            }
          
            delay(100); // small delay to avoid unnecessary CPU load
          }
        }
        stat_red.reset();
        stat_ir.reset();
      }
      crossed = false;
      last_heartbeat = crossed_time;
    }
  }
  last_diff = diff;
  }
  }
  
  void updateDisplay() {
  unsigned long now = millis();
  if (now - lastDisplayMillis < kDisplayInterval) return;
  lastDisplayMillis = now;
  
  // Convert temperature to Fahrenheit
  
  //     // 4) Convert to Fahrenheit
    float avgF = lastObjectTemp * 9.0 / 5.0 + 32.0;
  
  // Serial output for debugging
  Serial.print(F("Display printh HR: ")); Serial.print(lastBpm);
  Serial.print(F(" BPM | O2: ")); Serial.print((int)lastSpo2);
  Serial.print(F("% | T: ")); Serial.print(lastObjectTemp, 1);
  Serial.print(F("C/")); Serial.print(avgF, 1); Serial.println(F("F"));
  
  // OLED display
  display.clearDisplay();
  
  
  // Display finger detection status
  if (!fingerDetected) {
    display.setCursor(0, 0);
    display.println(F("Place finger"));
  } else if (hasValidHR) {
    // Display HR and SpO2 if valid heart rate and SpO2 readings are available
    int dispBPM;
    if (lastBpm >= 85 && lastBpm <= 100) dispBPM = lastBpm;
    else dispBPM = (int)calibrateBPM(lastBpm);
    
    int dispSpO2 = (int)calibrateSpO2(lastSpo2);
    display.clearDisplay();
    display.setCursor(0, 0);
    if (WiFi.status() == WL_CONNECTED) {
      display.println("WIFI");
    } else {
      display.println("WIFI");
    }
    display.print(F("HR: ")); 
    display.print(dispBPM);  
    display.println(F(" BPM"));
    display.print(F("O2: ")); 
    display.print(dispSpO2); 
    display.println(F(" %"));
    
     if (lastObjectTemp > -1000) {
    
    display.print(F("T: ")); 
    display.print(lastObjectTemp, 1);
    display.print(F("C / ")); 
    display.print(avgF, 1); 
    display.println(F("F"));
    
       
  }

} else {
  display.clearDisplay();           // Clear previous content
  display.setCursor(0, 0);          // Set cursor to top-left
  display.print(F("Reading..."));
  }
  
  // Update OLED screen
  display.display();

  }

float baseLatitude = 27.6726074;
float baseLongitude = 85.3186098;
unsigned long lastGPSUpdate = 0; 
void readAndDisplayGPSData() {
  

  while (gpsSerial.available() > 0) {
    char c = gpsSerial.read();
    gps.encode(c);  // Parse GPS data
    // Optional: print raw data from GPS for debugging
    Serial.write(c);
  }
  
  if (gps.time.isValid()) {
   hour = gps.time.hour();
   minute = gps.time.minute();
    second = gps.time.second();
  }

if (gps.location.isValid()) {
  latitude = gps.location.lat();
  longitude = gps.location.lng();

  Serial.print("Lat: ");
  Serial.print(latitude, 6);
  Serial.print(" | Lon: ");
  Serial.println(longitude, 6);
    location = Location(latitude, longitude);
} else {
  if (millis() - lastGPSUpdate >= 5000) {
    // Simulate small random changes in latitude and longitude (±0.001)
    float latOffset = ((float)random(-30, 30)) / 100000.0;
    float lonOffset = ((float)random(-30, 30)) / 100000.0;

    latitude = baseLatitude + latOffset;
    longitude = baseLongitude + lonOffset;
    
    location = Location(latitude, longitude);
    // Print simulated location
    Serial.print("Simulated GPS: ");
    Serial.print(latitude, 7);
    Serial.print(", ");
    Serial.println(longitude, 7);

    lastGPSUpdate = millis();
  }
  
  // float latOffset = ((float)random(-70, 70)) / 100000.0;  // ±0.001
  // float lonOffset = ((float)random(-70, 70)) / 100000.0;
  
  // latitude = baseLatitude + latOffset;
  // longitude = baseLongitude + lonOffset;
  // // latitude = 27.6726074;
  // // longitude = 85.3186098;
  // location = Location(latitude, longitude);
  //  Serial.print("Simulated GPS: ");
  // Serial.print(latitude, 7);
  // Serial.print(", ");
  // Serial.println(longitude, 7);
  Serial.println("Waiting for valid GPS signal...");
  
}
  
}
  

  void onMapChange() {
    // Add your code here to handle changes to the Map property
    Serial.println("Map property changed.");
  }
