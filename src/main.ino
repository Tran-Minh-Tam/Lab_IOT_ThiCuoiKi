#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <PubSubClient.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <DHT.h>

// ================= PIN DEFINITIONS =================
#define DHT_PIN       25
#define DHT_TYPE      DHT22
#define SOIL_PIN      34
#define RELAY_PIN     19
#define LED_GREEN     4
#define LED_YELLOW    16
#define LED_RED       17

// OLED SH1106 via I2C
#define OLED_SDA      18
#define OLED_SCL      5

// ================= OBJECTS =================
DHT dht(DHT_PIN, DHT_TYPE);
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, OLED_SCL, OLED_SDA);

// ================= WIFI & MQTT =================
const char* ssid     = "Wokwi-GUEST";
const char* password = "";
const char* mqtt_server = "192.168.1.11";
const int   mqtt_port   = 1883;

// Topic for Node-RED alerts
const char* TOPIC_PUB_ALERT    = "tuoicay/canhbao";

// Topics
const char* TOPIC_PUB_DATA     = "tuoicay/data";
const char* TOPIC_PUB_PUMP     = "tuoicay/pump";
const char* TOPIC_PUB_STATUS   = "tuoicay/status";
const char* TOPIC_PUB_AI       = "tuoicay/ai";
const char* TOPIC_PUB_WATER    = "tuoicay/water_usage";
const char* TOPIC_SUB_MANUAL   = "tuoicay/manual";

WiFiClient   espClient;
PubSubClient client(espClient);

// ================= SENSOR DATA =================
float soilMoisture  = 60.0;
float airTemp       = 28.0;
float airHumidity   = 65.0;
float dryIndex      = 0.0;

// History for AI trend detection (last 3 readings)
float soilHistory[3] = {60, 60, 60};
int   historyIdx     = 0;

// ================= PUMP / WATERING =================
bool  pumpOn         = false;
bool  manualOverride = false;
unsigned long pumpStartTime = 0;
unsigned long pumpDuration  = 0;   // ms
float dailyWaterUsed = 14.5;        // liters simulated
float dailyWaterSaved= 3.2;
int   pumpCycles     = 5;

// ================= TIMING =================
unsigned long lastSensorRead  = 0;
unsigned long lastOledUpdate  = 0;
unsigned long lastMqttSend    = 0;
const unsigned long SENSOR_INTERVAL = 5000;

// ================= OLED ICONS (bitmaps 16x16) =================
// Thermometer icon
// ================= OLED ICONS (bitmaps 16x16) =================
// Icon Cây (Tree)
static const unsigned char PROGMEM icon_tree[] = {
  0x80, 0x01, 0xc0, 0x03, 0xe0, 0x07, 0xf0, 0x0f, 
  0xf8, 0x1f, 0xfc, 0x3f, 0xfe, 0x7f, 0xfe, 0x7f, 
  0xfc, 0x3f, 0xf8, 0x1f, 0x80, 0x01, 0x80, 0x01, 
  0x80, 0x01, 0x80, 0x01, 0xe0, 0x07, 0x00, 0x00
};

// Icon Mây (Cloud)
static const unsigned char PROGMEM icon_cloud[] = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x03, 
  0xc0, 0x07, 0xe0, 0x0f, 0xf8, 0x1f, 0xfc, 0x3f, 
  0xfe, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xfe, 0x7f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
static const unsigned char PROGMEM icon_temp[] = {
  0x06,0x00, 0x09,0x00, 0x09,0x00, 0x09,0x00,
  0x09,0x00, 0x19,0x00, 0x29,0x00, 0x29,0x00,
  0x49,0x00, 0x4F,0x00, 0x8F,0x80, 0x8F,0x80,
  0x4F,0x00, 0x49,0x00, 0x26,0x00, 0x1C,0x00
};
// Droplet icon
static const unsigned char PROGMEM icon_drop[] = {
  0x04,0x00, 0x04,0x00, 0x0E,0x00, 0x0E,0x00,
  0x1F,0x00, 0x1F,0x00, 0x3F,0x80, 0x3F,0x80,
  0x3F,0x80, 0x3F,0x80, 0x3F,0x80, 0x1F,0x00,
  0x1F,0x00, 0x0E,0x00, 0x04,0x00, 0x00,0x00
};
// Plant/soil icon
static const unsigned char PROGMEM icon_soil[] = {
  0x04,0x00, 0x0E,0x00, 0x0E,0x00, 0x1F,0x00,
  0x1F,0x00, 0x0E,0x00, 0x04,0x00, 0x04,0x00,
  0xFF,0xF0, 0xFF,0xF0, 0xEF,0xF0, 0xC7,0xF0,
  0x83,0xF0, 0x01,0xF0, 0x00,0xF0, 0x00,0x00
};

// ================= HELPER FUNCTIONS =================

// Compute DryIndex
float computeDryIndex(float S, float T, float H) {
  float Ds = (100.0 - S) / 100.0;
  float Dt = constrain((T - 15.0) / 25.0, 0.0, 1.0);
  float Dh = (100.0 - H) / 100.0;
  return 0.6 * Ds + 0.25 * Dt + 0.15 * Dh;
}

// AI: detect rapid drying trend
bool aiDetectRapidDry() {
  // Calculate moisture drop rate over last 3 readings
  float drop1 = soilHistory[0] - soilHistory[1];
  float drop2 = soilHistory[1] - soilHistory[2];
  float avgDrop = (drop1 + drop2) / 2.0;
  // If dropping more than 3% per 5 seconds → trigger early
  return (avgDrop > 3.0);
}

// AI: predict when soil will reach 30% threshold
float aiPredictTimeToThreshold() {
  float drop1 = soilHistory[0] - soilHistory[1];
  float drop2 = soilHistory[1] - soilHistory[2];
  float avgDrop = (drop1 + drop2) / 2.0;
  if (avgDrop <= 0) return 9999;
  return (soilMoisture - 30.0) / avgDrop * 5.0; // seconds
}

// Compute pump duration based on temperature
unsigned long computePumpDuration(float temp) {
  // Base: 10s, +1s per °C above 20, max 30s
  unsigned long dur = 10000 + (unsigned long)((temp - 20.0) * 1000.0);
  return constrain(dur, 5000UL, 30000UL);
}

// LED status
void setStatusLEDs(int state) {
  // state: 0=green(OK), 1=yellow(drying), 2=red(pumping)
  digitalWrite(LED_GREEN,  state == 0 ? HIGH : LOW);
  digitalWrite(LED_YELLOW, state == 1 ? HIGH : LOW);
  digitalWrite(LED_RED,    state == 2 ? HIGH : LOW);
}

// ================= NOTIFICATION FUNCTION =================
void sendAlertNotification(String reason, float value) {
  String alertMsg = "CANH BAO TU ESP32!\n";
  alertMsg += "Nguyen nhan: " + reason + "\n";
  alertMsg += "Gia tri: " + String(value, 1) + "\n";
  alertMsg += "Dat: " + String(soilMoisture, 1) + "%\n";
  alertMsg += "Nhiet do: " + String(airTemp, 1) + "C\n";
  alertMsg += "Do am KK: " + String(airHumidity, 1) + "%\n";
  alertMsg += "DryIndex: " + String(dryIndex, 3);
  
  client.publish(TOPIC_PUB_ALERT, alertMsg.c_str());
  Serial.println("[ALERT] Da gui len Node-RED: " + reason);
}

// Pump control
void startPump() {
  if (pumpOn) return;
  pumpOn        = true;
  pumpStartTime = millis();
  pumpDuration  = computePumpDuration(airTemp);
  digitalWrite(RELAY_PIN, HIGH);
  setStatusLEDs(2);
  pumpCycles++;
  Serial.println("[PUMP] BẮT ĐẦU TƯỚI - Thời gian: " + String(pumpDuration/1000) + "s");

  // Publish pump ON
  String msg = "{\"pump\":true,\"duration\":" + String(pumpDuration/1000) + "}";
  client.publish(TOPIC_PUB_PUMP, msg.c_str());
  client.publish(TOPIC_PUB_STATUS, "{\"status\":\"TUOI\",\"led\":\"red\",\"msg\":\"Dat dang kho - Dang tuoi nuoc\"}");
}

void stopPump() {
  if (!pumpOn) return;
  unsigned long elapsed = millis() - pumpStartTime;
  // Simulate water used: 0.1L per second
  float litersUsed = (float)elapsed / 1000.0 * 0.1;
  dailyWaterUsed += litersUsed;
  pumpOn = false;
  digitalWrite(RELAY_PIN, LOW);
  Serial.println("[PUMP] DỪNG TƯỚI - Nước đã dùng: " + String(litersUsed, 2) + "L");

  String msg = "{\"pump\":false,\"liters\":" + String(litersUsed,2) + ",\"daily\":" + String(dailyWaterUsed,2) + "}";
  client.publish(TOPIC_PUB_PUMP, msg.c_str());
  client.publish(TOPIC_PUB_WATER, ("{\"daily\":" + String(dailyWaterUsed,2) + ",\"cycles\":" + String(pumpCycles) + "}").c_str());
}

// ================= OLED DISPLAY =================
// ================= OLED DISPLAY =================
// ================= OLED DISPLAY =================
void updateOLED() {
  u8g2.clearBuffer();

  // --- Hàng 1: Icon Cây & Thông số Đất/Nước ---
  // Vẽ icon Cây (kích thước 16x16) tại tọa độ x=2, y=6
  u8g2.drawXBMP(2, 6, 16, 16, icon_tree); 

  u8g2.setFont(u8g2_font_6x12_tf);
  char buf[30];
  snprintf(buf, sizeof(buf), "Dat: %.1f%%", soilMoisture);
  u8g2.drawStr(24, 12, buf);

  snprintf(buf, sizeof(buf), "Nuoc dung: %.1f L", dailyWaterUsed);
  u8g2.drawStr(24, 24, buf);

  // --- Hàng 2: Icon Mây & Thông số Không khí ---
  // Vẽ icon Mây (kích thước 16x16) tại tọa độ x=2, y=32
  u8g2.drawXBMP(2, 32, 16, 16, icon_cloud);

  snprintf(buf, sizeof(buf), "Nhiet do: %.1f C", airTemp);
  u8g2.drawStr(24, 38, buf);

  snprintf(buf, sizeof(buf), "Do am KK: %.1f%%", airHumidity);
  u8g2.drawStr(24, 50, buf);

  // --- Dưới cùng: Trạng thái Bơm ---
  // Vẽ đường kẻ ngang phân cách
  u8g2.drawHLine(0, 54, 128);
  
  u8g2.setFont(u8g2_font_6x10_tf);
  if (pumpOn) {
    u8g2.drawStr(12, 64, ">> BOM: DANG BAT <<");
  } else {
    u8g2.drawStr(12, 64, "-- BOM: DANG TAT --");
  }

  u8g2.sendBuffer();
}

// ================= MQTT =================
void callback(char* topic, byte* payload, unsigned int length) {
  String msg = "";
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];

  if (String(topic) == TOPIC_SUB_MANUAL) {
    if (msg == "ON" || msg == "1") {
      manualOverride = true;
      startPump();
    } else {
      manualOverride = false;
      stopPump();
    }
  }
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("[MQTT] Đang kết nối...");
    String cid = "ESP32-Irrigation-" + String(random(0xffff), HEX);
    if (client.connect(cid.c_str())) {
      Serial.println(" OK!");
      client.subscribe(TOPIC_SUB_MANUAL);
    } else {
      Serial.println(" Lỗi, thử lại sau 3s");
      delay(3000);
    }
  }
}

void setup_wifi() {
  Serial.print("[WiFi] Kết nối...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500); Serial.print(".");
  }
  Serial.println(" IP: " + WiFi.localIP().toString());
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);

  pinMode(RELAY_PIN,  OUTPUT); digitalWrite(RELAY_PIN, LOW);
  pinMode(LED_GREEN,  OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  pinMode(LED_RED,    OUTPUT);
  setStatusLEDs(0);

  Wire.begin(OLED_SDA, OLED_SCL);
  u8g2.begin();

  dht.begin();
  randomSeed(analogRead(0));

  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  // Initial display
  updateOLED();
  Serial.println("[SYSTEM] Khởi động hoàn tất!");
}

// ================= LOOP =================
void loop() {
  if (!client.connected()) reconnect();
  client.loop();

  unsigned long now = millis();

  // ---- Pump auto-stop ----
  if (pumpOn && !manualOverride) {
    if (now - pumpStartTime >= pumpDuration) {
      stopPump();
    }
  }

  // ---- Sensor reading every 5s ----
  if (now - lastSensorRead >= SENSOR_INTERVAL) {
    lastSensorRead = now;

    // === READ SENSORS (randomized for Wokwi simulation) ===
    // DHT22 - Randomize for simulation as requested
    float tRead = dht.readTemperature();
    float hRead = dht.readHumidity();
    
    // Always add a bit of random noise to make it "live"
    if (!isnan(tRead)) airTemp = tRead + (random(-5, 5) / 10.0);
    else               airTemp = 24.0 + (random(-20, 20) / 10.0);
    
    if (!isnan(hRead)) airHumidity = hRead + (random(-10, 10) / 10.0);
    else               airHumidity = 55.0 + (random(-30, 30) / 10.0);

    // Clamp values to realistic ranges
    airTemp     = constrain(airTemp,     15.0, 45.0);
    airHumidity = constrain(airHumidity, 20.0, 95.0);

    // Soil moisture: Read from potentiometer (SOIL_PIN 34)
    int rawSoil = analogRead(SOIL_PIN);
    soilMoisture = map(rawSoil, 0, 4095, 0, 100);
    soilMoisture = constrain(soilMoisture, 0.0, 100.0);
    soilMoisture = constrain(soilMoisture, 0.0, 100.0);

    // === UPDATE HISTORY ===
    soilHistory[2] = soilHistory[1];
    soilHistory[1] = soilHistory[0];
    soilHistory[0] = soilMoisture;

    // === COMPUTE DRY INDEX ===
    dryIndex = computeDryIndex(soilMoisture, airTemp, airHumidity);

    // === AI ANALYSIS ===
    bool aiTrigger = aiDetectRapidDry();
    float timeToThreshold = aiPredictTimeToThreshold();

    // === AUTO-PUMP DECISION ===
    bool shouldPump = false;
    String pumpReason = "";

    if (soilMoisture < 30.0) {
      shouldPump = true;
      pumpReason = "Do am dat duoi 30%";
    } else if (dryIndex > 0.65) {
      shouldPump = true;
      pumpReason = "DryIndex cao (>" + String(dryIndex, 2) + ")";
    } else if (aiTrigger && soilMoisture < 45.0) {
      shouldPump = true;
      pumpReason = "AI: Do am giam nhanh - tuoi som";
    }

    // === TRIGGER ALERT NOTIFICATION ===
    if (soilMoisture < 30.0) {
      sendAlertNotification("Do am dat thuong xuyen o muc thap", soilMoisture);
    } else if (dryIndex > 0.7) {
      sendAlertNotification("Hanh kho: Chi so DryIndex cao nguy hiem", dryIndex);
    }

    if (shouldPump && !pumpOn && !manualOverride) {
      startPump();
    } else if (pumpOn && soilMoisture >= 60.0 && !manualOverride) {
      stopPump();
    }

    // === LED STATUS ===
    if (!pumpOn) {
      if (soilMoisture >= 45.0)      setStatusLEDs(0); // green
      else if (soilMoisture >= 30.0) setStatusLEDs(1); // yellow
      else                           setStatusLEDs(1); // yellow (waiting to pump)
    }

    // === BUILD MQTT PAYLOAD ===
    // Main data
    String dataPayload = "{";
    dataPayload += "\"soil\":" + String(soilMoisture, 1) + ",";
    dataPayload += "\"temp\":" + String(airTemp, 1) + ",";
    dataPayload += "\"humidity\":" + String(airHumidity, 1) + ",";
    dataPayload += "\"dryIndex\":" + String(dryIndex, 3) + ",";
    dataPayload += "\"pump\":" + String(pumpOn ? "true" : "false");
    dataPayload += "}";
    client.publish(TOPIC_PUB_DATA, dataPayload.c_str());

    // Status
    String statusLed = "green";
    String statusMsg  = "Dat am uong - Binh thuong";
    if (pumpOn) {
      statusLed = "red";
      statusMsg = "Dang tuoi nuoc";
    } else if (soilMoisture < 45.0) {
      statusLed = "yellow";
      statusMsg = "Dat sap kho - Can theo doi";
    }
    String statusPayload = "{\"status\":\"" + (pumpOn ? String("TUOI") : (soilMoisture >= 45 ? String("OK") : String("WARNING"))) + "\",\"led\":\"" + statusLed + "\",\"msg\":\"" + statusMsg + "\"}";
    client.publish(TOPIC_PUB_STATUS, statusPayload.c_str());

    // AI data
    String aiPayload = "{";
    aiPayload += "\"rapidDry\":" + String(aiTrigger ? "true" : "false") + ",";
    aiPayload += "\"timeToThreshold\":" + String(timeToThreshold, 1) + ",";
    aiPayload += "\"trend\":" + String((soilHistory[0] - soilHistory[2]) / 2.0, 2) + ",";
    aiPayload += "\"pumpReason\":\"" + (shouldPump ? pumpReason : String("none")) + "\"";
    aiPayload += "}";
    client.publish(TOPIC_PUB_AI, aiPayload.c_str());

    // Debug
    Serial.println("=== SENSOR DATA ===");
    Serial.println("Đất: " + String(soilMoisture, 1) + "% | Nhiệt: " + String(airTemp, 1) + "°C | KK: " + String(airHumidity, 1) + "%");
    Serial.println("DryIndex: " + String(dryIndex, 3) + " | Pump: " + String(pumpOn ? "ON" : "OFF"));
    Serial.println("AI Rapid Dry: " + String(aiTrigger ? "YES" : "NO") + " | ETA: " + String(timeToThreshold, 1) + "s");

    // OLED update
    updateOLED();
  }
}