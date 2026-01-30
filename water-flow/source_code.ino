#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <vector> 

const char* ssid = "";
char* password = "";
const char* mqtt_server = "";

WiFiClient espClient;
PubSubClient mqttClient(espClient);

const float millilitersPerPulse = 1000.0 / 60.0;
const int flowSensorPin = 25;

volatile unsigned long pulseAccumulator = 0;
unsigned long lastSampleTime = 0;
unsigned long lastPulseCount = 0;

void IRAM_ATTR increase() {
  pulseAccumulator++;
}

void wifiStatus() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Connecting to WiFi");
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("WiFi connected");
    }
  }
}

void mqttStatus() {
  if (WiFi.status() == WL_CONNECTED && !mqttClient.connected()) {
    mqttClient.connect("test-sensor", "mqtt_indicator_1", "mqtt");
    Serial.println("Connecting to MQTT...");
    if (mqttClient.connected()) {
      Serial.println("MQTT connected");
    }
  }
}

void readAndPublish() {
  // Read Pulse and Time accumulate if Wi-Fi/MQTT is not ready.
  if (millis() - lastSampleTime < 1000) return;

  noInterrupts();
  unsigned long now = millis();
  unsigned long pulses = pulseAccumulator;
  interrupts();

  // Difference since last sample
  unsigned long pulseDelta = pulses - lastPulseCount;
  unsigned long timeDelta = now - lastSampleTime;

  lastPulseCount += pulseDelta;
  lastSampleTime += timeDelta;

  // MQTT Client will automatically disconnect if ESP is not connected to WiFi
  if (mqttClient.connected()) {

    StaticJsonDocument<200> doc;
    doc["Pulses"] = lastPulseCount;
    doc["Millis"] = lastSampleTime;
  
    char buffer[200];
    serializeJson(doc, buffer, sizeof(buffer));

    if (mqttClient.publish("sensor/Pulses-Time", buffer)) {
      lastPulseCount = pulses;
      lastSampleTime = now;
    }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(flowSensorPin, INPUT_PULLUP);
  lastSampleTime = millis();
  attachInterrupt(digitalPinToInterrupt(flowSensorPin), increase, RISING);

  WiFi.begin(ssid, password);
  mqttClient.setServer(mqtt_server, 1883);
}

void loop() {
  mqttStatus();

  readAndPublish();

  wifiStatus();  
  mqttClient.loop();

}