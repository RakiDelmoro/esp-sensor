#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <vector> 

const char* ssid = "";
char* password = "";
const char* mqtt_server = "";

WiFiClient espClient;
PubSubClient mqttClient(espClient);

struct FlowWithTimeAccumulator {
  unsigned long totalTimeMs = 0;
  unsigned long totalPulses = 0;
};

FlowWithTimeAccumulator offlineAcc;

const float PULSES_PER_LITER = 60;
unsigned long lastSampleTime = 0;

const int flowSensorPin = 25;
volatile unsigned long pulses = 0;
unsigned long lastPulses = 0;

void IRAM_ATTR increase() {
  pulses++;
}

void wifiStatus() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Connecting to WiFi");
  }
  else {
    Serial.println("Connected");
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

void readTotalTimeAndPulse() {
  if (millis() < lastSampleTime + 1000) return;

  unsigned long currentPulses;
  noInterrupts();
  currentPulses = pulses;
  interrupts();
  
  unsigned long now = millis();
  unsigned long pulsesSinceLastRead = currentPulses - lastPulses;
  unsigned long timeSinceLastRead = now - lastSampleTime;

  lastPulses = currentPulses;
  lastSampleTime = now;

  offlineAcc.totalTimeMs += timeSinceLastRead;
  offlineAcc.totalPulses += pulsesSinceLastRead;
}

void publishSensorReads() {
  if (offlineAcc.totalTimeMs == 0 || !mqttClient.connected()) return;

  const float pulsesPerLiter = 60.0;

  float totalLiters = offlineAcc.totalPulses / pulsesPerLiter;
  float totalMinutes = offlineAcc.totalTimeMs / 60000.0;
  float flowRateLpm = totalLiters / totalMinutes;

  char payload[64];
  snprintf(payload, sizeof(payload), "{\"flow_lpm\":%.3f,\"minutes\":%.2f}", flowRateLpm, totalMinutes);

  if (mqttClient.publish("sensor/flow-rate-lpm", payload)) {
    offlineAcc.totalTimeMs = 0;
    offlineAcc.totalPulses = 0;
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
  readTotalTimeAndPulse();

  wifiStatus();
  mqttStatus();
  
  publishSensorReads();
  mqttClient.loop();

}