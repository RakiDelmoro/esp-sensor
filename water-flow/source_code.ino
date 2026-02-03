#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <vector> 

const char* ssid = "";
char* password = "";
const char* mqtt_server = "";

const char* mqtt_user = "";
const char* mqtt_password = "";

WiFiClient espClient;
PubSubClient mqttClient(espClient);
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
  }
}

void readAndPublish() {
  if (millis() - lastSampleTime < 1000) return;
  noInterrupts();
  unsigned long now = millis();
  unsigned long pulses = pulseAccumulator;
  interrupts();

  if (!mqttClient.connect("pulses-reader", mqtt_user, mqtt_password)) return;
  unsigned long pulseDelta = pulses - lastPulseCount;
  unsigned long timeDelta = now - lastSampleTime;
  StaticJsonDocument<200> doc;
  doc["Pulses"] = pulseDelta;
  doc["Millis"] = timeDelta;
  char buffer[200];
  serializeJson(doc, buffer, sizeof(buffer));

  if (!mqttClient.publish("sensor/Pulses-Time", buffer)) return;
  lastPulseCount += pulseDelta;
  lastSampleTime += timeDelta;
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
  readAndPublish();
  wifiStatus();  
  mqttClient.loop();
}