#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <vector> 

const char* ssid = "Zoltu";
char* password = "1029384756";
const char* mqtt_server = "192.168.50.47";

WiFiClient espClient;
PubSubClient mqttClient(espClient);

std::vector<int> sensorReads;

const int flowSensorPin = D2;
volatile unsigned long pulse = 0;

void IRAM_ATTR increase() {
  pulse++;
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
    mqttClient.connect("test-sensor", "mqtt_indicator_1", "mqtt")
    Serial.println("Connecting to MQTT...");
    if mqttClient.connected() {
      Serial.println("MQTT connected");
    }
  }
}

void readAndStore() {
  noInterrupts();
  unsigned int count = pulse;
  pulse = 0;
  interrupts();

  sensorReads.push_back(count);
}

void publishSensorReads() {
  if (sensorReads.empty() || !mqttClient.connected()) return;

  StaticJsonDocument<256> doc;
  JsonArray array = doc.to<JsonArray>();
  for (int v : sensorReads) {
    array.add(v);
  }

  String payload;
  serializeJson(doc, payload);
  mqttClient.publish("test-sensor", payload.c_str());
  sensorReads.clear();
}

void setup() {
  Serial.begin(115200);
  pinMode(flowSensorPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(flowSensorPin), increase, RISING);
  WiFi.begin(ssid, password);
  mqttClient.setServer(mqtt_server, 1883);
}

void loop() {
  readAndStore();
  publishSensorReads();
  delay(500);
}