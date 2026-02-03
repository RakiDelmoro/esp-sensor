#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <algorithm>
#include <esp_wifi.h>

#define uS_TO_S_FACTOR 1000000
#define SLEEP_DURATION_IN_SECONDS 1800

// Network Credentials
const char* my_ssid = "";
const char* my_password = "";
const char* my_mqtt_server = "";

unsigned long lastMsg = 0;
const long interval = 1000; 
const int waterLevelSensorPowerPin1 = 13;
const int waterLevelSensorPowerPin2 = 5;
const int waterLevelSensorPin1 = 39;
const int waterLevelSensorPin2 = 34;

WiFiClient wifiClient;
PubSubClient mqttClient(my_mqtt_server, 1883, wifiClient);

void going_to_deep_sleep() {
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  btStop();

  esp_sleep_enable_timer_wakeup(SLEEP_DURATION_IN_SECONDS * uS_TO_S_FACTOR);
  Serial.println("Entering Deep Sleep...");
  Serial.flush();
  esp_deep_sleep_start();
}

IPAddress local_IP(192, 168, 50, 77);
IPAddress gateway(192, 168, 50, 1);
IPAddress subnet(255, 255, 255, 0);

void setup_wifi() {
  // We start by connecting to a WiFi network
  WiFi.config(local_IP, gateway, subnet);
  WiFi.mode(WIFI_STA);
  WiFi.begin(my_ssid, my_password);
  Serial.print("Connecting");
  unsigned long startTime = millis();

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    if (millis() - startTime > 2000) {
        Serial.print("Can't connect to WiFi for 2seconds going to deep sleep.");
        going_to_deep_sleep();
      }
  }
}

void connectingToMqtt() {
  // Use of fixed client ID based on MAC address
  String clientId = "ESP-FIREBEETLE-" + String(ESP.getEfuseMac(), HEX); 

  unsigned long startTime = millis();
  // Loop until ESP client connected to Home assistant MQTT Server
  while (!mqttClient.connected()) {
    Serial.println("Client ID: " + clientId);
    Serial.print("Connecting to MQTT Broker...");
                                                
    // Check if it's connected within 2seconds if not it will go into deep sleep
    if (millis() - startTime > 2000) {
      going_to_deep_sleep();
    }
    else {
      // Connect Client into Home assistant Mqtt broker
      mqttClient.connect(clientId.c_str(), "mqtt_indicator_1", "mqtt");
      Serial.println("connected");
    }
  }
}

struct Sorted {
    int min;
    int mid;
    int max;
};
 
Sorted sort(int a, int b, int c);

Sorted sort(int a, int b, int c)
{
    // Calculate the XOR of all values
    int all = a ^ b ^ c; // This will be calculated in parallel by out of order cpu

    // Find the minimum and maximum values 
    int _min = std::min(std::min(a, b), c);
    int _max = std::max(std::max(a, b), c);    

    // Extract the middle value
    int middle = all ^ _min ^ _max;

    return {_min, middle, _max};
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  // Setup sensor 1 pin as OUTPUT
  pinMode(waterLevelSensorPowerPin1, OUTPUT);
  // Sensor 1 powered
  digitalWrite(waterLevelSensorPowerPin1, HIGH);
  delay(150);
  // Sensor 1 readings
  int reading1 = analogRead(waterLevelSensorPin1);
  int reading2 = analogRead(waterLevelSensorPin1);
  int reading3 = analogRead(waterLevelSensorPin1);
  Sorted sorted1 = sort(reading1, reading2, reading3);
  int waterLevelValue1 = sorted1.max - sorted1.mid > sorted1.mid - sorted1.min
    ? (sorted1.mid + sorted1.min) / 2
    : (sorted1.max + sorted1.mid) / 2;
  delay(150);
  // Sensor 1 shutdown
  digitalWrite(waterLevelSensorPowerPin1, LOW);

  // Setup sensor 2 pin as OUTPUT
  pinMode(waterLevelSensorPowerPin2, OUTPUT);
  // Sensor 2 powered
  digitalWrite(waterLevelSensorPowerPin2, HIGH);
  delay(150);
  // Sensor 2 readings
  int reading4 = analogRead(waterLevelSensorPin2);
  int reading5 = analogRead(waterLevelSensorPin2);
  int reading6 = analogRead(waterLevelSensorPin2);
  Sorted sorted2 = sort(reading4, reading5, reading6);
  int waterLevelValue2 = sorted2.max - sorted2.mid > sorted2.mid - sorted2.min
    ? (sorted2.mid + sorted2.min) / 2
    : (sorted2.max + sorted2.mid) / 2;
  delay(150);
  // Sensor 2 shutdown
  digitalWrite(waterLevelSensorPowerPin2, LOW);

  setup_wifi();
  connectingToMqtt();

  // Publish sensor 1 data to the server
  char waterLevelStr1[10];
  snprintf(waterLevelStr1, sizeof(waterLevelStr1), "%d", waterLevelValue1);

  // Publish sensor 2 data to the server
  char waterLevelStr2[10];
  snprintf(waterLevelStr2, sizeof(waterLevelStr2), "%d", waterLevelValue2);
  char jsonPayload[50];
  snprintf(jsonPayload, sizeof(jsonPayload), "{\"water_tank_1\": %d, \"water_tank_2\": %d}", waterLevelStr1, waterLevelStr2);
  Serial.println(jsonPayload);
  mqttClient.publish("esp/sensors", jsonPayload);
  mqttClient.publish("esp/sensors", jsonPayload);
  mqttClient.publish("esp/sensors", jsonPayload);

  delay(150);
  going_to_deep_sleep();
}
void loop() {
}
