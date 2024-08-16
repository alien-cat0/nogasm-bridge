#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

#define I2C_SLAVE_ADDR 0x08 // Must match the address used in Teensy code

// WiFi credentials
const char *ssid = "XXX";
const char *password = "XXX";

// MQTT Broker settings
const char *mqtt_server = "XXX";
const int mqtt_port = 1883;

// MQTT topics
const char *motorSpeedTopic = "homeassistant/sensor/motorspeed/state";
const char *pressureTopic = "homeassistant/sensor/pressure/state";
const char *avgPressureTopic = "homeassistant/sensor/avgpressure/state";
const char *speedSetTopic = "homeassistant/number/motorspeed/set";

WiFiClient espClient;
PubSubClient client(espClient);

int motorSpeed = 0;
int newSpeed = 0;
int pressure = 0;
int avgPressure = 0;

unsigned long lastPublishTime = 0;
const long publishInterval = 500; // 500 ms = 0.5 seconds

volatile bool newDataAvailable = false;

void publishSensorData()
{
  if (newDataAvailable)
  {
    char motorSpeedStr[8];
    char pressureStr[8];
    char avgPressureStr[8];

    snprintf(motorSpeedStr, sizeof(motorSpeedStr), "%d", motorSpeed);
    snprintf(pressureStr, sizeof(pressureStr), "%d", pressure);
    snprintf(avgPressureStr, sizeof(avgPressureStr), "%d", avgPressure);

    client.publish(motorSpeedTopic, motorSpeedStr);
    client.publish(pressureTopic, pressureStr);
    client.publish(avgPressureTopic, avgPressureStr);

    newDataAvailable = false;
  }
}

void receiveEvent(int numBytes)
{
  if (Wire.available() == 5)
  {                           // We expect 6 bytes: 2 for motor speed, 2 for pressure, 2 for avgPressure
    motorSpeed = Wire.read(); // Read motor speed (1 byte)
    int pressureHigh = Wire.read();
    int pressureLow = Wire.read();
    pressure = (pressureHigh << 8) | pressureLow;
    // Read avgPressure (2 bytes)
    int avgPressureHigh = Wire.read();
    int avgPressureLow = Wire.read();
    avgPressure = (avgPressureHigh << 8) | avgPressureLow;
    newDataAvailable = true;
  }
}

void requestEvent()
{
  Wire.write(newSpeed);
}

void setMotorSpeed(int speed)
{
  newSpeed = speed;
}

void setup_wifi()
{
  delay(10);
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void publishDiscoveryMessages()
{
  DynamicJsonDocument doc(256);

  // Motor Speed Sensor
  doc["name"] = "Nogasm Motor Speed";
  doc["state_topic"] = motorSpeedTopic;
  doc["unique_id"] = "nogasm_motor_speed_sensor";

  char buffer[256];
  serializeJson(doc, buffer);
  client.publish("homeassistant/sensor/motorspeed/config", buffer, true);

  // Pressure Sensor
  doc.clear();
  doc["name"] = "Nogasm Pressure";
  doc["state_topic"] = pressureTopic;
  doc["unique_id"] = "nogasm_pressure_sensor";

  serializeJson(doc, buffer);
  client.publish("homeassistant/sensor/pressure/config", buffer, true);

  // Average Pressure Sensor
  doc.clear();
  doc["name"] = "Nogasm Average Pressure";
  doc["state_topic"] = avgPressureTopic;
  doc["unique_id"] = "nogasm_avg_pressure_sensor";

  serializeJson(doc, buffer);
  client.publish("homeassistant/sensor/avgpressure/config", buffer, true);

  // Motor Speed Control
  doc.clear();
  doc["name"] = "Set Nogasm Motor Speed";
  doc["command_topic"] = speedSetTopic;
  doc["state_topic"] = motorSpeedTopic;
  doc["min"] = 0;
  doc["max"] = 179;
  doc["step"] = 1;
  doc["unique_id"] = "nogasm_motor_speed_control";

  serializeJson(doc, buffer);
  client.publish("homeassistant/number/motorspeed/config", buffer, true);
}
void reconnect()
{
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP32Client"))
    {
      Serial.println("connected");
      client.subscribe(speedSetTopic);
      publishDiscoveryMessages();
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void mqtt_callback(char *topic, byte *payload, unsigned int length)
{
  if (strcmp(topic, speedSetTopic) == 0)
  {
    String speedStr = "";
    for (int i = 0; i < length; i++)
    {
      speedStr += (char)payload[i];
    }
    int newSpeed = speedStr.toInt();
    if (newSpeed >= 0 && newSpeed <= 255)
    {
      setMotorSpeed(newSpeed);
    }
  }
}

void setup()
{
  Serial.begin(115200);
  Serial.println("Wire.begin");
  // Initialize I2
  Wire.begin(I2C_SLAVE_ADDR);
  Serial.println("onReceive");
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  Serial.println("ESP32 I2C Slave Ready");

  setup_wifi();

  // Initialize MQTT client
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(mqtt_callback);

  Serial.println("ESP32 I2C Slave and MQTT Client Ready");
}

void loop()
{
  if (!client.connected())
  {
    reconnect();
  }
  client.loop();

  unsigned long currentMillis = millis();
  if (currentMillis - lastPublishTime >= publishInterval)
  {
    lastPublishTime = currentMillis;
    publishSensorData();
  }
}