#include <Arduino.h>
#include <ArduinoJson.h>
#include <M5Core2.h>
#include <MQTTClient.h>
#include <string>
#include <WiFiClientSecure.h>
#include "secrets.h"
#include "WiFi.h"

// The MQTT topics that this device should publish/subscribe to
#define AWS_IOT_PUBLISH_TOPIC AWS_IOT_PUBLISH_TOPIC_THING
#define AWS_IOT_SUBSCRIBE_TOPIC AWS_IOT_SUBSCRIBE_TOPIC_THING

#define PORT 8883

WiFiClientSecure wifiClient = WiFiClientSecure();
MQTTClient mqttClient = MQTTClient(256);

long lastMsg = 0;

// IMU variables - Inertial Motion Unit

float accX = 0.0F;
float accY = 0.0F;
float accZ = 0.0F;

float gyroX = 0.0F;
float gyroY = 0.0F;
float gyroZ = 0.0F;

float pitch = 0.0F;
float roll = 0.0F;
float yaw = 0.0F;

float temp = 0.0F;



void lcdPrint(const String & s) {
    M5.Lcd.printf(s.c_str());
}

void lcdNewLine() {
  lcdPrint("\n");
}

void imuSetup() {
  Serial.begin(115200);

  M5.begin();
  M5.IMU.Init();
  M5.Lcd.setTextColor(GREEN, BLACK);
  M5.Lcd.setTextSize(2);
}

void imuLoop() {
  // Get accelerometer values
  M5.IMU.getAccelData(&accX, &accY, &accZ);
  M5.Lcd.setCursor(0, 20);
  M5.Lcd.printf("accX,   accY,  accZ");
  M5.Lcd.setCursor(0, 42);
  M5.Lcd.printf("%5.2f  %5.2f  %5.2f G", accX, accY, accZ);

  // Get gryoscope values
  M5.IMU.getGyroData(&gyroX, &gyroY, &gyroZ);
  M5.Lcd.setCursor(0, 70);
  M5.Lcd.printf("gyroX,  gyroY, gryoZ");
  M5.Lcd.setCursor(0, 92);
  M5.Lcd.printf("%6.2f %6.2f%6.2f o/s", gyroX, gyroY, gyroZ);

  // Get spatial values
  M5.IMU.getAhrsData(&pitch, &roll, &yaw);
  M5.Lcd.setCursor(0, 120);
  M5.Lcd.printf("pitch,  roll,  yaw");
  M5.Lcd.setCursor(0, 142);
  M5.Lcd.printf("%5.2f %5.2f  %5.2f deg", pitch, roll, yaw);

  // Get temperature value
  M5.IMU.getTempData(&temp);
  M5.Lcd.setCursor(0, 175);
  M5.Lcd.printf("Temperature : %.2f C", temp);

  delay(10);
}

void connectWifi()
{
  Serial.print("Attempting to connect to SSID: ");
  Serial.print(WIFI_SSID);

  // Connect to the specified Wi-Fi network
  // Retries every 500ms
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(500);
  }
  Serial.println();
}

// Handle message from AWS IoT Core
void messageHandler(String &topic, String &payload)
{
  Serial.println("Incoming: " + topic + " - " + payload);

  // Parse the incoming JSON
  StaticJsonDocument<200> jsonDoc;
  deserializeJson(jsonDoc, payload);

  const bool LED = jsonDoc["LED"].as<bool>();

  // Decide to turn LED on or off
  if (LED) {
    Serial.print("LED STATE: ");
    Serial.println(LED);
    M5.Axp.SetLed(true);
  } else if (!LED) {
    Serial.print("LED_STATE: ");
    Serial.println(LED);
    M5.Axp.SetLed(false);
  }
}

// Connect to the AWS MQTT message broker
void connectAWSIoTCore()
{
  // Create a message handler
  mqttClient.onMessage(messageHandler);

  // Configure WiFiClientSecure to use the AWS IoT device credentials
  wifiClient.setCACert(AWS_CERT_CA);
  wifiClient.setCertificate(AWS_CERT_CRT);
  wifiClient.setPrivateKey(AWS_CERT_PRIVATE);

  // Connect to the MQTT broker on AWS
  Serial.print("Attempting to AWS IoT Core message broker at mqtt:\\\\");
  Serial.print(AWS_IOT_ENDPOINT);
  Serial.print(":");
  Serial.println(PORT);

  // Connect to AWS MQTT message broker
  // Retries every 500ms
  mqttClient.begin(AWS_IOT_ENDPOINT, PORT, wifiClient);
  while (!mqttClient.connect(THINGNAME.c_str())) {
    Serial.print("Failed to connect to AWS IoT Core. Error code = ");
    Serial.print(mqttClient.lastError());
    Serial.println(". Retrying...");
    delay(500);
  }
  Serial.println("Connected to AWS IoT Core!");

  // Subscribe to the topic on AWS IoT
  mqttClient.subscribe(AWS_IOT_SUBSCRIBE_TOPIC.c_str());
}

void setup() {
  Serial.begin(115200);

  // Initialise M5 LED to off
  M5.begin();
  M5.Axp.SetLed(false);

  connectWifi();
  connectAWSIoTCore();
}

void reconnect_to_iot() {
  // Reconnection Code if disconnected from the MQTT Client/Broker
  if (!mqttClient.connected()) {
    Serial.println("Device has disconnected from MQTT Broker, reconnecting...");
    connectAWSIoTCore();
  }
  mqttClient.loop();
}

void publish_ping() {

  long now = millis();
  if (now - lastMsg > 30000) {
    lastMsg = now;
    // Initialise json object and print
    StaticJsonDocument<200> jsonDoc;
    char jsonBuffer[512];

    JsonObject thingObject = jsonDoc.createNestedObject("ThingInformation");
    thingObject["time"] = millis();
    thingObject["team"] = TEAMNAME;

    jsonDoc ["message"] = "Hello, this is transmitting from the Edukit";

    serializeJsonPretty(jsonDoc, jsonBuffer);
    Serial.println("");
    Serial.print("Publishing to " + AWS_IOT_PUBLISH_TOPIC + ": ");
    Serial.println(jsonBuffer);

    M5.Lcd.clear();
    M5.Lcd.printf("Message has been sent at %d\n", millis());
    lcdPrint("Hi from Team Q\n");

    // Publish json to AWS IoT Core
    mqttClient.publish(AWS_IOT_PUBLISH_TOPIC.c_str(), jsonBuffer);
  }
}

void loop() {
  
  reconnect_to_iot();
  publish_ping();

}