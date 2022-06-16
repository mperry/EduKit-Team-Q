#include <Arduino.h>
#include <ArduinoJson.h>
#include <M5Core2.h>
#include <MQTTClient.h>
#include <string>
#include <WiFiClientSecure.h>
#include <FastLED.h>

#include "secrets.h"
#include "WiFi.h"

// The MQTT topics that this device should publish/subscribe to
#define AWS_IOT_PUBLISH_TOPIC AWS_IOT_PUBLISH_TOPIC_THING
#define AWS_IOT_SUBSCRIBE_TOPIC AWS_IOT_SUBSCRIBE_TOPIC_THING

#define PORT 8883

// motion
#define NUM_LEDS 10
#define LED_PINS 25


WiFiClientSecure wifiClient = WiFiClientSecure();
MQTTClient mqttClient = MQTTClient(256);

// motion

CRGB leds[NUM_LEDS];

uint8_t hue = 0;

int sensor = 26;              // the pin that the sensor is atteched to
int state = LOW;             // by default, no motion detected
int val = 0;                 // variable to store the sensor status (value)

// message variables

long lastPingMsg = 0;
int TIME_BETWEEN_PINGS = 30000; // milliseconds
int CONNECT_DELAY = 500;

bool USE_WIFI = false;

// IMU variables - Inertial Motion Unit

const int MAX_IMU_VALUES = 10;
long lastImuMsg = 0;
long minTimeBetweenImuMessages = 5000;
int imuDelay = 0; // milliseconds

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


String BoolToString(bool b) {
  return b ? "true" : "false";
}

void lcdPrint(const String & s) {
    M5.Lcd.printf(s.c_str());
}

void lcdNewLine() {
  lcdPrint("\n");
}

bool useWifi() {
  return USE_WIFI;

}

void sendTestMessage(int delayMs, const String & msg) {
  StaticJsonDocument<200> jsonDoc;
  char jsonBuffer[512];

  delay(delayMs);

  JsonObject thingObject = jsonDoc.createNestedObject("ThingInformation");
  thingObject["time"] = millis();
  thingObject["team"] = TEAMNAME;

  jsonDoc ["message"] = msg;

  serializeJsonPretty(jsonDoc, jsonBuffer);
  Serial.println("");
  Serial.print("Publishing to " + AWS_IOT_PUBLISH_TOPIC + ": ");
  Serial.println(jsonBuffer);

  // M5.Lcd.clear();
  M5.Lcd.printf("Test msg sent at %d\n", millis());
  // lcdPrint("Hi from Team Q\n");

  // Publish json to AWS IoT Core
  bool b = mqttClient.publish(AWS_IOT_PUBLISH_TOPIC.c_str(), jsonBuffer);
  Serial.println("Test msg publish status:" + BoolToString(b) + "\n");
}



void imuSetup() {
  // Serial.begin(115200);
  // M5.begin();
  M5.IMU.Init();
  // M5.Lcd.setTextColor(GREEN, BLACK);
  // M5.Lcd.setTextSize(2);
}


void publishImu(float array[MAX_IMU_VALUES]) {
    sendTestMessage(100, "Simple test message from IMU3");

  long now = millis();
  if (now - lastImuMsg > minTimeBetweenImuMessages) {
    lastImuMsg = now;
    // Initialise json object and print
    StaticJsonDocument<500> jsonDoc;
    char jsonBuffer[1024];

    JsonObject thingObject = jsonDoc.createNestedObject("ThingInformation");
    thingObject["time"] = millis();
    thingObject["team"] = TEAMNAME;

    JsonObject imuObject = jsonDoc.createNestedObject("IMU");
    imuObject["accX"] = array[0];
    imuObject["accY"] = array[1];
    imuObject["accZ"] = array[2];

    imuObject["gyroX"] = array[3];
    imuObject["gyroY"] = array[4];
    imuObject["gyroZ"] = array[5];

    imuObject["pitch"] = array[6];
    imuObject["roll"] = array[7];
    imuObject["yaw"] = array[8];

    imuObject["temp"] = array[9];

    serializeJsonPretty(jsonDoc, jsonBuffer);
    Serial.println("");
    Serial.print("Publishing to " + AWS_IOT_PUBLISH_TOPIC + ": ");
    Serial.println(jsonBuffer);

    // M5.Lcd.clear();
    M5.Lcd.printf("IMU sent at %d\n", millis());
    // lcdPrint("Hi from Team Q\n");

    sendTestMessage(100, "Simple test message from IMU4");

    // Publish json to AWS IoT Core
    bool b = mqttClient.publish(AWS_IOT_PUBLISH_TOPIC.c_str(), jsonBuffer);
    Serial.print("IMU publish status:" + BoolToString(b) + "\n");

  }
}

void imuLoop() {
  sendTestMessage(100, "Simple test message from IMU1");

  // Get accelerometer values
  M5.IMU.getAccelData(&accX, &accY, &accZ);
  // M5.Lcd.setCursor(0, 20);
  // M5.Lcd.printf("accX,   accY,  accZ");
  // M5.Lcd.setCursor(0, 42);
  // M5.Lcd.printf("%5.2f  %5.2f  %5.2f G", accX, accY, accZ);

  // Get gryoscope values
  M5.IMU.getGyroData(&gyroX, &gyroY, &gyroZ);
  // M5.Lcd.setCursor(0, 70);
  // M5.Lcd.printf("gyroX,  gyroY, gryoZ");
  // M5.Lcd.setCursor(0, 92);
  // M5.Lcd.printf("%6.2f %6.2f%6.2f o/s", gyroX, gyroY, gyroZ);

  // Get spatial values
  M5.IMU.getAhrsData(&pitch, &roll, &yaw);
  // M5.Lcd.setCursor(0, 120);
  // M5.Lcd.printf("pitch,  roll,  yaw");
  // M5.Lcd.setCursor(0, 142);
  // M5.Lcd.printf("%5.2f %5.2f  %5.2f deg", pitch, roll, yaw);

  // Get temperature value
  M5.IMU.getTempData(&temp);
  // M5.Lcd.setCursor(0, 175);
  // M5.Lcd.printf("Temperature : %.2f C", temp);
  float array[MAX_IMU_VALUES] = {accX, accY, accZ, gyroX, gyroY, gyroZ, pitch, roll, yaw, temp};
    sendTestMessage(100, "Simple test message from IMU2");

  publishImu(array);
  // delay(imuDelay);
}

void connectWifi()
{
  if (useWifi()) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.print(WIFI_SSID);

    // Connect to the specified Wi-Fi network
    // Retries every 500ms
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED)
    {
      Serial.print(".");
      delay(CONNECT_DELAY);
    }
    Serial.println();

  }
 
}

// Handle message from AWS IoT Core
void messageHandler(String &topic, String &payload)
{
  Serial.println("Incoming: " + topic + " - " + payload);
  lcdPrint("Incoming: " + topic + " - " + payload + "\n");

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
void connectAWSIoTCore() {
  if (useWifi()) {

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
      delay(CONNECT_DELAY);
    }
    Serial.println("Connected to AWS IoT Core!");

    // Subscribe to the topic on AWS IoT
    mqttClient.subscribe(AWS_IOT_SUBSCRIBE_TOPIC.c_str());
  }
 
}


void motionSetup() {
  pinMode(sensor, INPUT);    // initialize sensor as an input
  M5.begin(true, true, true, true); // Init M5Core2.
  Serial.begin(115200);
  
  FastLED.addLeds<NEOPIXEL, LED_PINS>(leds, NUM_LEDS);
  Serial.println(F("Testing Motion Sensor"));
  M5.Lcd.print("Device successfully hit setup");
}


void setup() {
  Serial.begin(115200);

  // Initialise M5 LED to off
  M5.begin();
  // lcdPrint("M5 initialised at " + millis());
  lcdPrint("M5 initialised.");

  M5.Axp.SetLed(false);
  imuSetup();
  //motionSetup();

  connectWifi();
  connectAWSIoTCore();

}

void reconnectToIot() {
  // Reconnection Code if disconnected from the MQTT Client/Broker
  if (!mqttClient.connected()) {
    Serial.println("Device has disconnected from MQTT Broker, reconnecting...");
    connectAWSIoTCore();
  }
  mqttClient.loop();
}


void publishPing() {

  long now = millis();
  if (now - lastPingMsg > TIME_BETWEEN_PINGS) {
    lastPingMsg = now;
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

    // M5.Lcd.clear();
    M5.Lcd.printf("Ping sent at %d\n", millis());
    // lcdPrint("Hi from Team Q\n");

    // Publish json to AWS IoT Core
    bool b = mqttClient.publish(AWS_IOT_PUBLISH_TOPIC.c_str(), jsonBuffer);
    Serial.println("Ping publish status:" + BoolToString(b) + "\n");
  }
}

void motionLoop(){
  delay(500);
  val = digitalRead(sensor);   // read sensor value
  if (val == HIGH) {            // if motion detected
    M5.Lcd.setCursor(0,0);
    Serial.println("Hey I got you!!!");
    M5.Lcd.setTextSize(2);
    M5.Lcd.println("Hey We got you!!!");
    M5.Axp.SetLed(true);
    FastLED.showColor(CHSV(255, 0, 0));
 } 
  else {
    M5.Lcd.clearDisplay();
    Serial.println("Hey where did you go?");
    M5.Axp.SetLed(false);
    FastLED.clear();
 }
}

void loop() {
  
  reconnectToIot();
  // publishPing();
  sendTestMessage(500, "Simple test message");

  // reconnectToIot();
  // imuLoop();
  // motionLoop();
}

