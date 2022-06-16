#include <Arduino.h>
#include <ArduinoJson.h>
#include <M5Core2.h>
#include <MQTTClient.h>
#include <string>
#include <WiFiClientSecure.h>
#include <FastLED.h>

#include "secrets.h"
#include "WiFi.h"

#include <driver/i2s.h>
#include "data.c"

// speaker


extern const unsigned char previewR[120264];

#define CONFIG_I2S_BCK_PIN 12
#define CONFIG_I2S_LRCK_PIN 0
#define CONFIG_I2S_DATA_PIN 2
#define CONFIG_I2S_DATA_IN_PIN 34

#define Speak_I2S_NUMBER I2S_NUM_0

#define SAMPLE_RATE 44100
#define BITS_PER_SAMPLE I2S_BITS_PER_SAMPLE_16BIT

// The MQTT topics that this device should publish/subscribe to
#define AWS_IOT_PUBLISH_TOPIC AWS_IOT_PUBLISH_TOPIC_THING
#define AWS_IOT_SUBSCRIBE_TOPIC AWS_IOT_SUBSCRIBE_TOPIC_THING

#define PORT 8883
#define JSON_DOC_SIZE 512
#define JSON_BUFFER_SIZE 1024
#define MQTT_BUFFER_SIZE 1024

// motion
#define NUM_LEDS 10
#define LED_PINS 25

static const String AWS_IMU_TOPIC = THINGNAME + "/imu";

static const String AWS_MOTION_TOPIC = THINGNAME +  "/motion";
static const String ALARM_TOPIC = THINGNAME +  "/alarm";
static const String ALARM_RESET_TOPIC = THINGNAME +  "/alarm-reset";

WiFiClientSecure wifiClient = WiFiClientSecure();
MQTTClient mqttClient = MQTTClient(MQTT_BUFFER_SIZE);

// motion

CRGB leds[NUM_LEDS];

uint8_t hue = 0;

int sensor = 26;              // the pin that the sensor is atteched to
int state = LOW;             // by default, no motion detected
int val = 0;                 // variable to store the sensor status (value)

// message variables

int IMU_SLEEP = 5000;

long lastPingMsg = 0;
int TIME_BETWEEN_PINGS = 30000; // milliseconds
int CONNECT_DELAY = 500;

bool USE_WIFI = true;

// IMU variables - Inertial Motion Unit

const int MAX_IMU_VALUES = 10;
long lastImuMsg = 0;
long minTimeBetweenImuMessages = 30000;
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


void clientLoop() {
    mqttClient.loop();

}


String BoolToString(bool b) {
  return b ? "true" : "false";
}


void speakerInit() {
  M5.Axp.SetSpkEnable(true);

  esp_err_t err = ESP_OK;

  i2s_config_t i2s_config = {
      .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
      .sample_rate = SAMPLE_RATE,                           // Sample rate in Hz
      .bits_per_sample = BITS_PER_SAMPLE,                   // Sampling rate in bits
      .channel_format = I2S_CHANNEL_FMT_ALL_RIGHT,          // Channel format
      .communication_format = I2S_COMM_FORMAT_I2S,          // Communication format/protocol
      .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,             // Interrupt level
      .dma_buf_count = 2,                                   // Direct memory access buffer
      .dma_buf_len = 128,                                   // Direct memory access length
      .use_apll = false,                                    // Use AAPL clock
      .tx_desc_auto_clear = true,                           // Auto clear tx descriptor if there is underflow
  };

  // Install driver with new settings
  err += i2s_driver_install(Speak_I2S_NUMBER, &i2s_config, 0, NULL);

  // Define pin configuration for speaker and clock
  i2s_pin_config_t tx_pin_config = {
    .bck_io_num = CONFIG_I2S_BCK_PIN,
    .ws_io_num = CONFIG_I2S_LRCK_PIN,
    .data_out_num = CONFIG_I2S_DATA_PIN,
    .data_in_num = CONFIG_I2S_DATA_IN_PIN,

  };
  err += i2s_set_pin(Speak_I2S_NUMBER, &tx_pin_config);
  err += i2s_set_clk(Speak_I2S_NUMBER, SAMPLE_RATE, BITS_PER_SAMPLE, I2S_CHANNEL_MONO);
}


// Play the sound by writing via I2S
void dingDong()
{
  size_t btyes_written = 0;
  i2s_write(Speak_I2S_NUMBER, previewR, 120264, &btyes_written, portMAX_DELAY);
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
  StaticJsonDocument<JSON_DOC_SIZE> jsonDoc;
  char jsonBuffer[JSON_BUFFER_SIZE];

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
  // sendTestMessage(100, "Simple test message from IMU3");

  // Initialise json object and print
  StaticJsonDocument<JSON_DOC_SIZE> jsonDoc;
  char jsonBuffer[JSON_BUFFER_SIZE];

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
  Serial.print("Checking publish to " + AWS_IMU_TOPIC + ": ");
  Serial.println(jsonBuffer);

  // M5.Lcd.clear();
  // M5.Lcd.printf("IMU sent at %d\n", millis());
  // lcdPrint("Hi from Team Q\n");

  // sendTestMessage(100, "Simple test message from IMU4");

  // Publish json to AWS IoT Core
  // clientLoop();


  long now = millis();
  long diff = now - lastImuMsg;
  
  Serial.print("IMU diff: ");
  Serial.print(diff);
  Serial.println("");

  if (diff < minTimeBetweenImuMessages) {
      Serial.println("Skipping IMU publish.");
  } else {
    lastImuMsg = now;
    bool b = mqttClient.publish(AWS_IMU_TOPIC.c_str(), jsonBuffer);
    Serial.println("IMU publish status:" + BoolToString(b));
  }
  
  // sendTestMessage(100, "IMU publish test");

  delay(IMU_SLEEP);

}

void imuLoop() {
  // sendTestMessage(100, "Simple test message from IMU1");

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
  // float array[MAX_IMU_VALUES] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};

  // sendTestMessage(100, "Simple test message from IMU2");

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

  if (topic == ALARM_TOPIC) {
    Serial.println("Run alarm");
    dingDong();
    M5.Axp.SetLed(true);

  }

   if (topic == ALARM_RESET_TOPIC) {
    Serial.println("Reset alarm");
    dingDong();
    M5.Axp.SetLed(false);

  }


  // Parse the incoming JSON
  /*
  StaticJsonDocument<JSON_DOC_SIZE> jsonDoc;
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
  */
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
    mqttClient.subscribe(ALARM_TOPIC.c_str());
    mqttClient.subscribe(ALARM_RESET_TOPIC.c_str());


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
//  M5.begin();
  M5.begin(true, true, true, true);

  // lcdPrint("M5 initialised at " + millis());
  // lcdPrint("M5 initialised.");

  M5.Axp.SetLed(false);
  imuSetup();
  //motionSetup();

  speakerInit();
  // dingDong();

  connectWifi();
  connectAWSIoTCore();

}

void reconnectToIot() {
  if (useWifi()) {
    // Reconnection Code if disconnected from the MQTT Client/Broker
    if (!mqttClient.connected()) {
      Serial.print("Client is not connected. Last error code = ");
      Serial.println(mqttClient.lastError());
      Serial.println("Device has disconnected from MQTT Broker, reconnecting...");
      connectAWSIoTCore();
    }
    clientLoop();
  }
 
}


void publishPing() {

  long now = millis();
  if (now - lastPingMsg > TIME_BETWEEN_PINGS) {
    lastPingMsg = now;
    // Initialise json object and print
    StaticJsonDocument<JSON_DOC_SIZE> jsonDoc;
    char jsonBuffer[JSON_BUFFER_SIZE];

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
    Serial.println("Ping publish status:" + BoolToString(b));
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
  publishPing();
  // sendTestMessage(500, "Simple test message");

  // reconnectToIot();
  imuLoop();
  // motionLoop();
}

