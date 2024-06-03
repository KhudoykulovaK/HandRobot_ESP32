#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "ESP32Servo.h"
#include <ArduinoOTA.h>

// Define the maximum and minimum angles for the first servo
#define SERVO_ONE_MAX 180
#define SERVO_ONE_MIN 0

// Define the maximum and minimum angles for the second servo
#define SERVO_TWO_MAX 175
#define SERVO_TWO_MIN 85

// WiFi credentials
const char* ssid = "IoTPrivate";
const char* password = "iotprivate303";

// MQTT broker details
const char* mqtt_server = "leapservo.ddns.net";
const int mqtt_port = 1883;
const char* mqtt_topic = "leap/servo";

// Define GPIO pins for servos
const int firstPin = 25;
const int secondPin = 33;

// Create servo objects
Servo firstServo;
Servo secondServo;

// Initial servo angles
int f_angle = 90;
int s_angle = 90;

// Target servo angles
int f_final_angle = 90;
int s_final_angle = 90;

// WiFi and MQTT client objects
WiFiClient espClient;
PubSubClient client(espClient);

// Function declarations
void servo_handle(void);
void setup_wifi();
void callback(char* topic, byte* payload, unsigned int length);
void reconnect();

 // Connect to WiFi network
void setup_wifi(){
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

// MQTT callback function
void callback(char* topic, byte* payload, unsigned int length){
  String topicStr = String(topic);  
  String msg;

  // Construct the message from the payload
  for (unsigned int i = 0; i < length; i++) {
    msg += (char)payload[i];
  }
  msg.trim(); // Remove any leading/trailing whitespace

  // Check if the topic is correct
  if (topicStr != mqtt_topic){
    Serial.print("Wrong topic: ");
    Serial.println(topicStr);
    Serial.print("Msg: ");
    Serial.println(msg);
    return;
  }

  // Find the position of the comma
  int commaIndex = msg.indexOf(',');

  // Extract the substrings before and after the comma
  String xStr = msg.substring(0, commaIndex);
  String yStr = msg.substring(commaIndex + 1);

  // Convert these substrings to integers
  int x = xStr.toInt();
  int y = yStr.toInt();

  // Constrain the angles within the defined limits
  if (x < SERVO_ONE_MIN){
    x = SERVO_ONE_MIN;
  }

  if (x > SERVO_ONE_MAX){
    x = SERVO_ONE_MAX;
  }

  if (y < SERVO_TWO_MIN){
    y = SERVO_TWO_MIN;
  }

  if (y > SERVO_TWO_MAX){
    y = SERVO_TWO_MAX;
  }

  // Map the angles to the servo's range of motion
  x = map(x, SERVO_ONE_MIN, SERVO_ONE_MAX, SERVO_ONE_MAX, SERVO_ONE_MIN);
  y = map(y, SERVO_TWO_MIN, SERVO_TWO_MAX, SERVO_TWO_MAX, SERVO_TWO_MIN);

  // Set the final angles
  f_final_angle = x;
  s_final_angle = y;
}

// Reconnect to MQTT broker
void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP32Client")) {
      Serial.println("connected");
      client.subscribe(mqtt_topic);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

// Initialize serial communication
void setup() {
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  // Attach the servos to the GPIO pins and set initial positions
  firstServo.attach(firstPin);
  firstServo.write(90);
  secondServo.attach(secondPin);
  secondServo.write(90);

  /*
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_SPIFFS
      type = "filesystem";
    }
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });
  ArduinoOTA.begin();
  */
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  // ArduinoOTA.handle();

  servo_handle();
}

void servo_handle(void)
{
  static uint32_t prev_time = millis();

  int diff_one, diff_two;
  int q_one, q_two;

  // Calculate the difference between current and target angles for the first servo
  if (f_angle > f_final_angle){
    diff_one = f_angle - f_final_angle;
    q_one = -1;
  }
  else{
    diff_one = f_final_angle - f_angle;
    q_one = 1;
  }

  // Calculate the difference between current and target angles for the second servo
  if (s_angle > s_final_angle){
    diff_two = s_angle - s_final_angle;
    q_two = -1;
  }
  else{
    diff_two = s_final_angle - s_angle;
    q_two = 1;
  }

  // Increment the angles towards the target angles gradually
  if (millis() - prev_time > 10){
    if (diff_one > 0){
      f_angle = f_angle + q_one;
      firstServo.write(f_angle);
    }

    if (diff_two > 0){
      s_angle = s_angle + q_two;
      secondServo.write(s_angle);
    }

    prev_time = millis();
  }
}
