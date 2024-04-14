// BLYNK account details
#define BLYNK_TEMPLATE_ID "user14"                   // replace with your blynk username
#define BLYNK_TEMPLATE_NAME "user14@server.wyns.it"  // replace with your blynk server

//library imports
#include <Wire.h>
#include <MPU6050.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp8266.h>

// sensor name
MPU6050 mpu;

// float parameters
float velocityms = 0.0;
float velocitykph = 0.0;
float acceleration = 0.0;
float previousacceleration = 0.0;
float highestSpeedms = 0.0;
float highestSpeedkph = 0.0;
float test = 0;

// blynk and wifi connection details
char auth[] = "DMJPfkUVvBdqNXPLgs8uED8BRAqaicJ9";  // replace with your Blynk authentication token
char ssid[] = "Thars samsung a40";                 // replace with your WiFi SSID
char pass[] = "Tharsps4";                          // replace with your WiFi password

void setup() {
  // start serial monitor and Wire
  Serial.begin(9600);
  Wire.begin();

  // initialize and test sensor
  mpu.initialize();
  Serial.println("Testing MPU-6050...");
  Serial.println("Initialized successfully");
  Serial.println("Reading sensor data...");
  delay(500);

  // connect to wifi
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, pass);
  int wifi_ctr = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");  // if you cant connect serial monitor will keep printing .........
  }
  Serial.println("WiFi connected");

  // begin Blynk
  Blynk.begin(auth, ssid, pass, "server.wyns.it", 8081);

//  BLYNK account details
#define BLYNK_TEMPLATE_ID "user14"
#define BLYNK_TEMPLATE_NAME "user14@server.wyns.it"

  // serial monitor prints for checking blynk connection
  if (Blynk.connected()) {
    Serial.println("Connected to Blynk");
  } else {
    Serial.println("Failed to connect to Blynk");
  }
}

void loop() {
  Blynk.run();  // Run Blynk

  //time parameters
  static unsigned long previousMillis = millis();
  unsigned long currentMillis = millis();
  float timeElapsed = (currentMillis - previousMillis) / 1000.0;  // Convert to seconds

  // Read accelerometer data
  int16_t ax, ay, az, gx, gy, gz;                // Add variables for gyroscope data
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);  // Provide all six arguments

  acceleration = ay / 16384.0;                        // Convert raw accelerometer data to g (gravity)
  if (acceleration < 0.05 && acceleration > -0.05) {  // margin for error so at standstill theres no intefierence from earths gravity
    acceleration = 0.0;
  }
  // change speed with acceleration
  velocityms += 0.5 * (acceleration + previousacceleration) * timeElapsed * 10;  // Integrate acceleration to estimate velocityms
  velocitykph = velocityms * 3.6;                                                // convert from m/s to km/h

  // Update previous acceleration for the next iteration
  previousacceleration = acceleration;

  // Update previous time for the next iteration
  previousMillis = currentMillis;

  if (velocityms < highestSpeedms) {  // Update highest speed if current velocityms is greater
    highestSpeedms = velocityms;
    highestSpeedkph = highestSpeedms * 3.6;
    Blynk.virtualWrite(V12, -highestSpeedkph);  // Update Blynk with highest speed
  }

  // Send data to Blynk app
  Blynk.virtualWrite(V7, -acceleration);  // Sending acceleration to virtual pin V7
  Blynk.virtualWrite(V13, -velocitykph);  // Sending velocityms to virtual pin V8
}

BLYNK_WRITE(V10) {  // Blynk button to reset highest speed
  if (param.asInt() == 1) {
    highestSpeedms = 0.0;
    Blynk.virtualWrite(V9, highestSpeedms);  // Update Blynk with reset highest speed
  }
}
BLYNK_WRITE(V11) {  // Blynk button to reset current speed
  if (param.asInt() == 1) {
    velocityms = 0.0;
    Blynk.virtualWrite(V8, velocityms);  // Update Blynk with reset current speed
  }
}