#include <WiFi.h>
#include <WiFiUdp.h>
#include "MPU9250.h"
#include <Wire.h> // Needed for I2C to read IMU
#include <ArduinoJson.h> // Compatible amb versió 7.4.2
#include <IMU_RoboticsUB.h>   // Nom de la llibreria custom

// Device ID
const char *deviceId = "G1_Gri";

// Wi-Fi credentials
const char *ssid = "Robotics_UB";
const char *password = "rUBot_xx";

// Vibration motor settings
const int vibrationPin = 23; // Pin for the vibration motor

// Botons
const int PIN_S1 = 14;
const int PIN_S2 = 27;
int s1Status = HIGH;
int s2Status = HIGH;

// UDP settings
IPAddress receiverESP32IP(192, 168, 1, 13); // IP of receiver ESP32
IPAddress receiverComputerIP(192, 168, 1, 15); // IP of PC
const int udpPort = 12345;
WiFiUDP udp;

// IMU object
IMU imu;

// Orientation data
float Gri_roll = 0.0, Gri_pitch = 0.0, Gri_yaw = 0.0;

// Recieved Torque data
float Torque_roll1 = 0.0, Torque_roll2 = 0.0, Torque_pitch = 0.0, Torque_yaw = 0.0;

void connectToWiFi() {
  Serial.print("Connecting to Wi-Fi");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("\nWi-Fi connected!");
  Serial.println("IP Address: " + WiFi.localIP().toString());
  Serial.print("ESP32 MAC Address: ");
  Serial.println(WiFi.macAddress());
}

void updateOrientation() {
  //Llegeix FIFO del DMP i actualitza càlculs interns
  imu.ReadSensor();
  // Obté els angles (roll, pitch, yaw) via GetRPW()
  float* rpw = imu.GetRPW();
  Gri_yaw = rpw[2];
  Gri_pitch = rpw[1];
  Gri_roll = rpw[0];
  s1Status = digitalRead(PIN_S1);
  s2Status = digitalRead(PIN_S2);
}

void sendOrientationUDP() {
  JsonDocument doc;
  doc["device"] = deviceId;
  doc["roll"] = Gri_roll;
  doc["pitch"] = Gri_pitch;
  doc["yaw"] = Gri_yaw;
  doc["s1"] = s1Status;
  doc["s2"] = s2Status;

  char jsonBuffer[512];
  serializeJson(doc, jsonBuffer, sizeof(jsonBuffer));

  // Send to ESP32 Servos
  udp.beginPacket(receiverESP32IP, udpPort);
  udp.write((const uint8_t*)jsonBuffer, strlen(jsonBuffer));
  udp.endPacket();

  // Send to Computer
  udp.beginPacket(receiverComputerIP, udpPort);
  udp.write((const uint8_t*)jsonBuffer, strlen(jsonBuffer));
  udp.endPacket();
}

void recieveTorquesUDP() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    byte packetBuffer[512];
    int len = udp.read(packetBuffer, 512);
    if (len > 0) {
      packetBuffer[len] = '\0';
      Serial.print("Received packet size: ");
      Serial.println(packetSize);
      Serial.print("Received: ");
      Serial.println((char*)packetBuffer);

      JsonDocument doc; 
      DeserializationError error = deserializeJson(doc, packetBuffer);
      if (error) {
        Serial.print(F("deserializeJson() failed: "));
        Serial.println(error.f_str());
        return;
      }

      const char* device = doc["device"];
      if (strcmp(device, "G1_Servos") == 0) {
        Torque_roll1 = doc["t_roll1"];
        Torque_roll2 = doc["t_roll2"];
        Torque_pitch = doc["t_pitch"];
        Torque_yaw = doc["t_yaw"];

        Serial.print("Torques recieved");
        Serial.print("Torque_roll1: "); Serial.print(Torque_roll1);
        Serial.print("Torque_roll2: "); Serial.print(Torque_roll2);
        Serial.print("Torque_pitch: "); Serial.print(Torque_pitch);
        Serial.print("Torque_yaw: "); Serial.print(Torque_yaw);
      } else {
        Serial.println("Unkown device.");
      }
    }
  }

  // Vibration motor control based on torque values
  float totalTorque = Torque_roll1 + Torque_pitch + Torque_yaw;

  // Convert torque to PWM value (0-255)
  int vibrationValue = constrain(totalTorque * 5, 0, 255); // Adjust the scaling factor as needed

  ledcWrite(0, vibrationValue); // Set the PWM value for the vibration motor

  Serial.print("Vibration motor value: ");
  Serial.println(vibrationValue); 
  }

void setup() {
  Serial.begin(115200);
  Wire.begin();
  delay(2000);

  // Inicialitza IMU (amb DMP)
  imu.Install();

  connectToWiFi();
  udp.begin(udpPort);
  Serial.println("UDP initialized");

  pinMode(PIN_S1, INPUT);
  pinMode(PIN_S2, INPUT);

  // Configure PWM for the vibration motor (channel 0)
  ledcSetup(0, 5000, 8); // Channel 0, frequency 5kHz, resolution 8 bits
  ledcAttachPin(vibrationPin, 0); // Attach the vibration motor to channel 0
}

void loop() {
  updateOrientation();
  sendOrientationUDP();
  recieveTorquesUDP();
  delay(10);
}
