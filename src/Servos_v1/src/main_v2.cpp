#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <ArduinoJson.h> // Compatible amb versió 7.4.2
#include <ESP32Servo.h>

// Device ID
const char *deviceId = "G1_Servos";

// Wi-Fi credentials
const char *ssid = "Robotics_UB";
const char *password = "rUBot_xx";

// UDP settings
IPAddress receiverESP32IP(192, 168, 1, 13);
IPAddress receiverComputerIP(192, 168, 1, 15);
const int udpPort = 12345;
WiFiUDP udp;

// Servo settings
Servo servo_yaw;
Servo servo_pitch;
Servo servo_roll1;
Servo servo_roll2;

// Pins
const int PIN_ANALOG_YAW = 36;
const int PIN_SIGNAL_YAW = 32;
const int PIN_ANALOG_PITCH = 39;
const int PIN_SIGNAL_PITCH = 33;
const int PIN_ANALOG_ROLL1 = 34;
const int PIN_SIGNAL_ROLL1 = 25;
const int PIN_ANALOG_ROLL2 = 35;
const int PIN_SIGNAL_ROLL2 = 27;

const float Rshunt = 1.6;

// Variables
float Gri_roll = 0.0, Gri_pitch = 0.0, Gri_yaw = 0.0;
float Torque_roll1 = 0.0, Torque_roll2 = 0.0, Torque_pitch = 0.0, Torque_yaw = 0.0;
float prevRoll1 = 0, prevRoll2 = 0, prevPitch = 0, prevYaw = 0;
float sumRoll1 = 0, sumRoll2 = 0, sumPitch = 0, sumYaw = 0;
float OldValueRoll = 0, OldValuePitch = 0, OldValueYaw = 0;
float roll = 0, pitch = 0, yaw = 0;
int s1 = 1, s2 = 1;

// AÑADIDO
bool firstPacket = true;   // para capturar la referencia la 1ª vez
float yaw_ref = 0.0f;      // referencia de yaw (independiente del Norte)


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

void receiveOrientationUDP() {
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

      JsonDocument doc;  // ✅ Versió 7
      DeserializationError error = deserializeJson(doc, packetBuffer);
      if (error) {
        Serial.print(F("deserializeJson() failed: "));
        Serial.println(error.f_str());
        return;
      }

      const char* device = doc["device"];
      if (strcmp(device, "G1_Gri") == 0) {
        Gri_roll = round(doc["roll"].as<float>());
        Gri_pitch = round(doc["pitch"].as<float>());
        Gri_yaw = round(doc["yaw"].as<float>());
        s1 = doc["s1"];
        s2 = doc["s2"];

        if (firstPacket) {
          yaw_ref = Gri_yaw; // Guarda como ref la posicion inicial del gripper
          firstPacket = false; // No lo vuelve a ejecutar pq firstPacket se queda false
        }
        Serial.print("Gri_Roll: "); Serial.print(Gri_roll);
        Serial.print(" Gri_Pitch: "); Serial.print(Gri_pitch);
        Serial.print(" Gri_Yaw: "); Serial.println(Gri_yaw);
        Serial.print("S1: "); Serial.print(s1);
        Serial.print(" S2: "); Serial.println(s2);
      } else {
        Serial.println("Unknown device.");
      }
    }
  }
}

float getCurrent(uint32_t integrationTimeMs, int pin) {
  uint32_t startTime = millis();
  float integratedCurrent = 0;
  while (millis() < startTime + integrationTimeMs) {
    uint16_t adcValue = analogRead(pin);
    integratedCurrent += ((float)adcValue / 4095.0 * 3.3) / Rshunt;
  }
  return integratedCurrent;
}

float getTorque(float& sum, int analogPin, float& previous) {
  float current = getCurrent(20, analogPin);
  sum += current;
  float diff = abs(sum - previous);
  previous = sum;
  return diff;
}

// AÑADIDO
void readTorques() {
  Torque_yaw   = getTorque(sumYaw,   PIN_ANALOG_YAW,   prevYaw);
  Torque_pitch = getTorque(sumPitch, PIN_ANALOG_PITCH, prevPitch);
  Torque_roll1 = getTorque(sumRoll1, PIN_ANALOG_ROLL1, prevRoll1);
  Torque_roll2 = getTorque(sumRoll2, PIN_ANALOG_ROLL2, prevRoll2);
}

// AÑADIDO
void sendTorquesUDP() {
  JsonDocument doc;
  doc["device"]  = "G1_Servos";     // identifica el módulo que envía
  doc["t_roll1"] = Torque_roll1;
  doc["t_roll2"] = Torque_roll2;
  doc["t_pitch"] = Torque_pitch;
  doc["t_yaw"]   = Torque_yaw;
  doc["ms"]      = (uint32_t)millis();  // opcional: sello de tiempo

  char jsonBuffer[256];
  size_t n = serializeJson(doc, jsonBuffer, sizeof(jsonBuffer));

  // → al ESP32 del gripper (para háptica/feedback si lo usas)
  udp.beginPacket(receiverESP32IP, udpPort);
  udp.write((const uint8_t*)jsonBuffer, n);
  udp.endPacket();

  // → al PC (simulación/monitorización)
  udp.beginPacket(receiverComputerIP, udpPort);
  udp.write((const uint8_t*)jsonBuffer, n);
  udp.endPacket();
}

void moveServos() {
  roll = Gri_roll;
  OldValueRoll = roll;
  pitch = Gri_pitch;
  OldValuePitch = pitch;
  yaw = Gri_yaw;
  OldValueYaw = yaw;

  float delta = 0;
  if (s1 == 0) {
    delta = 40;
    Serial.println("S1 premut → Obrint");
  }

  float yaw_rel = Gri_yaw - yaw_ref; // Cuanto he girado respecto a la orientacion inicial, no Norte

  servo_roll1.write(90 + Gri_roll + delta);
  servo_roll2.write(90 - Gri_roll);
  servo_pitch.write(90 + pitch);
  servo_yaw.write(90 + yaw);
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  delay(2000);

  connectToWiFi();
  udp.begin(udpPort);
  Serial.println("UDP initialized");

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  servo_yaw.setPeriodHertz(50);
  servo_pitch.setPeriodHertz(50);
  servo_roll1.setPeriodHertz(50);
  servo_roll2.setPeriodHertz(50);

  servo_yaw.attach(PIN_SIGNAL_YAW);
  servo_pitch.attach(PIN_SIGNAL_PITCH);
  servo_roll1.attach(PIN_SIGNAL_ROLL1);
  servo_roll2.attach(PIN_SIGNAL_ROLL2);

  pinMode(PIN_ANALOG_YAW, INPUT);
  pinMode(PIN_ANALOG_PITCH, INPUT);
  pinMode(PIN_ANALOG_ROLL1, INPUT);
  pinMode(PIN_ANALOG_ROLL2, INPUT);

  servo_yaw.write(90);
  servo_pitch.write(90);
  servo_roll1.write(90);
  servo_roll2.write(90);
}

void loop() {
  receiveOrientationUDP();
  moveServos();
  readTorques();           
  sendTorquesUDP();
  delay(10);
}
