#include <Wire.h>
#include <MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>
#include <TinyGPSPlus.h>
#include <PubSubClient.h>
#include <EEPROM.h>

// Pins
#define PPM_PIN 2
#define RIGHT_AILERON_SERVO_PIN 8
#define LEFT_AILERON_SERVO_PIN 9
#define ELEVATOR_SERVO_PIN 10
#define ESC1_PIN 0
#define ESC2_PIN 1
#define GPS_RX 20
#define GPS_TX 21
#define PPM_FRAME_GAP 3000
#define CALIBRATION_SAMPLES 500

// WiFi & MQTT
const char* ssid = "telemetry";
const char* password = "telemetry";
const char* mqtt_server = "test.mosquitto.org";
WiFiClient espClient;
PubSubClient mqttClient(espClient);

// Objects
MPU6050 mpu;
Adafruit_BMP280 bmp;
WebServer server(80);
Servo esc1, esc2, servoRight, servoLeft, servoElevator;
TinyGPSPlus gps;
HardwareSerial GPS_Serial(1);

// EEPROM configuration structure
struct Config {
  float kP, kI, kD;
  float altitudeKP, altitudeKD, altitudePitchKP;
  bool reverseAileron, reverseElevator;
  bool reverseRightAileron, reverseLeftAileron, reverseElevatorServo;
} config;

#define EEPROM_SIZE sizeof(Config)

// State
volatile uint16_t ppmChannels[9] = {1500};
volatile uint8_t ppmIndex = 0;
volatile uint32_t lastPPMTime = 0;
float rollFiltered = 0, pitchFiltered = 0;
float rollOffset = 0, pitchOffset = 0;
bool calibrated = false;
bool gyroStabilizationEnabled = false;
bool differentialThrustEnabled = false;
bool altitudeHoldEnabled = false;
bool altitudeHoldEngaged = false;
int throttleSignal = 1000;

// Sensor data
float accelX, accelY, accelZ, gyroX, gyroY;
float roll, pitch, dt;
float lastAltitude = 0, currentAltitude = 0, targetAltitude = 0;
float altitudeError = 0, altitudeCorrection = 0;
float altitudeDerivative = 0, altitudePitchCorrection = 0;

// GPS data
String gpsLatitude = "0.0";
String gpsLongitude = "0.0";

// Macros
#define ch1Value ppmChannels[0]
#define ch2Value ppmChannels[1]
#define ch3Value ppmChannels[2]
#define ch4Value ppmChannels[3]
#define ch7Value ppmChannels[6]
#define ch9Value ppmChannels[4]

// PPM ISR
void IRAM_ATTR ppmISR() {
  uint32_t now = micros();
  uint32_t duration = now - lastPPMTime;
  lastPPMTime = now;
  if (duration >= PPM_FRAME_GAP) ppmIndex = 0;
  else if (ppmIndex < 9) ppmChannels[ppmIndex++] = duration;
}

// Calibrate IMU
void calibrateMPU() {
  float rollSum = 0, pitchSum = 0;
  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    accelX = mpu.getAccelerationX();
    accelY = mpu.getAccelerationY();
    accelZ = -mpu.getAccelerationZ();
    float accelRoll = atan2(-accelY, accelZ) * 180 / PI;
    float accelPitch = atan2(accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180 / PI;
    rollSum += accelRoll;
    pitchSum += accelPitch;
    delay(5);
  }
  rollOffset = rollSum / CALIBRATION_SAMPLES;
  pitchOffset = pitchSum / CALIBRATION_SAMPLES;
  calibrated = true;
}

// EEPROM functions
void loadEEPROM() {
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.get(0, config);
  if (isnan(config.kP)) {
    config.kP = 2.7;
    config.kI = 0.0;
    config.kD = 2.0;
    config.altitudeKP = 2.0;
    config.altitudeKD = 0.8;
    config.altitudePitchKP = 1.5;
    config.reverseAileron = false;
    config.reverseElevator = false;
    config.reverseRightAileron = false;
    config.reverseLeftAileron = false;
    config.reverseElevatorServo = false;
    EEPROM.put(0, config);
    EEPROM.commit();
  }
}

void saveEEPROM() {
  EEPROM.put(0, config);
  EEPROM.commit();
}

// Web UI Handlers
void handleRoot() {
  String html = R"rawliteral(
  <!DOCTYPE html><html><head><title>Flight Controller</title>
  <style>
    .section {
      margin-bottom: 20px;
      padding: 15px;
      border: 1px solid #ddd;
      border-radius: 5px;
    }
    h1, h2 {
      color: #333;
    }
  </style>
  <script>
    function fetchGPS() {
      fetch('/gps')
        .then(response => response.json())
        .then(data => {
          document.getElementById('lat').textContent = data.lat;
          document.getElementById('lon').textContent = data.lon;
        });
    }
    setInterval(fetchGPS, 1000);
    window.onload = fetchGPS;
  </script></head><body>
  <div class="section">
    <h1>PID Settings</h1>
    <form method='GET' action='/update'>
      kP:<input name='kP' value=")rawliteral" + String(config.kP) + R"rawliteral("><br>
      kI:<input name='kI' value=")rawliteral" + String(config.kI) + R"rawliteral("><br>
      kD:<input name='kD' value=")rawliteral" + String(config.kD) + R"rawliteral("><br>
      Altitude KP:<input name='altitudeKP' value=")rawliteral" + String(config.altitudeKP) + R"rawliteral("><br>
      Altitude KD:<input name='altitudeKD' value=")rawliteral" + String(config.altitudeKD) + R"rawliteral("><br>
      Altitude Pitch KP:<input name='altitudePitchKP' value=")rawliteral" + String(config.altitudePitchKP) + R"rawliteral("><br>
      <input type='submit' value='Update'>
    </form>
  </div>

  <div class="section">
    <h2>Channel Reversal</h2>
    <form method='GET' action='/reverse'>
      Aileron Channel: <input type='checkbox' name='aileron' )rawliteral" + (config.reverseAileron ? "checked" : "") + R"rawliteral(><br>
      Elevator Channel: <input type='checkbox' name='elevator' )rawliteral" + (config.reverseElevator ? "checked" : "") + R"rawliteral(><br>
      <input type='submit' value='Save Channel Reversals'>
    </form>
  </div>

  <div class="section">
    <h2>Individual Servo Reversal</h2>
    <form method='GET' action='/reverseServos'>
      Right Aileron: <input type='checkbox' name='rightAileron' )rawliteral" + (config.reverseRightAileron ? "checked" : "") + R"rawliteral(><br>
      Left Aileron: <input type='checkbox' name='leftAileron' )rawliteral" + (config.reverseLeftAileron ? "checked" : "") + R"rawliteral(><br>
      Elevator Servo: <input type='checkbox' name='elevatorServo' )rawliteral" + (config.reverseElevatorServo ? "checked" : "") + R"rawliteral(><br>
      <input type='submit' value='Save Servo Reversals'>
    </form>
  </div>

  <div class="section">
    <form action='/calibrate'><input type='submit' value='Calibrate IMU'></form>
  </div>

  <div class="section">
    <h2>GPS Coordinates</h2>
    Latitude: <span id='lat'>Loading...</span><br>
    Longitude: <span id='lon'>Loading...</span><br>
  </div>
  </body></html>
  )rawliteral";
  server.send(200, "text/html", html);
}

void handleUpdate() {
  if (server.hasArg("kP")) config.kP = server.arg("kP").toFloat();
  if (server.hasArg("kI")) config.kI = server.arg("kI").toFloat();
  if (server.hasArg("kD")) config.kD = server.arg("kD").toFloat();
  if (server.hasArg("altitudeKP")) config.altitudeKP = server.arg("altitudeKP").toFloat();
  if (server.hasArg("altitudeKD")) config.altitudeKD = server.arg("altitudeKD").toFloat();
  if (server.hasArg("altitudePitchKP")) config.altitudePitchKP = server.arg("altitudePitchKP").toFloat();
  saveEEPROM();
  server.send(200, "text/plain", "PID settings updated. <a href='/'>Back</a>");
}

void handleReverse() {
  config.reverseAileron = server.hasArg("aileron");
  config.reverseElevator = server.hasArg("elevator");
  saveEEPROM();
  server.send(200, "text/plain", "Channel reversal settings updated. <a href='/'>Back</a>");
}

void handleReverseServos() {
  config.reverseRightAileron = server.hasArg("rightAileron");
  config.reverseLeftAileron = server.hasArg("leftAileron");
  config.reverseElevatorServo = server.hasArg("elevatorServo");
  saveEEPROM();
  server.send(200, "text/plain", "Servo reversal settings updated. <a href='/'>Back</a>");
}

void handleCalibrate() {
  calibrateMPU();
  server.send(200, "text/plain", "IMU Calibrated. <a href='/'>Back</a>");
}

void handleGPS() {
  String json = "{\"lat\":\"" + gpsLatitude + "\",\"lon\":\"" + gpsLongitude + "\"}";
  server.send(200, "application/json", json);
}

// Connection handling
unsigned long lastWiFiCheck = 0, lastMQTTCheck = 0;

void checkWiFi() {
  if (WiFi.status() != WL_CONNECTED && millis() - lastWiFiCheck > 5000) {
    Serial.println("Reconnecting WiFi...");
    WiFi.disconnect();
    WiFi.begin(ssid, password);
    lastWiFiCheck = millis();
  }
}

void checkMQTT() {
  if (WiFi.status() == WL_CONNECTED && !mqttClient.connected() && millis() - lastMQTTCheck > 5000) {
    Serial.println("Connecting to MQTT...");
    if (mqttClient.connect("ESP32FlightController")) {
      Serial.println("MQTT Connected");
    } else {
      Serial.print("MQTT Failed, state: ");
      Serial.println(mqttClient.state());
    }
    lastMQTTCheck = millis();
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin(6, 7);
  
  // Initialize EEPROM and load configuration
  EEPROM.begin(EEPROM_SIZE);
  loadEEPROM();
  
  mpu.initialize();
  if (!mpu.testConnection()) while (1);
  calibrateMPU();
  if (!bmp.begin(0x76)) while (1);

  esc1.attach(ESC1_PIN, 1000, 2000);
  esc2.attach(ESC2_PIN, 1000, 2000);
  servoRight.attach(RIGHT_AILERON_SERVO_PIN, 1000, 2000);
  servoLeft.attach(LEFT_AILERON_SERVO_PIN, 1000, 2000);
  servoElevator.attach(ELEVATOR_SERVO_PIN, 1000, 2000);

  esc1.writeMicroseconds(1000);
  esc2.writeMicroseconds(1000);

  pinMode(PPM_PIN, INPUT_PULLUP);
  attachInterrupt(PPM_PIN, ppmISR, RISING);

  GPS_Serial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);

  WiFi.begin(ssid, password);
  mqttClient.setServer(mqtt_server, 1883);

  server.on("/", handleRoot);
  server.on("/update", handleUpdate);
  server.on("/reverse", handleReverse);
  server.on("/reverseServos", handleReverseServos);
  server.on("/calibrate", handleCalibrate);
  server.on("/gps", handleGPS);
  server.begin();
}

void loop() {
  static unsigned long lastTime = millis();
  static unsigned long lastMqttPublish = millis();

  checkWiFi();
  checkMQTT();
  mqttClient.loop();
  server.handleClient();

  while (GPS_Serial.available()) {
    gps.encode(GPS_Serial.read());
    if (gps.location.isUpdated()) {
      gpsLatitude = String(gps.location.lat(), 6);
      gpsLongitude = String(gps.location.lng(), 6);
    }
  }

  currentAltitude = bmp.readAltitude(1013.25);
  bool newAltHold = ch9Value >= 1800;
  if (newAltHold && !altitudeHoldEngaged) {
    targetAltitude = currentAltitude;
    altitudeHoldEngaged = true;
  } else if (!newAltHold) altitudeHoldEngaged = false;
  altitudeHoldEnabled = newAltHold;
  gyroStabilizationEnabled = (ch9Value >= 1400);
  differentialThrustEnabled = (ch7Value >= 1500);

  unsigned long now = millis();
  dt = (now - lastTime) / 1000.0;
  lastTime = now;

  accelX = mpu.getAccelerationX();
  accelY = mpu.getAccelerationY();
  accelZ = -mpu.getAccelerationZ();
  gyroX = mpu.getRotationX() / 131.0;
  gyroY = mpu.getRotationY() / 131.0;

  float accelRoll = atan2(-accelY, accelZ) * 180 / PI - rollOffset;
  float accelPitch = atan2(accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180 / PI - pitchOffset;
  rollFiltered = 0.98 * (rollFiltered + gyroX * dt) + 0.02 * accelRoll;
  pitchFiltered = 0.98 * (pitchFiltered + gyroY * dt) + 0.02 * accelPitch;

  float rollError = -rollFiltered;
  float pitchError = -pitchFiltered;
  float rollCorrection = gyroStabilizationEnabled ? config.kP * rollError : 0;
  float pitchCorrection = gyroStabilizationEnabled ? config.kP * pitchError : 0;

  if (altitudeHoldEnabled && altitudeHoldEngaged) {
    altitudeError = targetAltitude - currentAltitude;
    altitudeDerivative = (lastAltitude - currentAltitude) / dt;
    altitudeCorrection = config.altitudeKP * altitudeError + config.altitudeKD * altitudeDerivative;
    altitudePitchCorrection = config.altitudePitchKP * altitudeError;
    lastAltitude = currentAltitude;
  } else {
    altitudeCorrection = 0;
    altitudePitchCorrection = 0;
  }

  static float smoothAileron = 90, smoothElevator = 90;
  float alpha = 0.5;
  int targetAileron = constrain(map(ch1Value, 1000, 2000, 0, 180) + rollCorrection, 0, 180);
  int targetElevator = constrain(map(ch2Value, 1000, 2000, 0, 180) + pitchCorrection + altitudePitchCorrection, 0, 180);
  smoothAileron = alpha * targetAileron + (1 - alpha) * smoothAileron;
  smoothElevator = alpha * targetElevator + (1 - alpha) * smoothElevator;
  throttleSignal = constrain(ch3Value + altitudeCorrection, 1000, 2000);

  // Apply servo reversals
  int rightAileronPos = smoothAileron;
  int leftAileronPos = smoothAileron;
  int elevatorPos = smoothElevator;

  if (config.reverseRightAileron) rightAileronPos = 180 - rightAileronPos;
  if (config.reverseLeftAileron) leftAileronPos = 180 - leftAileronPos;
  if (config.reverseElevatorServo) elevatorPos = 180 - elevatorPos;

  if (config.reverseAileron) {
    servoRight.write(leftAileronPos);  // Note the swap when channel is reversed
    servoLeft.write(rightAileronPos);
  } else {
    servoRight.write(rightAileronPos);
    servoLeft.write(leftAileronPos);
  }

  if (config.reverseElevator) {
    servoElevator.write(180 - elevatorPos);
  } else {
    servoElevator.write(elevatorPos);
  }

  if (differentialThrustEnabled) {
    int yawOffset = map(ch4Value, 1000, 2000, -100, 100);
    int esc1Throttle = constrain(throttleSignal + yawOffset, 1000, 2000);
    int esc2Throttle = constrain(throttleSignal - yawOffset, 1000, 2000);
    esc1.writeMicroseconds(esc1Throttle);
    esc2.writeMicroseconds(esc2Throttle);
  } else {
    esc1.writeMicroseconds(throttleSignal);
    esc2.writeMicroseconds(throttleSignal);
  }

  if (millis() - lastMqttPublish > 500 && mqttClient.connected()) {
    String telemetryJson = "{";
    telemetryJson += "\"altitude\":" + String(currentAltitude, 2) + ",";
    telemetryJson += "\"pitch\":" + String(pitchFiltered, 2) + ",";
    telemetryJson += "\"roll\":" + String(rollFiltered, 2) + ",";
    telemetryJson += "\"latitude\":" + gpsLatitude + ",";
    telemetryJson += "\"longitude\":" + gpsLongitude + ",";
    telemetryJson += "\"gyroStabilizationEnabled\":" + String(gyroStabilizationEnabled ? "true" : "false") + ",";
    telemetryJson += "\"differentialThrustEnabled\":" + String(differentialThrustEnabled ? "true" : "false") + ",";
    telemetryJson += "\"altitudeHoldEnabled\":" + String(altitudeHoldEnabled ? "true" : "false") + ",";
    telemetryJson += "\"altitudeHoldEngaged\":" + String(altitudeHoldEngaged ? "true" : "false") + ",";
    telemetryJson += "\"throttleSignal\":" + String(throttleSignal);
    telemetryJson += "}";
    mqttClient.publish("esp32/telemetry", telemetryJson.c_str());
    lastMqttPublish = millis();
  }

  delay(20);
}
