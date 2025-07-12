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
#include <ArduinoJson.h>


// Pins
#define PPM_PIN 2
#define RIGHT_AILERON_SERVO_PIN 8
#define LEFT_AILERON_SERVO_PIN 9
#define ELEVATOR_SERVO_PIN 10
#define RUDDER 1
#define ESC1_PIN 0
#define ESC2_PIN 4
#define GPS_RX 20
#define GPS_TX 21
#define PPM_FRAME_GAP 3000
#define CALIBRATION_SAMPLES 500
#define BATTERY_ADC_PIN 3

// WiFi & MQTT
const char* ssid = "telemetry";
const char* password = "telemetry";
const char* mqtt_server = "test.mosquitto.org";
const float R1 = 9500; // ohms
const float R2 = 1800; // ohms
const float ADC_REF = 3.3; // ESP32-C3 max ADC input voltage
const int ADC_MAX = 4095;  // 12-bit ADC
WiFiClient espClient;
PubSubClient mqttClient(espClient);


struct Waypoint {
  float lat;
  float lon;
  float alt;
};

#define MAX_WAYPOINTS 10
Waypoint waypoints[MAX_WAYPOINTS];
int waypointCount = 0;
int currentWaypointIndex = 0;
bool missionStarted = false;
bool missionFinished = false;

float homeLat = 0.0, homeLon = 0.0, homeAlt = 0.0;


// Objects
MPU6050 mpu;
Adafruit_BMP280 bmp;
WebServer server(80);
Servo esc1, esc2, servoRight, servoLeft, servoElevator, servoRudder;
TinyGPSPlus gps;
HardwareSerial GPS_Serial(1);

// EEPROM configuration structure
struct Config {
  // PID constants
  float kP, kI, kD;
  float altitudeKP, altitudeKD, altitudePitchKP;
  float rollP, rollI, rollD;
  float pitchP, pitchI, pitchD;
  float yawP, yawI, yawD;
  
  // Reversal settings
  bool reverseAileron, reverseElevator;
  bool reverseRightAileron, reverseLeftAileron, reverseElevatorServo;
} config;

#define EEPROM_SIZE sizeof(Config)

// State variables
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
float accelX, accelY, accelZ, gyroX, gyroY, gyroZ;
float roll, pitch, dt;
float lastAltitude = 0, currentAltitude = 0, targetAltitude = 0;
float altitudeError = 0, altitudeCorrection = 0;
float altitudeDerivative = 0, altitudePitchCorrection = 0;

// PID state variables
float rollErrorSum = 0, lastRollError = 0;
float pitchErrorSum = 0, lastPitchError = 0;
float yawErrorSum = 0, lastYawError = 0;

// GPS data
String gpsLatitude = "0.0";
String gpsLongitude = "0.0";

// Macros for channel access
#define ch1Value ppmChannels[0]
#define ch2Value ppmChannels[1]
#define ch3Value ppmChannels[2]
#define ch4Value ppmChannels[3]
#define ch7Value ppmChannels[6]
#define ch9Value ppmChannels[4]

// PPM Interrupt Service Routine
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

// Load configuration from EEPROM
void loadEEPROM() {
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.get(0, config);
  if (isnan(config.kP)) {
    // Default PID values
    config.kP = 2.7; config.kI = 0.0; config.kD = 2.0;
    config.altitudeKP = 2.0; config.altitudeKD = 0.8; config.altitudePitchKP = 1.5;
    config.rollP = 1.5; config.rollI = 0.0; config.rollD = 0.8;
    config.pitchP = 1.5; config.pitchI = 0.0; config.pitchD = 0.8;
    config.yawP = 1.0; config.yawI = 0.0; config.yawD = 0.5;
    
    // Default reversal settings
    config.reverseAileron = false;
    config.reverseElevator = false;
    config.reverseRightAileron = false;
    config.reverseLeftAileron = false;
    config.reverseElevatorServo = false;
    
    EEPROM.put(0, config);
    EEPROM.commit();
  }
}

// Save configuration to EEPROM
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
    <h1>Stabilization PID Settings</h1>
    <form method='GET' action='/update'>
      <h2>Roll PID</h2>
      P:<input name='rollP' value=")rawliteral" + String(config.rollP) + R"rawliteral("><br>
      I:<input name='rollI' value=")rawliteral" + String(config.rollI) + R"rawliteral("><br>
      D:<input name='rollD' value=")rawliteral" + String(config.rollD) + R"rawliteral("><br>
      
      <h2>Pitch PID</h2>
      P:<input name='pitchP' value=")rawliteral" + String(config.pitchP) + R"rawliteral("><br>
      I:<input name='pitchI' value=")rawliteral" + String(config.pitchI) + R"rawliteral("><br>
      D:<input name='pitchD' value=")rawliteral" + String(config.pitchD) + R"rawliteral("><br>
      
      <h2>Yaw PID</h2>
      P:<input name='yawP' value=")rawliteral" + String(config.yawP) + R"rawliteral("><br>
      I:<input name='yawI' value=")rawliteral" + String(config.yawI) + R"rawliteral("><br>
      D:<input name='yawD' value=")rawliteral" + String(config.yawD) + R"rawliteral("><br>
      
      <h2>Altitude Hold PID</h2>
      kP:<input name='kP' value=")rawliteral" + String(config.kP) + R"rawliteral("><br>
      kI:<input name='kI' value=")rawliteral" + String(config.kI) + R"rawliteral("><br>
      kD:<input name='kD' value=")rawliteral" + String(config.kD) + R"rawliteral("><br>
      Altitude KP:<input name='altitudeKP' value=")rawliteral" + String(config.altitudeKP) + R"rawliteral("><br>
      Altitude KD:<input name='altitudeKD' value=")rawliteral" + String(config.altitudeKD) + R"rawliteral("><br>
      <input type='submit' value='Update PID'>
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
  
  if (server.hasArg("rollP")) config.rollP = server.arg("rollP").toFloat();
  if (server.hasArg("rollI")) config.rollI = server.arg("rollI").toFloat();
  if (server.hasArg("rollD")) config.rollD = server.arg("rollD").toFloat();
  if (server.hasArg("pitchP")) config.pitchP = server.arg("pitchP").toFloat();
  if (server.hasArg("pitchI")) config.pitchI = server.arg("pitchI").toFloat();
  if (server.hasArg("pitchD")) config.pitchD = server.arg("pitchD").toFloat();
  if (server.hasArg("yawP")) config.yawP = server.arg("yawP").toFloat();
  if (server.hasArg("yawI")) config.yawI = server.arg("yawI").toFloat();
  if (server.hasArg("yawD")) config.yawD = server.arg("yawD").toFloat();
  
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
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
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

float readBatteryVoltage() {
  int adcValue = analogRead(BATTERY_ADC_PIN);
  float voltageAtPin = (adcValue * ADC_REF) / ADC_MAX;
  float batteryVoltage = voltageAtPin * (R1 + R2) / R2 * 0.942;
  return batteryVoltage;
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];

  if (String(topic) == "esp32/waypoints") {
    Serial.println("Received Waypoints:");
    Serial.println(msg);

    // Parse waypoints: expected format [{"lat":27.7,"lon":85.3,"alt":100}, {...}]
    DynamicJsonDocument doc(1024);
    DeserializationError err = deserializeJson(doc, msg);
    if (!err) {
      waypointCount = min((int)doc.size(), MAX_WAYPOINTS);
      for (int i = 0; i < waypointCount; i++) {
        waypoints[i].lat = doc[i]["lat"];
        waypoints[i].lon = doc[i]["lon"];
        waypoints[i].alt = doc[i]["alt"];
      }
      Serial.println("Waypoints loaded.");
      missionFinished = false;
      currentWaypointIndex = 0;
    } else {
      Serial.println("Failed to parse waypoints JSON.");
    }
  }
}


void setup() {
  Serial.begin(115200);
  Wire.begin(6, 7);
  
  // Initialize EEPROM and load configuration
  EEPROM.begin(EEPROM_SIZE);
  loadEEPROM();
  
  // Initialize MPU6050
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (1);
  }
  
  // Initialize BMP280
  if (!bmp.begin(0x76)) {
    Serial.println("BMP280 connection failed");
    while (1);
  }
  
  // Attach servos and ESCs
  esc1.attach(ESC1_PIN, 1000, 2000);
  esc2.attach(ESC2_PIN, 1000, 2000);
  servoRight.attach(RIGHT_AILERON_SERVO_PIN, 1000, 2000);
  servoLeft.attach(LEFT_AILERON_SERVO_PIN, 1000, 2000);
  servoElevator.attach(ELEVATOR_SERVO_PIN, 1000, 2000);
  servoRudder.attach(RUDDER, 1000, 2000);

  // Initialize motors to minimum throttle
  esc1.writeMicroseconds(1000);
  esc2.writeMicroseconds(1000);

  // Setup PPM input
  pinMode(PPM_PIN, INPUT_PULLUP);
  attachInterrupt(PPM_PIN, ppmISR, RISING);

  // Initialize GPS
  GPS_Serial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);

  // Connect to WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  // while (WiFi.status() != WL_CONNECTED) {
  //   delay(500);
  //   Serial.print(".");
  // }
  Serial.println("\nConnected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // Setup MQTT
  mqttClient.setServer(mqtt_server, 1883);

  // Setup web server
  server.on("/", handleRoot);
  server.on("/update", handleUpdate);
  server.on("/reverse", handleReverse);
  server.on("/reverseServos", handleReverseServos);
  server.on("/calibrate", handleCalibrate);
  server.on("/gps", handleGPS);
  server.begin();

  mqttClient.setCallback(mqttCallback);
  mqttClient.subscribe("esp32/waypoints");


  // Calibrate IMU
  calibrateMPU();
}

void loop() {
  static unsigned long lastTime = millis();
  static unsigned long lastMqttPublish = millis();

  // Handle network connections
  checkWiFi();
  checkMQTT();
  mqttClient.loop();
  server.handleClient();

  // Process GPS data
  while (GPS_Serial.available()) {
    gps.encode(GPS_Serial.read());
    if (gps.location.isUpdated()) {
      gpsLatitude = String(gps.location.lat(), 6);
      gpsLongitude = String(gps.location.lng(), 6);
    }
  }

static unsigned long lastBatRead = 0;
static float batteryVoltage = 0;

if (millis() - lastBatRead > 1000) {
  batteryVoltage = readBatteryVoltage();
  // Serial.print("Battery Voltage: ");
  // Serial.println(batteryVoltage);
  lastBatRead = millis();
}


  // Read altitude
  currentAltitude = bmp.readAltitude(1013.25);
  
  // Check flight modes
  bool newAltHold = ch9Value >= 1800;
  if (newAltHold && !altitudeHoldEngaged) {
    targetAltitude = currentAltitude;
    altitudeHoldEngaged = true;
  } else if (!newAltHold) altitudeHoldEngaged = false;
  altitudeHoldEnabled = newAltHold;
  gyroStabilizationEnabled = (ch9Value >= 1400);
  differentialThrustEnabled = (ch7Value >= 1500);

  // Calculate time delta
  unsigned long now = millis();
  dt = (now - lastTime) / 1000.0;
  lastTime = now;

  // Read IMU data
  accelX = mpu.getAccelerationX();
  accelY = mpu.getAccelerationY();
  accelZ = -mpu.getAccelerationZ();
  gyroX = mpu.getRotationX() / 131.0;  // Convert to deg/s
  gyroY = mpu.getRotationY() / 131.0;
  gyroZ = mpu.getRotationZ() / 131.0;

  // Calculate attitude
  float accelRoll = atan2(-accelY, accelZ) * 180 / PI - rollOffset;
  float accelPitch = atan2(accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180 / PI - pitchOffset;
  rollFiltered = 0.98 * (rollFiltered + gyroX * dt) + 0.02 * accelRoll;
  pitchFiltered = 0.98 * (pitchFiltered + gyroY * dt) + 0.02 * accelPitch;

  // Initialize target positions from RC input
  int targetAileron = constrain(map(ch1Value, 1000, 2000, 0, 180), 0, 180);
  int targetElevator = constrain(map(ch2Value, 1000, 2000, 0, 180), 0, 180);
  int targetRudder = constrain(map(ch4Value, 1000, 2000, 0, 180), 0, 180);

  // Apply gyro stabilization if enabled
  if (gyroStabilizationEnabled) {
    // Calculate errors
    float rollError = -rollFiltered;  // Negative because we want to counteract the movement
    float pitchError = -pitchFiltered;
    float yawError = -gyroZ * dt;     // Yaw stabilization based on rate only

    // Roll PID
    rollErrorSum += rollError * dt;
    float rollDerivative = (rollError - lastRollError) / dt;
    float rollCorrection = config.rollP * rollError + config.rollI * rollErrorSum + config.rollD * rollDerivative;
    lastRollError = rollError;

    // Pitch PID
    pitchErrorSum += pitchError * dt;
    float pitchDerivative = (pitchError - lastPitchError) / dt;
    float pitchCorrection = config.pitchP * pitchError + config.pitchI * pitchErrorSum + config.pitchD * pitchDerivative;
    lastPitchError = pitchError;

    // Yaw PID
    yawErrorSum += yawError * dt;
    float yawDerivative = (yawError - lastYawError) / dt;
    float yawCorrection = config.yawP * yawError + config.yawI * yawErrorSum + config.yawD * yawDerivative;
    lastYawError = yawError;

    // Apply corrections
    targetAileron += rollCorrection;
    targetElevator += pitchCorrection;
    targetRudder += yawCorrection;

    // Constrain the outputs
    targetAileron = constrain(targetAileron, 0, 180);
    targetElevator = constrain(targetElevator, 0, 180);
    targetRudder = constrain(targetRudder, 0, 180);

    // Anti-windup - only integrate if not at limits
    if (targetAileron <= 0 || targetAileron >= 180) rollErrorSum = 0;
    if (targetElevator <= 0 || targetElevator >= 180) pitchErrorSum = 0;
    if (targetRudder <= 0 || targetRudder >= 180) yawErrorSum = 0;
  }

  // Apply altitude hold if enabled
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

  // Apply smoothing to control surfaces
  static float smoothAileron = 90, smoothElevator = 90;
  float alpha = 0.5;
  smoothAileron = alpha * targetAileron + (1 - alpha) * smoothAileron;
  smoothElevator = alpha * (targetElevator + altitudePitchCorrection) + (1 - alpha) * smoothElevator;
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
  
  // Handle rudder
  servoRudder.write(targetRudder);

  // Handle differential thrust
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

  // Publish telemetry
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
    telemetryJson += "\"throttleSignal\":" + String(throttleSignal)+",";
    telemetryJson += "\"batteryVoltage\":" + String(batteryVoltage);
    telemetryJson += "}";
    mqttClient.publish("esp32/telemetry", telemetryJson.c_str());
    lastMqttPublish = millis();
  }
  bool ch8MissionToggle = (ppmChannels[7] >= 1800);  // CH8 high = mission active

if (ch8MissionToggle && !missionStarted && waypointCount > 0) {
  missionStarted = true;
  missionFinished = false;
  Serial.println("Mission started.");
  if (gps.location.isValid()) {
    homeLat = gps.location.lat();
    homeLon = gps.location.lng();
    homeAlt = currentAltitude;
  }
}

if (!ch8MissionToggle) {
  missionStarted = false;
}

  if (missionStarted && !missionFinished) {
  Waypoint wp = waypoints[currentWaypointIndex];
  float targetLat = wp.lat;
  float targetLon = wp.lon;
  float targetAlt = wp.alt;

  float distanceToWP = TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), targetLat, targetLon);
  float bearingToWP = TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), targetLat, targetLon);
  
  float headingError = bearingToWP - gps.course.deg();
  if (headingError > 180) headingError -= 360;
  if (headingError < -180) headingError += 360;

  // Map heading error to rudder/yaw or differential thrust
  int yawCorrection = map(headingError, -180, 180, -30, 30);
  targetRudder = constrain(90 + yawCorrection, 60, 120);

  // Altitude hold
  targetAltitude = targetAlt;
  altitudeHoldEnabled = true;
  altitudeHoldEngaged = true;

  // If close enough to waypoint, move to next
  if (distanceToWP < 10) {
    currentWaypointIndex++;
    if (currentWaypointIndex >= waypointCount) {
      missionFinished = true;
      Serial.println("Mission complete, returning home.");
      // Optional: Set home as final waypoint
      waypoints[0] = { homeLat, homeLon, homeAlt };
      waypointCount = 1;
      currentWaypointIndex = 0;
    }
  }
}


  delay(20);
}



