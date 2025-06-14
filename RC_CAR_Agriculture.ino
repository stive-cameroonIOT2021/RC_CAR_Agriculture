#include <Wire.h>
#include <Adafruit_SHT31.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <TinyGPS++.h>
#include <ESP32Servo.h>
#include "BTS7960.h"

// ── UART2 (GPS) pinout ─────────────────────────────────
static const uint8_t RX_PIN = 16;      // GPIO16 ← GPS-TX
static const uint8_t TX_PIN = 17;      // GPIO17 → GPS-RX
static const uint32_t GPS_BAUD = 9600;

// Pin configuration
constexpr uint8_t SERVO_PIN = 18;
constexpr uint8_t EN_PIN = 19;
constexpr uint8_t L_PWM_PIN = 26;
constexpr uint8_t R_PWM_PIN = 27;
#define PH_PIN 35          // Analog pin for pH sensor (PO pin)
#define MOISTURE_PIN 34    // Analog pin for soil moisture sensor (AOUT)
#define LED_PIN 2          // GPIO2 for built-in LED

// Steering constants
constexpr int16_t MIN_STEERING = 0;
constexpr int16_t MAX_STEERING = 180;
constexpr int16_t STEP_SIZE = 2;
constexpr int16_t INITIAL_STEERING = 90;

// Motor speed constants
constexpr int8_t FORWARD_SPEED = 200;
constexpr int8_t BACKWARD_SPEED = -200;

#define ADC_RESOLUTION 4095.0 // ESP32 ADC resolution (12-bit)
#define VOLTAGE_REFERENCE 3.3 // ESP32 reference voltage (3.3V)
#define PH_OFFSET 0.12     // pH calibration offset
#define DRY_VALUE 3400     // Soil moisture sensor value in air (dry)
#define WET_VALUE 800      // Soil moisture sensor value in water (wet)
#define SAMPLING_INTERVAL 1000 // Time interval for readings (ms)
#define GPS_TIMEOUT 10000  // GPS fix timeout (ms)
#define UART_TIMEOUT 100   // UART read timeout (ms)

// ── Objects ────────────────────────────────────────────
Adafruit_SHT31 sht35;
Adafruit_BNO055 bno(55, 0x28); // ID=55, default addr 0x28
TinyGPSPlus gps;
HardwareSerial gpsSerial(2);   // UART2 on ESP32
Servo steeringServo;
BTS7960 motorController(EN_PIN, L_PWM_PIN, R_PWM_PIN);

// State variables
int16_t steeringAngle = INITIAL_STEERING;
int8_t motorSpeed = 0;

// ── Global variables ───────────────────────────────────
bool sht35Ok = false, bnoOk = false, gpsOk = false;
unsigned long lastGpsUpdate = 0; // Track last GPS update


// Function to log errors
void logError(String message) {
  Serial.println("[ERROR at " + String(millis()) + "ms] " + message);
}

// Function to read pH value
float readPH(int rawValue) {
  if (rawValue < 0 || rawValue > ADC_RESOLUTION) {
    logError("Invalid pH ADC value: " + String(rawValue));
    return -1.0;
  }
  float voltage = rawValue * (VOLTAGE_REFERENCE / ADC_RESOLUTION);
  float pHValue = 7.0 - ((voltage - 2.5) / (-0.1786)) + PH_OFFSET;
  return isnan(pHValue) || isinf(pHValue) ? -1.0 : pHValue;
}

// Function to read soil moisture percentage
float readSoilMoisture(int rawValue) {
  if (rawValue < 0 || rawValue > ADC_RESOLUTION) {
    logError("Invalid soil moisture ADC value: " + String(rawValue));
    return -1.0;
  }
  float moisturePercent = (rawValue*100) / 4095.0;
  return constrain(moisturePercent, 0, 100);
}

// Function to read and send sensor values
void readSensors(bool sendOverUART) {
  String output = "=== Sensor Readings ===\n";

  // pH
  float pHValue = readPH(analogRead(PH_PIN));
  output += pHValue >= 0 ? String("pH: ") + String(pHValue, 2) + "\n" : "pH: Error\n";

  // Soil Moisture
  float moisturePercent = readSoilMoisture(analogRead(MOISTURE_PIN));
  output += moisturePercent >= 0 ? String("Soil Moisture: ") + String(moisturePercent, 1) + "%\n" : "Soil Moisture: Error\n";

  // SHT31 (Temperature and Humidity)
  float temperature = sht35Ok ? sht35.readTemperature() : NAN;
  float humidity = sht35Ok ? sht35.readHumidity() : NAN;
  if (!isnan(temperature)) {
    output += String("Temperature: ") + String(temperature, 1) + "°C\n";
  } else {
    output += "Temperature: N/A\n";
    if (sht35Ok) logError("Invalid SHT31 temperature reading");
  }
  if (!isnan(humidity)) {
    output += String("Humidity: ") + String(humidity, 1) + "%\n";
  } else {
    output += "Humidity: N/A\n";
    if (sht35Ok) logError("Invalid SHT31 humidity reading");
  }

  // BNO055 (Orientation and Accelerometer)
  if (bnoOk) {
    sensors_event_t orientEvent, accelEvent;
    if (bno.getEvent(&orientEvent, Adafruit_BNO055::VECTOR_EULER) && 
        bno.getEvent(&accelEvent, Adafruit_BNO055::VECTOR_ACCELEROMETER)) {
      float roll = orientEvent.orientation.x;
      float pitch = orientEvent.orientation.y;
      float yaw = orientEvent.orientation.z;
      float accelX = accelEvent.acceleration.x;
      float accelY = accelEvent.acceleration.y;
      float accelZ = accelEvent.acceleration.z;
      if (!isnan(roll) && !isnan(pitch) && !isnan(yaw)) {
        output += String("Orientation (R/P/Y): ") + String(roll, 1) + "° / " + 
                  String(pitch, 1) + "° / " + String(yaw, 1) + "°\n";
      } else {
        output += "Orientation: Error\n";
        logError("Invalid BNO055 orientation data");
      }
      if (!isnan(accelX) && !isnan(accelY) && !isnan(accelZ)) {
        output += String("Acceleration (X/Y/Z): ") + String(accelX, 1) + " / " + 
                  String(accelY, 1) + " / " + String(accelZ, 1) + " m/s²\n";
      } else {
        output += "Acceleration: Error\n";
        logError("Invalid BNO055 acceleration data");
      }
    } else {
      output += "Orientation: Error\nAcceleration: Error\n";
      logError("Failed to read BNO055 events");
    }
  } else {
    output += "Orientation: N/A\nAcceleration: N/A\n";
  }

  // GPS
  bool gpsValid = gpsOk && gps.location.isValid() && gps.satellites.isValid() && 
                  gps.speed.isValid() && (millis() - lastGpsUpdate < GPS_TIMEOUT);
  if (gpsValid) {
    float latitude = gps.location.lat();
    float longitude = gps.location.lng();
    float speedKmph = gps.speed.kmph();
    uint32_t satellites = gps.satellites.value();
    output += String("GPS: Lat=") + String(latitude, 6) + ", Lon=" + 
              String(longitude, 6) + ", Speed=" + String(speedKmph, 1) + 
              " km/h, Sat=" + String(satellites) + "\n";
  } else {
    output += "GPS: No valid fix\n";
    if (gpsOk && !gpsValid) logError("GPS data invalid or timed out");
  }

  output += "======================\n";
  //Serial.println(output);

  if (sendOverUART) {
    Serial.println(output);
  }
}

// Motor control function
inline void setMotorSpeed(int8_t speed) {
  if (motorSpeed == speed) return;  // Avoid redundant motor updates
  motorSpeed = speed;
  if (speed > 0) {
    motorController.TurnRight(speed);
  } else if (speed < 0) {
    motorController.TurnLeft(-speed);
  } else {
    motorController.Stop();
  }
}

// Function to handle UART commands
void handleUARTCommands() {
  static String commandBuffer = "";
  unsigned long startTime = millis();

  while (Serial.available() && millis() - startTime < UART_TIMEOUT) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') continue; // Ignore newlines
    commandBuffer += c;
    if (commandBuffer.length() >= 1) { // Process single-character commands
      char command = commandBuffer[0];
      commandBuffer = ""; // Clear buffer
      switch (command) {
        case 'K': // Send sensor data
          readSensors(true);
          break;
        case 'S': // Stop
            setMotorSpeed(0);
          break;
        case 'L': // Turn left
            steeringAngle -= STEP_SIZE;
            if (steeringAngle < MIN_STEERING) steeringAngle = MIN_STEERING;
          break;
        case 'R': // Turn right
            steeringAngle += STEP_SIZE;
            if (steeringAngle > MAX_STEERING) steeringAngle = MAX_STEERING;
          break;
        case 'F': // Move forward
            setMotorSpeed(FORWARD_SPEED);
          break;
        case 'B': // Move backward
            setMotorSpeed(BACKWARD_SPEED);
          break;
        default:
          logError("Unknown command: " + String(command));
          break;
      }
    }
  }
}

void setup() {
  Serial.begin(115200);  // Kept original baud rate

  Serial.setTimeout(20); // Fast UART response
  while (!Serial && millis() < 5000); // Wait up to 5s for Serial
  if (!Serial) logError("Serial initialization failed");
  Serial.println("Booting…");

  // Initialize pins
  pinMode(PH_PIN, INPUT);
  pinMode(MOISTURE_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW); // LED off initially

  // I2C sensors
  Wire.begin(21, 22); // SDA, SCL
  sht35Ok = sht35.begin(0x44);
  if (!sht35Ok) {
    logError("SHT35 initialization failed");
  } else {
    Serial.println("✅ SHT35 ready");
  }
  bnoOk = bno.begin();
  if (!bnoOk) {
    logError("BNO055 initialization failed");
  } else {
    bno.setExtCrystalUse(true);
    Serial.println("✅ BNO055 ready");
  }

  // GPS
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, RX_PIN, TX_PIN);
  gpsOk = gpsSerial; // Check if UART2 initialized
  if (!gpsOk) {
    logError("GPS UART2 initialization failed");
  } else {
    Serial.println("✅ GPS UART2 @ 9600");
  }


  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  steeringServo.setPeriodHertz(50);
  steeringServo.attach(SERVO_PIN, 500, 2400);
  steeringServo.write(INITIAL_STEERING);
  Serial.println(F("Send 'L'/'R' to steer, 'F'/'B'/'S' for motor control"));

  motorController.Enable();
  motorController.Stop();

  digitalWrite(LED_PIN, HIGH); // LED ony
}

void loop() {
  // Feed GPS parser
  if (gpsOk) {
    while (gpsSerial.available()) {
      if (gps.encode(gpsSerial.read())) {
        lastGpsUpdate = millis(); // Update timestamp on valid data
      }
    }
  }

  // Handle UART commands
  handleUARTCommands();
  steeringServo.write(steeringAngle);

  // Read and display sensors at intervals (local debugging)
  static unsigned long lastTime = 0;
  if (millis() - lastTime >= SAMPLING_INTERVAL) {
    readSensors(false); // Local display
    lastTime = millis();
  }
/*   if (!Serial.available()) return;  // Early return if no input

  int16_t prevSteeringAngle = steeringAngle;  // Track previous angle
  switch (Serial.read()) {
    case 'L':
      steeringAngle -= STEP_SIZE;
      if (steeringAngle < MIN_STEERING) steeringAngle = MIN_STEERING;
      break;
    case 'R':
      steeringAngle += STEP_SIZE;
      if (steeringAngle > MAX_STEERING) steeringAngle = MAX_STEERING;
      break;
    case 'F':
      setMotorSpeed(FORWARD_SPEED);
      Serial.println(F("Motor: Forward"));
      break;
    case 'B':
      setMotorSpeed(BACKWARD_SPEED);
      Serial.println(F("Motor: Backward"));
      break;
    case 'S':
      setMotorSpeed(0);
      Serial.println(F("Motor: Stopped"));
      break;
    default:
      return;  // Ignore invalid commands
  }

  // Update servo only if angle changed
  if (steeringAngle != prevSteeringAngle) {
    steeringServo.write(steeringAngle);
    Serial.print(F("Steering Angle: "));
    Serial.println(steeringAngle);
  } */
}