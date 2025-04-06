#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <math.h>

// ----------------------
// PIN & HARDWARE DEFINITIONS
// ----------------------
#define GPS_RX_PIN 17           // GPS module TX (Arduino receives here)
#define GPS_TX_PIN 16           // GPS module RX (Arduino sends here)

// Define the two points (lat_ref1, lon_ref1) and (lat_ref2, lon_ref2)
float lat_ref1 = 37.7749;   // Point 1's latitude (San Francisco)
float lon_ref1 = -122.4194; // Point 1's longitude
float lat_ref2 = 37.8044;   // Point 2's latitude (Oakland)
float lon_ref2 = -122.2711; // Point 2's longitude

// CMPS12 Commands
#define CMPS_GET_ANGLE8 0x12
#define CMPS_GET_ANGLE16 0x13
#define CMPS_GET_PITCH 0x14
#define CMPS_GET_ROLL 0x15

// ----------------------
// GLOBAL OBJECTS & VARIABLES
// ----------------------
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Servo channels
#define SERVO1_CHANNEL 0       // Sail servo
#define SERVO2_CHANNEL 1       // Rudder servo

// GPS objects
SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN);
TinyGPSPlus gps;

// GPS position variables
float current_lat = 0.0;
float current_lon = 0.0;

// Servo pulse limits
const uint16_t SERVOMIN = 150; // Minimum pulse length count
const uint16_t SERVOMAX = 600; // Maximum pulse length count

// CMPS12 Variables
unsigned char high_byte, low_byte, angle8;
char pitch, roll;
unsigned int angle16;

// Servo control variables
int rudderAngle = 90;  // Neutral position
int sailAngle = 90;    // Neutral position
int lastRudderAngle = 90;
int lastSailAngle = 90;

// System state
bool gpsValid = false;
bool compassValid = false;
unsigned long lastValidGPS = 0;
const unsigned long GPS_TIMEOUT = 10000; // 10 seconds

// Simulated wind direction (0 = coming from North, 90 = from East, etc.)
const float SIMULATED_WIND_DIRECTION = 45.0; // 45 degrees = NE wind
const float SIMULATED_WIND_SPEED = 10.0;     // 10 knots wind speed

// Current heading and control
float current_heading = 0.0;

// Control parameters
const float RUDDER_P_GAIN = 0.5;    // Proportional gain for rudder control
const float RUDDER_D_GAIN = 0.1;    // Derivative gain for rudder control
float prev_heading_error = 0.0;     // For derivative control

// Logging interval control
unsigned long lastLogTime = 0;
const unsigned long LOG_INTERVAL = 1000; // Log every 1 second

void setup() {
  Serial.begin(9600);     // Start serial communication for debugging
  Wire.begin();           // Initialize I2C communication

  // Initialize PWM servo driver
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(60);  // Match the frequency from working code
  
  // Initialize GPS
  gpsSerial.begin(9600);

  // Initialize CMPS12
  Serial3.begin(9600);

  Serial.println("Sailboat Navigation System Initialized");
}

void loop() {
  // 1. Update GPS data
  updateGPS();
  
  // 2. Read compass heading from CMPS12
  float heading = readCMPS12();
  
  // Print compass data immediately for debugging
  Serial.print("Raw Compass Heading: ");
  Serial.print(heading);
  Serial.println(" degrees");
  
  // Calculate desired heading to target if GPS is valid
  float target_heading = 0.0;
  if (gpsValid) {
    target_heading = calculateTargetHeading();
  }
  
  // Calculate heading error
  float heading_error = normalizeAngle(target_heading - heading);
  
  // Calculate optimal sail angle based on wind
  float optimal_sail = calculateSailAngle(SIMULATED_WIND_DIRECTION);
  
  // Calculate rudder angle using PD control
  float rudder_angle = calculateRudderAngle(heading_error);
  
  // Apply controls to servos with explicit mapping
  setServoAngles(optimal_sail, rudder_angle);
  
  // Print sensor data and debug information at regular intervals
  if (millis() - lastLogTime >= LOG_INTERVAL) {
    printSensorData();
    printNavigationData(target_heading, heading_error, optimal_sail, rudder_angle);
    printActuatorData();
    lastLogTime = millis();
  }
  
  delay(100); // Update 10 times per second
}

float readCMPS12() {
  Serial3.write(CMPS_GET_ANGLE16);  // Request 16-bit angle
  while (Serial3.available() < 2);   // Wait for 2 bytes
  high_byte = Serial3.read();
  low_byte = Serial3.read();
  angle16 = (high_byte << 8) | low_byte;  // Calculate 16-bit angle
  compassValid = (angle16 != 0xFFFF);      // Simple validity check

  return angle16 / 10.0;  // Return heading in degrees
}

void updateGPS() {
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      if (gps.location.isValid()) {
        current_lat = gps.location.lat();
        current_lon = gps.location.lng();
        gpsValid = true;
        lastValidGPS = millis();
      }
    }
  }
  
  // Check GPS timeout
  if (millis() - lastValidGPS > GPS_TIMEOUT) {
    gpsValid = false;
    // Serial.println("[ERROR] GPS timeout");
  }
}

// Convert angle to PWM pulse
uint16_t angleToPulse(int angle) {
  return map(constrain(angle, 0, 180), 0, 180, SERVOMIN, SERVOMAX);
}

// Set servo angles with rate limiting
void setServoAngles(int sail, int rudder) {
  // Rate limit (max 10° change per cycle)
  sail = constrain(sail, lastSailAngle - 10, lastSailAngle + 10);
  rudder = constrain(rudder, lastRudderAngle - 10, lastRudderAngle + 10);
  
  // Update servos using PWM driver
  pwm.setPWM(SERVO1_CHANNEL, 0, angleToPulse(sail));
  pwm.setPWM(SERVO2_CHANNEL, 0, angleToPulse(rudder));
  
  // Store last positions
  lastSailAngle = sail;
  lastRudderAngle = rudder;
}

float calculateTargetHeading() {
  // Calculate bearing between current position and target
  float delta_lon = radians(lon_ref2 - current_lon);
  float lat1 = radians(current_lat);
  float lat2 = radians(lat_ref2);
  
  float y = sin(delta_lon) * cos(lat2);
  float x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(delta_lon);
  float bearing = degrees(atan2(y, x));
  
  return normalizeAngle(bearing);
}

float calculateSailAngle(float wind_direction) {
  // Calculate relative wind angle
  float relative_wind = normalizeAngle(wind_direction - current_heading);
  float sail_angle;
  
  // Implement basic sail angle rules
  if (relative_wind <= 45) {
    // Close hauled
    sail_angle = 45;
  } else if (relative_wind <= 90) {
    // Close reach
    sail_angle = relative_wind * 0.8;
  } else if (relative_wind <= 180) {
    // Beam reach to broad reach
    sail_angle = 90 - (relative_wind - 90) * 0.3;
  } else {
    // Running
    sail_angle = relative_wind > 270 ? 45 : 90;
  }
  
  return constrain(sail_angle, 0, 90);
}

float calculateRudderAngle(float heading_error) {
  // PD control for rudder
  float derivative = (heading_error - prev_heading_error) / 0.1; // 0.1s is our loop time
  float rudder_angle = -(RUDDER_P_GAIN * heading_error + RUDDER_D_GAIN * derivative);
  return constrain(rudder_angle, -60, 60);
}

float normalizeAngle(float angle) {
  while (angle < 0) angle += 360;
  while (angle >= 360) angle -= 360;
  return angle;
}

void printSensorData() {
  Serial.println("\n[SENSOR DATA]");
  
  // GPS Data
  Serial.print("GPS: ");
  Serial.print(gpsValid ? "VALID" : "NO FIX");
  if (gpsValid) {
    Serial.print(" | Pos: "); Serial.print(current_lat, 6); 
    Serial.print(", "); Serial.print(current_lon, 6);
    Serial.print(" | Sats: "); Serial.print(gps.satellites.value());
    Serial.print(" | HDOP: "); Serial.print(gps.hdop.value());
    Serial.print(" | Speed: "); Serial.print(gps.speed.knots());
    Serial.print(" knots | Course: "); Serial.print(gps.course.deg());
    Serial.println("°");
  } else {
    Serial.println();
  }
  
  // Compass Data
  Serial.print("Compass: ");
  Serial.print(compassValid ? "VALID" : "ERROR");
  // Serial.print(" | Raw: 0x"); Serial.print(angle16, HEX);
  // Serial.print(" (H:0x"); Serial.print(high_byte, HEX);
  // Serial.print(" L:0x"); Serial.print(low_byte, HEX);
  Serial.print(") | Heading: "); Serial.print(angle16 / 10.0);
  Serial.println("°");
  
  // // Wind Data (Simulated)
  // Serial.print("Wind: ");
  // Serial.print(SIMULATED_WIND_DIRECTION);
  // Serial.print("° | Speed: "); Serial.print(SIMULATED_WIND_SPEED);
  // Serial.println(" knots");
}

void printNavigationData(float target_heading, float heading_error, float optimal_sail, float rudder_angle) {
  Serial.println("\n[NAVIGATION DATA]");
  Serial.print("Target: "); Serial.print(target_heading);
  Serial.print("° | Current: "); Serial.print(current_heading);
  Serial.print("° | Error: "); Serial.print(heading_error);
  Serial.print("° | Rudder: "); Serial.print(rudder_angle);
  Serial.print("° | Sail: "); Serial.print(optimal_sail);
  Serial.println("°");
  
  // Calculate distances
  float distance_to_target = TinyGPSPlus::distanceBetween(
    current_lat, current_lon,
    lat_ref2, lon_ref2);
    
  float distance_to_line = TinyGPSPlus::distanceBetween(
    current_lat, current_lon,
    lat_ref1, lon_ref1);
  
  Serial.print("Distance to line: "); Serial.print(distance_to_line);
  Serial.print("m | Distance to target: "); Serial.print(distance_to_target);
  Serial.println("m");
}

void printActuatorData() {
  Serial.println("\n[ACTUATOR DATA]");
  Serial.print("Rudder: "); Serial.print(rudderAngle);
  Serial.print("° | Sail: "); Serial.print(sailAngle);
  Serial.println("°");
} 
