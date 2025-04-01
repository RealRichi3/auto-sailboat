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

#define WIND_SPEED_PIN 2       // Digital pin for Davis anemometer pulses (interrupt-capable)
#define WIND_DIRECTION_PIN A15  // Analog pin for Davis wind vane

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

// Wind variables
volatile unsigned int windPulseCount = 0;
unsigned long windSpeedLastMillis = 0;
const unsigned long WIND_SPEED_INTERVAL = 5000;  // 5 seconds
float windSpeedMph = 0.0;
unsigned long lastWindPulseTime = 0;
const unsigned long DEBOUNCE_TIME = 100;  // 100ms debounce

// Servo pulse limits
const uint16_t SERVOMIN = 150; // Minimum pulse length count
const uint16_t SERVOMAX = 600; // Maximum pulse length count

// CMPS12 Variables
unsigned char high_byte, low_byte, angle8;
char pitch, roll;
unsigned int angle16;

// RC Control Pins
#define PIN_RUDDER    4  // PWM pin for rudder control
#define PIN_SAIL      5  // PWM pin for sail control

// Servo control variables
int rudderAngle = 90;  // Neutral position
int sailAngle = 90;    // Neutral position
int lastRudderAngle = 90;
int lastSailAngle = 90;

// System state
bool rcOverrideEnabled = false;
bool gpsValid = false;
bool compassValid = false;
unsigned long lastValidGPS = 0;
const unsigned long GPS_TIMEOUT = 10000; // 10 seconds

// ----------------------
// FUNCTION PROTOTYPES
// ----------------------
int rudderPWM = 0;      // Stores RC rudder PWM value
int sailPWM = 0;        // Stores RC sail PWM value
float readCMPS12();
int readWindDirection();
void updateServos();
void calculateWindSpeed();
void windSpeedISR();
uint16_t angleToPulse(int angle);
void updateCourse(float distanceToLine, float windDirection, float heading, float &courseAdjusted);
float calculateDistanceToLine(float lat_ref1, float lon_ref1, float lat_ref2, float lon_ref2, float lat_m, float lon_m);
void setServoAngles(int sail, int rudder);
void checkRCOverride();
void handleGPSLoss();
void systemWatchdog();

// ----------------------
// SETUP FUNCTION
// ----------------------
void setup() {
  Serial.begin(9600);
  Wire.begin();

  // Initialize PWM servo driver
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(120);
  
  // Initialize GPS
  gpsSerial.begin(9600);

  // Initialize CMPS12
  Serial3.begin(9600);

  // Wind sensor setup
  pinMode(WIND_SPEED_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(WIND_SPEED_PIN), windSpeedISR, FALLING);

  // Initialize timers
  windSpeedLastMillis = millis();
  lastValidGPS = millis();

  Serial.println("Sailboat Navigation System Initialized");
}

// ----------------------
// MAIN LOOP FUNCTION
// ----------------------
void loop() {
  systemWatchdog(); // Check for system hangs

  // 1. Check RC override first
  checkRCOverride();
  
  if (rcOverrideEnabled) {
    // Skip autonomous calculations if in RC mode
    setServoAngles(sailAngle, rudderAngle);
    delay(100);
    return;
  }

  // 2. Read sensors
  float heading = readCMPS12();
  int windDirection = readWindDirection();
  
  // 3. Process GPS data
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      lastValidGPS = millis();
      gpsValid = gps.location.isValid();
    }
  }

  // Check GPS timeout
  if (millis() - lastValidGPS > GPS_TIMEOUT) {
    handleGPSLoss();
  }

  // 4. Calculate wind speed periodically
  if (millis() - windSpeedLastMillis >= WIND_SPEED_INTERVAL) {
    calculateWindSpeed();
    windSpeedLastMillis = millis();
  }

  // 5. Autonomous navigation calculations
  if (gpsValid) {
    float distanceToLine = calculateDistanceToLine(lat_ref1, lon_ref1, lat_ref2, lon_ref2, 
                                                 gps.location.lat(), gps.location.lng());
    
    float courseAdjusted;
    updateCourse(distanceToLine, windDirection, heading, courseAdjusted);
  }

  // 6. Update servos with rate limiting
  setServoAngles(sailAngle, rudderAngle);

  // 7. Debug output
  Serial.print("Heading: ");
  Serial.print(heading);
  Serial.print("°, Wind: ");
  Serial.print(windDirection);
  Serial.print("°, Rudder: ");
  Serial.print(rudderAngle);
  Serial.print("°, Sail: ");
  Serial.print(sailAngle);
  Serial.print("°, GPS: ");
  Serial.println(gpsValid ? "Valid" : "Invalid");

  delay(100); // Main loop delay
}

// ----------------------
// FUNCTION DEFINITIONS
// ----------------------

// Read compass heading from CMPS12
float readCMPS12() {
  Serial3.write(CMPS_GET_ANGLE16);
  while (Serial3.available() < 2); // Wait for data
  high_byte = Serial3.read();
  low_byte = Serial3.read();
  angle16 = (high_byte << 8) | low_byte;
  compassValid = (angle16 != 0xFFFF); // Simple validity check
  return angle16 / 10.0;
}

// Read wind direction from analog vane
int readWindDirection() {
  int analogValue = analogRead(WIND_DIRECTION_PIN);
  return map(analogValue, 0, 1023, 0, 360);
}

// Calculate wind speed from pulse count
void calculateWindSpeed() {
  float intervalSeconds = WIND_SPEED_INTERVAL / 1000.0;
  float pulsesPerSecond = windPulseCount / intervalSeconds;
  windSpeedMph = pulsesPerSecond * 2.4; // Adjust calibration factor as needed
  windPulseCount = 0;
}

// Wind speed interrupt service routine with debouncing
void windSpeedISR() {
  unsigned long currentTime = millis();
  if (currentTime - lastWindPulseTime > DEBOUNCE_TIME) {
    windPulseCount++;
    lastWindPulseTime = currentTime;
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
  
  // Update servos
  pwm.setPWM(SERVO1_CHANNEL, 0, angleToPulse(sail));
  pwm.setPWM(SERVO2_CHANNEL, 0, angleToPulse(rudder));
  
  // Store last positions
  lastSailAngle = sail;
  lastRudderAngle = rudder;
}

// Calculate distance to line (in meters)
float calculateDistanceToLine(float lat_ref1, float lon_ref1, float lat_ref2, float lon_ref2, float lat_m, float lon_m) {
  // Convert to radians
  float lat1 = lat_ref1 * DEG_TO_RAD;
  float lon1 = lon_ref1 * DEG_TO_RAD;
  float lat2 = lat_ref2 * DEG_TO_RAD;
  float lon2 = lon_ref2 * DEG_TO_RAD;
  float lat = lat_m * DEG_TO_RAD;
  float lon = lon_m * DEG_TO_RAD;
  
  // Earth radius in meters
  const float R = 6371000.0;
  
  // Calculate cross track distance
  float d13 = acos(sin(lat1) * sin(lat) + cos(lat1) * cos(lat) * cos(lon - lon1)) * R;
  float theta13 = atan2(sin(lon - lon1) * cos(lat), 
                   cos(lat1) * sin(lat) - sin(lat1) * cos(lat) * cos(lon - lon1));
  float theta12 = atan2(sin(lon2 - lon1) * cos(lat2), 
                   cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(lon2 - lon1));
  
  float alpha = theta13 - theta12;
  float xtrack = asin(sin(d13 / R) * sin(alpha)) * R;
  
  return xtrack;
}

// Update course based on sensors
void updateCourse(float distanceToLine, float windDirection, float heading, float &courseAdjusted) {
  // Calculate line direction in radians
  float lineDirection = atan2(
    (lat_ref2 - lat_ref1) * DEG_TO_RAD,
    (lon_ref2 - lon_ref1) * cos((lat_ref1 + lat_ref2)/2 * DEG_TO_RAD) * DEG_TO_RAD
  );
  
  // Course adjustment based on distance to line
  courseAdjusted = lineDirection - (2 * M_PI / 180) * atan(distanceToLine / 25.0);
  
  // Convert to degrees
  courseAdjusted = courseAdjusted * RAD_TO_DEG;
  
  // Calculate heading error
  float headingError = heading - courseAdjusted;
  
  // Adjust rudder angle based on heading error
  rudderAngle = 90 + headingError * 2.0; // Proportional gain of 2.0
  rudderAngle = constrain(rudderAngle, 45, 135); // Limit rudder movement
  
  // Adjust sail angle based on wind direction
  float apparentWind = windDirection - heading;
  sailAngle = map(apparentWind, -180, 180, 45, 135);
  sailAngle = constrain(sailAngle, 45, 135);
}

// Check for RC override
void checkRCOverride() {
  rudderPWM = pulseIn(PIN_RUDDER, HIGH, 25000); // 25ms timeout
  sailPWM = pulseIn(PIN_SAIL, HIGH, 25000);
  
  rcOverrideEnabled = (rudderPWM >= 1000 && rudderPWM <= 2000 && 
                      sailPWM >= 1000 && sailPWM <= 2000);
  
  if (rcOverrideEnabled) {
    // Map RC inputs to servo angles
    rudderAngle = map(rudderPWM, 1000, 2000, 45, 135);
    sailAngle = map(sailPWM, 1000, 2000, 45, 135);
  }
}

// Handle GPS signal loss
void handleGPSLoss() {
  gpsValid = false;
  // Reduce sail and center rudder for safety
  sailAngle = 90;
  rudderAngle = 90;
  Serial.println("GPS signal lost - entering safety mode");
}

// Simple watchdog timer
void systemWatchdog() {
  static unsigned long lastLoopTime = millis();
  if (millis() - lastLoopTime > 1000) { // If loop takes >1 second
    Serial.println("System hang detected - resetting servos to neutral");
    setServoAngles(90, 90);
    // Consider software reset here if needed
  }
  lastLoopTime = millis();
}
