#include <Wire.h>

// CMPS12 Commands
#define CMPS_GET_ANGLE8 0x12
#define CMPS_GET_ANGLE16 0x13
#define CMPS_GET_PITCH 0x14
#define CMPS_GET_ROLL 0x15
#define CMPS_CALIBRATION_STATUS 0x24

// Wind Sensor Pins
#define WIND_SPEED_PIN 2       // Digital pin for Davis anemometer pulses
#define WIND_DIRECTION_PIN A15  // Analog pin for Davis wind vane

// Compass Constants
#define COMPASS_TIMEOUT 5000  // milliseconds
#define MIN_CALIBRATION_LEVEL 2  // Require at least "Mostly Calibrated"
#define COMPASS_READ_INTERVAL 100  // milliseconds between readings

// Wind Constants
#define WIND_SPEED_INTERVAL 5000  // 5 seconds between wind speed calculations
#define WIND_DEBOUNCE_TIME 100    // 100ms debounce for anemometer
#define WIND_SPEED_CALIBRATION 2.4 // Calibration factor for wind speed (mph per pulse)

// Timing
const unsigned long LOG_INTERVAL = 1000; // Log every second
unsigned long lastLogTime = 0;
unsigned long lastValidCompassTime = 0;
unsigned long lastCompassReadTime = 0;
unsigned long lastWindSpeedTime = 0;

// Data storage
struct CMPS12Data {
    float heading;      // 0-360 degrees
    float pitch;        // -90 to +90 degrees
    float roll;         // -90 to +90 degrees
    uint8_t calibration; // 0-3 (3 is fully calibrated)
    bool valid;         // Data validity flag
    bool isCalibrated;  // Whether compass meets minimum calibration level
} cmpsData;

// Wind data
struct WindData {
    float speed;        // Wind speed in mph
    float direction;    // Wind direction in degrees (0-360)
    volatile unsigned int pulseCount; // Anemometer pulse count
    unsigned long lastPulseTime;     // Last pulse timestamp
    bool valid;         // Data validity flag
} windData;

void setup() {
    // Initialize Serial for logging
    Serial.begin(115200);
    while (!Serial); // Wait for Serial to be ready
    
    // Initialize CMPS12 on Serial3
    Serial3.begin(9600);
    
    // Initialize compass data
    cmpsData.heading = 0.0;
    cmpsData.pitch = 0.0;
    cmpsData.roll = 0.0;
    cmpsData.calibration = 0;
    cmpsData.valid = false;
    cmpsData.isCalibrated = false;
    
    // Initialize wind data
    windData.speed = 0.0;
    windData.direction = 0.0;
    windData.pulseCount = 0;
    windData.lastPulseTime = 0;
    windData.valid = false;
    
    // Setup wind speed pin with interrupt
    pinMode(WIND_SPEED_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(WIND_SPEED_PIN), windSpeedISR, FALLING);
    
    // Print header
    Serial.println("\nAutonomous Sailboat - Sensor System");
    Serial.println("Time(ms) | Heading(°) | Wind Dir(°) | Wind Spd(mph) | Status");
    Serial.println("------------------------------------------------------------------");
}

void loop() {
    // Read compass data at appropriate interval
    if (millis() - lastCompassReadTime >= COMPASS_READ_INTERVAL) {
        readCMPS12Data();
        lastCompassReadTime = millis();
    }
    
    // Read wind direction
    readWindDirection();
    
    // Calculate wind speed periodically
    if (millis() - lastWindSpeedTime >= WIND_SPEED_INTERVAL) {
        calculateWindSpeed();
        lastWindSpeedTime = millis();
    }
    
    // Log data at specified interval
    if (millis() - lastLogTime >= LOG_INTERVAL) {
        logData();
        lastLogTime = millis();
    }
    
    delay(10); // Small delay to prevent overwhelming the system
}

void readCMPS12Data() {
    cmpsData.valid = true; // Assume data is valid until proven otherwise
    
    // Read 16-bit heading
    Serial3.write(CMPS_GET_ANGLE16);
    while (Serial3.available() < 2); // Wait for data
    uint8_t high_byte = Serial3.read();
    uint8_t low_byte = Serial3.read();
    uint16_t angle16 = (high_byte << 8) | low_byte;
    cmpsData.heading = angle16 / 10.0; // Convert to degrees
    
    // Read pitch
    Serial3.write(CMPS_GET_PITCH);
    while (Serial3.available() < 1); // Wait for data
    cmpsData.pitch = (int8_t)Serial3.read(); // Convert to degrees
    
    // Read roll
    Serial3.write(CMPS_GET_ROLL);
    while (Serial3.available() < 1); // Wait for data
    cmpsData.roll = (int8_t)Serial3.read(); // Convert to degrees
    
    // Read calibration status
    Serial3.write(CMPS_CALIBRATION_STATUS);
    while (Serial3.available() < 1); // Wait for data
    cmpsData.calibration = Serial3.read();
    
    // Update compass validity and calibration status
    if (cmpsData.valid && cmpsData.calibration >= MIN_CALIBRATION_LEVEL) {
        lastValidCompassTime = millis();
        cmpsData.isCalibrated = true;
    } else {
        cmpsData.isCalibrated = false;
    }
    
    // Check for compass timeout
    if (millis() - lastValidCompassTime > COMPASS_TIMEOUT) {
        cmpsData.valid = false;
    }
}

void readWindDirection() {
    int analogValue = analogRead(WIND_DIRECTION_PIN);
    windData.direction = map(analogValue, 0, 1023, 0, 360);
    windData.valid = true;
}

void calculateWindSpeed() {
    float intervalSeconds = WIND_SPEED_INTERVAL / 1000.0;
    float pulsesPerSecond = windData.pulseCount / intervalSeconds;
    windData.speed = pulsesPerSecond * WIND_SPEED_CALIBRATION;
    windData.pulseCount = 0;
}

// Wind speed interrupt service routine with debouncing
void windSpeedISR() {
    unsigned long currentTime = millis();
    if (currentTime - windData.lastPulseTime > WIND_DEBOUNCE_TIME) {
        windData.pulseCount++;
        windData.lastPulseTime = currentTime;
    }
}

void logData() {
    // Print timestamp
    Serial.print(millis());
    Serial.print(" | ");
    
    // Print current heading
    Serial.print(cmpsData.heading, 1);
    Serial.print("° | ");
    
    // Print wind direction
    Serial.print(windData.direction, 1);
    Serial.print("° | ");
    
    // Print wind speed
    Serial.print(windData.speed, 1);
    Serial.print(" mph | ");
    
    // Print system status
    if (!cmpsData.valid) {
        Serial.println("COMPASS ERROR");
    } else if (!cmpsData.isCalibrated) {
        Serial.println("NEEDS CALIBRATION");
    } else if (!windData.valid) {
        Serial.println("WIND SENSOR ERROR");
    } else {
        Serial.println("OK");
    }
} 