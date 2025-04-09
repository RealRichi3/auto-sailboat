#include <Wire.h>

// CMPS12 Commands
#define CMPS_GET_ANGLE8 0x12
#define CMPS_GET_ANGLE16 0x13
#define CMPS_GET_PITCH 0x14
#define CMPS_GET_ROLL 0x15
#define CMPS_CALIBRATION_STATUS 0x24

// Compass Constants
#define COMPASS_TIMEOUT 5000  // milliseconds
#define MIN_CALIBRATION_LEVEL 2  // Require at least "Mostly Calibrated"
#define COMPASS_READ_INTERVAL 100  // milliseconds between readings

// Timing
const unsigned long LOG_INTERVAL = 1000; // Log every second
unsigned long lastLogTime = 0;
unsigned long lastValidCompassTime = 0;
unsigned long lastCompassReadTime = 0;

// Data storage
struct CMPS12Data {
    float heading;      // 0-360 degrees
    float pitch;        // -90 to +90 degrees
    float roll;         // -90 to +90 degrees
    uint8_t calibration; // 0-3 (3 is fully calibrated)
    bool valid;         // Data validity flag
    bool isCalibrated;  // Whether compass meets minimum calibration level
} cmpsData;

// Navigation data
struct NavigationData {
    float targetHeading;    // Desired heading in degrees
    float headingError;     // Difference between current and target heading
    bool compassValid;      // Overall compass system status
} navData;

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
    
    // Initialize navigation data
    navData.targetHeading = 0.0;
    navData.headingError = 0.0;
    navData.compassValid = false;
    
    // Print header
    Serial.println("\nAutonomous Sailboat - Compass System");
    Serial.println("Time(ms) | Heading(°) | Pitch(°) | Roll(°) | Calibration | Status");
    Serial.println("------------------------------------------------------------------");
}

void loop() {
    // Read compass data at appropriate interval
    if (millis() - lastCompassReadTime >= COMPASS_READ_INTERVAL) {
        readCMPS12Data();
        lastCompassReadTime = millis();
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

void updateNavigation() {
    if (!navData.compassValid) {
        return;
    }
    
    // Calculate heading error (normalized to -180 to 180 degrees)
    navData.headingError = navData.targetHeading - cmpsData.heading;
    if (navData.headingError > 180.0) {
        navData.headingError -= 360.0;
    } else if (navData.headingError < -180.0) {
        navData.headingError += 360.0;
    }
    
    // Check for compass timeout
    if (millis() - lastValidCompassTime > COMPASS_TIMEOUT) {
        navData.compassValid = false;
    }
}

void logData() {
    // Print timestamp
    Serial.print(millis());
    Serial.print(" | ");
    
    // Print current heading
    Serial.print(cmpsData.heading, 1);
    Serial.print("° | ");
    
    // Print pitch with sign
    if (cmpsData.pitch >= 0) Serial.print("+");
    Serial.print(cmpsData.pitch, 1);
    Serial.print("° | ");
    
    // Print roll with sign
    if (cmpsData.roll >= 0) Serial.print("+");
    Serial.print(cmpsData.roll, 1);
    Serial.print("° | ");
    
    // Print calibration status
    switch(cmpsData.calibration) {
        case 0:
            Serial.print("Not Calibrated");
            break;
        case 1:
            Serial.print("Partial");
            break;
        case 2:
            Serial.print("Mostly");
            break;
        case 3:
            Serial.print("Fully");
            break;
        default:
            Serial.print("Unknown");
            break;
    }
    Serial.print(" | ");
    
    // Print system status
    if (!cmpsData.valid) {
        Serial.println("COMPASS ERROR");
    } else if (!cmpsData.isCalibrated) {
        Serial.println("NEEDS CALIBRATION");
    } else {
        Serial.println("OK");
    }
} 