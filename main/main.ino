#include <Wire.h>

// CMPS12 Commands
#define CMPS_GET_ANGLE8 0x12
#define CMPS_GET_ANGLE16 0x13
#define CMPS_GET_PITCH 0x14
#define CMPS_GET_ROLL 0x15
#define CMPS_CALIBRATION_STATUS 0x24

// Timing
const unsigned long LOG_INTERVAL = 1000; // Log every second
unsigned long lastLogTime = 0;

// Data storage
struct CMPS12Data {
    float heading;      // 0-360 degrees
    float pitch;        // -90 to +90 degrees
    float roll;         // -90 to +90 degrees
    uint8_t calibration; // 0-3 (3 is fully calibrated)
    bool valid;         // Data validity flag
} cmpsData;

void setup() {
    // Initialize Serial for logging
    Serial.begin(115200);
    while (!Serial); // Wait for Serial to be ready
    
    // Initialize CMPS12 on Serial3
    Serial3.begin(9600);
    
    // Print header
    Serial.println("\nCMPS12 Data Logger");
    Serial.println("Time(ms) | Heading(°) | Pitch(°) | Roll(°) | Calibration");
    Serial.println("--------------------------------------------------------");
}

void loop() {
    // Read all CMPS12 data
    readCMPS12Data();
    
    // Log data at specified interval
    if (millis() - lastLogTime >= LOG_INTERVAL) {
        logData();
        lastLogTime = millis();
    }
    
    delay(100); // Small delay to prevent overwhelming the sensor
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
}

void logData() {
    // Print timestamp
    Serial.print(millis());
    Serial.print(" | ");
    
    if (!cmpsData.valid) {
        Serial.println("Error reading CMPS12 data!");
        return;
    }
    
    // Print heading with 1 decimal place
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
    
    // Print calibration status with description
    switch(cmpsData.calibration) {
        case 0:
            Serial.println("Not Calibrated");
            break;
        case 1:
            Serial.println("Partially Calibrated");
            break;
        case 2:
            Serial.println("Mostly Calibrated");
            break;
        case 3:
            Serial.println("Fully Calibrated");
            break;
        default:
            Serial.println("Unknown");
            break;
    }
} 