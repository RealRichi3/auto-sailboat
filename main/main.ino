#include <Wire.h>
#include <Servo.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <SD.h>
#include <SPI.h>

// CMPS12 Commands
#define CMPS_GET_ANGLE8 0x12
#define CMPS_GET_ANGLE16 0x13
#define CMPS_GET_PITCH 0x14
#define CMPS_GET_ROLL 0x15
#define CMPS_CALIBRATION_STATUS 0x24

// GPS Pins
#define GPS_RX_PIN 17  // GPS module TX (Arduino receives here)
#define GPS_TX_PIN 16  // GPS module RX (Arduino sends here)

// Wind Sensor Pins
#define WIND_SPEED_PIN 2       // Digital pin for Davis anemometer pulses
#define WIND_DIRECTION_PIN A15  // Analog pin for Davis wind vane

// Servo Pins
#define RUDDER_SERVO_PIN 4     // PWM pin for rudder servo
#define SAIL_SERVO_PIN 5       // PWM pin for sail servo

// Navigation Constants
#define WAYPOINT_RADIUS 10     // meters
#define MAX_RUDDER_ANGLE 45    // degrees from neutral
#define COMPASS_TIMEOUT 5000   // milliseconds
#define COMPASS_READ_INTERVAL 100  // milliseconds between compass readings
#define MIN_CALIBRATION_LEVEL 2  // Require at least "Mostly Calibrated" (0-3 scale)
#define GPS_TIMEOUT 10000      // milliseconds
#define MIN_SATELLITES 4       // Minimum satellites for valid GPS fix
#define SIMULATE_GPS true     // Set to true to enable GPS simulation

// Wind Constants
#define WIND_SPEED_INTERVAL 5000  // 5 seconds between wind speed calculations
#define WIND_DEBOUNCE_TIME 100    // 100ms debounce for anemometer
#define WIND_SPEED_CALIBRATION 2.4 // Calibration factor for wind speed (mph per pulse)

// Servo Constants
#define SERVO_RATE_LIMIT 10    // Maximum degrees change per cycle
#define RUDDER_MIN_ANGLE 0     // Minimum rudder angle (degrees)
#define RUDDER_MAX_ANGLE 180   // Maximum rudder angle (degrees)
#define SAIL_MIN_ANGLE 0       // Minimum sail angle (degrees)
#define SAIL_MAX_ANGLE 180     // Maximum sail angle (degrees)
#define RUDDER_NEUTRAL 90      // Neutral rudder position
#define SAIL_NEUTRAL 90        // Neutral sail position

// Waypoints (latitude, longitude)
const float waypoints[][2] = {
    {52.486333, -1.889420},  // Waypoint 1
    {52.485990, -1.889547},  // Waypoint 2
    {52.486070, -1.888934}   // Waypoint 3
};
const int NUM_WAYPOINTS = 3;

// Timing
const unsigned long LOG_INTERVAL = 1000; // Log every second
unsigned long lastLogTime = 0;
unsigned long lastValidCompassTime = 0;
unsigned long lastCompassReadTime = 0;
unsigned long lastWindSpeedTime = 0;
unsigned long lastServoUpdateTime = 0;
unsigned long lastValidGPSTime = 0;

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

// GPS data
struct GPSData {
    float latitude;     // Current latitude
    float longitude;    // Current longitude
    float speed;        // Speed in km/h
    float course;       // Course over ground in degrees
    int satellites;     // Number of satellites in view
    bool valid;         // Data validity flag
    int currentWaypoint; // Current waypoint index
} gpsData;

// Servo data
struct ServoData {
    float rudderAngle;  // Current rudder angle (degrees)
    float sailAngle;    // Current sail angle (degrees)
    float targetRudder; // Target rudder angle (degrees)
    float targetSail;   // Target sail angle (degrees)
    bool valid;         // Data validity flag
} servoData;

// GPS objects
SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN);
TinyGPSPlus gps;

// Servo objects
Servo rudderServo;
Servo sailServo;

// GPS simulation data
struct GPSSimulation {
    float currentLat;    // Current simulated latitude
    float currentLon;    // Current simulated longitude
    float speed;         // Simulated speed in km/h
    float heading;       // Simulated heading
    unsigned long lastUpdate; // Last simulation update time
} gpsSim;

// SD Card Pin
#define SD_CS_PIN 53  // Chip Select pin for SD card

// File for logging
File logFile;

void setup() {
    // Initialize Serial for logging
    Serial.begin(115200);
    while (!Serial); // Wait for Serial to be ready
    
    // Initialize SD card
    if (!SD.begin(SD_CS_PIN)) {
        Serial.println("SD card initialization failed!");
        while (1); // Halt if SD card fails
    }
    
    // Create or open log file
    logFile = SD.open("sailboat.csv", FILE_WRITE);
    if (!logFile) {
        Serial.println("Error opening sailboat.csv");
        while (1); // Halt if file can't be opened
    }
    
    // Write CSV header if file is empty
    if (logFile.size() == 0) {
        logFile.println("Time(ms),Heading(°),WindDir(°),WindSpd(mph),Lat,Lon,Sat,Rudder(°),Sail(°),TargetSail(°),Status");
    }
    
    // Initialize CMPS12 on Serial3
    Serial3.begin(9600);
    
    // Initialize GPS
    gpsSerial.begin(9600);
    
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
    
    // Initialize GPS data
    gpsData.latitude = 0.0;
    gpsData.longitude = 0.0;
    gpsData.speed = 0.0;
    gpsData.course = 0.0;
    gpsData.satellites = 0;
    gpsData.valid = false;
    gpsData.currentWaypoint = 0;
    
    // Initialize servo data
    servoData.rudderAngle = RUDDER_NEUTRAL;
    servoData.sailAngle = SAIL_NEUTRAL;
    servoData.targetRudder = RUDDER_NEUTRAL;
    servoData.targetSail = SAIL_NEUTRAL;
    servoData.valid = true;
    
    // Setup wind speed pin with interrupt
    pinMode(WIND_SPEED_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(WIND_SPEED_PIN), windSpeedISR, FALLING);
    
    // Attach servos
    rudderServo.attach(RUDDER_SERVO_PIN);
    sailServo.attach(SAIL_SERVO_PIN);
    
    // Set servos to neutral position
    rudderServo.write(RUDDER_NEUTRAL);
    sailServo.write(SAIL_NEUTRAL);
    
    // Print header
    Serial.println("\nAutonomous Sailboat - Navigation System");
    Serial.println("Time(ms) | Heading(°) | Wind Dir(°) | Wind Spd(mph) | Lat | Lon | Sat | Rudder(°) | Sail(°) (Target) | Status");
    Serial.println("------------------------------------------------------------------");
    
    // Initialize GPS simulation
    if (SIMULATE_GPS) {
        // Start at first waypoint
        gpsSim.currentLat = waypoints[0][0];
        gpsSim.currentLon = waypoints[0][1];
        gpsSim.speed = 5.0; // 5 km/h
        gpsSim.heading = 0.0;
        gpsSim.lastUpdate = millis();
    }
    
    // Add a delay to allow Serial to be ready
    delay(2000);
    
    // Add a command to read the log file
    Serial.println("\nType 'r' to read the log file, or any other key to continue...");
    while (!Serial.available()) {
        delay(100);
    }
    
    if (Serial.read() == 'r') {
        readLogFile();
    }
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
    
    // Read GPS data
    readGPSData();
    
    // Update navigation and servo positions
    updateNavigation();
    updateServos();
    
    // Log data at specified interval
    if (millis() - lastLogTime >= LOG_INTERVAL) {
        logData();
        lastLogTime = millis();
    }
    
    // Check for read command during operation
    if (Serial.available()) {
        char command = Serial.read();
        if (command == 'r') {
            readLogFile();
        }
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

void windSpeedISR() {
    unsigned long currentTime = millis();
    if (currentTime - windData.lastPulseTime > WIND_DEBOUNCE_TIME) {
        windData.pulseCount++;
        windData.lastPulseTime = currentTime;
    }
}

void readGPSData() {
    if (SIMULATE_GPS) {
        simulateGPSData();
    } else {
        // Original GPS reading code
        while (gpsSerial.available() > 0) {
            if (gps.encode(gpsSerial.read())) {
                if (gps.location.isValid() && gps.satellites.value() >= MIN_SATELLITES) {
                    gpsData.latitude = gps.location.lat();
                    gpsData.longitude = gps.location.lng();
                    gpsData.speed = gps.speed.kmph();
                    gpsData.course = gps.course.deg();
                    gpsData.satellites = gps.satellites.value();
                    gpsData.valid = true;
                    lastValidGPSTime = millis();
                }
            }
        }
    }
    
    // Check GPS timeout
    if (millis() - lastValidGPSTime > GPS_TIMEOUT) {
        gpsData.valid = false;
    }
}

void simulateGPSData() {
    unsigned long currentTime = millis();
    float timeDiff = (currentTime - gpsSim.lastUpdate) / 1000.0; // Convert to seconds
    
    if (timeDiff >= 1.0) { // Update every second
        // Calculate distance moved
        float distanceMoved = (gpsSim.speed * timeDiff) / 3.6; // Convert km/h to m/s
        
        // Calculate bearing to next waypoint
        float targetBearing = calculateBearing(
            gpsSim.currentLat, gpsSim.currentLon,
            waypoints[gpsData.currentWaypoint][0], waypoints[gpsData.currentWaypoint][1]
        );
        
        // Update position
        float latChange = distanceMoved * cos(targetBearing * DEG_TO_RAD) / 111111.0;
        float lonChange = distanceMoved * sin(targetBearing * DEG_TO_RAD) / (111111.0 * cos(gpsSim.currentLat * DEG_TO_RAD));
        
        gpsSim.currentLat += latChange;
        gpsSim.currentLon += lonChange;
        
        // Update GPS data
        gpsData.latitude = gpsSim.currentLat;
        gpsData.longitude = gpsSim.currentLon;
        gpsData.speed = gpsSim.speed;
        gpsData.course = targetBearing;
        gpsData.satellites = 8; // Simulated number of satellites
        gpsData.valid = true;
        
        // Check if we've reached the current waypoint
        float distanceToWaypoint = calculateDistance(
            gpsSim.currentLat, gpsSim.currentLon,
            waypoints[gpsData.currentWaypoint][0], waypoints[gpsData.currentWaypoint][1]
        );
        
        if (distanceToWaypoint < WAYPOINT_RADIUS) {
            gpsData.currentWaypoint = (gpsData.currentWaypoint + 1) % NUM_WAYPOINTS;
        }
        
        gpsSim.lastUpdate = currentTime;
        lastValidGPSTime = currentTime;
    }
}

void updateNavigation() {
    if (!cmpsData.valid || !windData.valid || !gpsData.valid) {
        // If sensors are invalid, set servos to neutral
        servoData.targetRudder = RUDDER_NEUTRAL;
        servoData.targetSail = SAIL_NEUTRAL;
        return;
    }
    
    // Calculate distance to current waypoint
    float distanceToWaypoint = calculateDistance(
        gpsData.latitude, gpsData.longitude,
        waypoints[gpsData.currentWaypoint][0], waypoints[gpsData.currentWaypoint][1]
    );
    
    // Check if waypoint reached
    if (distanceToWaypoint < WAYPOINT_RADIUS) {
        gpsData.currentWaypoint = (gpsData.currentWaypoint + 1) % NUM_WAYPOINTS;
    }
    
    // Calculate bearing to next waypoint
    float bearing = calculateBearing(
        gpsData.latitude, gpsData.longitude,
        waypoints[gpsData.currentWaypoint][0], waypoints[gpsData.currentWaypoint][1]
    );
    
    // Calculate target angles based on bearing and wind
    calculateTargetAngles(bearing);
}

float calculateDistance(float lat1, float lon1, float lat2, float lon2) {
    // Convert to radians
    lat1 = lat1 * DEG_TO_RAD;
    lon1 = lon1 * DEG_TO_RAD;
    lat2 = lat2 * DEG_TO_RAD;
    lon2 = lon2 * DEG_TO_RAD;
    
    // Earth radius in meters
    const float R = 6371000.0;
    
    // Haversine formula
    float dLat = lat2 - lat1;
    float dLon = lon2 - lon1;
    float a = sin(dLat/2) * sin(dLat/2) +
              cos(lat1) * cos(lat2) * sin(dLon/2) * sin(dLon/2);
    float c = 2 * atan2(sqrt(a), sqrt(1-a));
    return R * c;
}

float calculateBearing(float lat1, float lon1, float lat2, float lon2) {
    // Convert to radians
    lat1 = lat1 * DEG_TO_RAD;
    lon1 = lon1 * DEG_TO_RAD;
    lat2 = lat2 * DEG_TO_RAD;
    lon2 = lon2 * DEG_TO_RAD;
    
    float y = sin(lon2 - lon1) * cos(lat2);
    float x = cos(lat1) * sin(lat2) -
              sin(lat1) * cos(lat2) * cos(lon2 - lon1);
    float bearing = atan2(y, x) * RAD_TO_DEG;
    
    // Normalize to 0-360
    bearing = fmod(bearing + 360.0, 360.0);
    return bearing;
}

void calculateTargetAngles(float targetBearing) {
    // Calculate relative wind angle (0-360 degrees)
    float relativeWind = fmod(windData.direction - cmpsData.heading + 360.0, 360.0);
    
    // Basic sail control based on wind angle
    if (relativeWind <= 45 || relativeWind >= 315) {
        // Close-hauled (wind angle < 45°)
        servoData.targetSail = SAIL_MIN_ANGLE;
    } else if (relativeWind <= 135) {
        // Beam reach (45° < wind angle < 135°)
        servoData.targetSail = map(relativeWind, 45, 135, SAIL_MIN_ANGLE, SAIL_MAX_ANGLE);
    } else if (relativeWind <= 225) {
        // Broad reach (135° < wind angle < 225°)
        servoData.targetSail = SAIL_MAX_ANGLE;
    } else {
        // Running (225° < wind angle < 315°)
        servoData.targetSail = map(relativeWind, 225, 315, SAIL_MAX_ANGLE, SAIL_MIN_ANGLE);
    }
    
    // Basic rudder control (course keeping)
    float headingError = fmod(targetBearing - cmpsData.heading + 540.0, 360.0) - 180.0;
    servoData.targetRudder = RUDDER_NEUTRAL + constrain(headingError * 0.5, -MAX_RUDDER_ANGLE, MAX_RUDDER_ANGLE);
}

void updateServos() {
    // Rate limit servo movements
    rateLimitServoMovement(servoData.rudderAngle, servoData.targetRudder);
    rateLimitServoMovement(servoData.sailAngle, servoData.targetSail);
    
    // Update servo positions
    rudderServo.write(servoData.rudderAngle);
    sailServo.write(servoData.sailAngle);
}

void rateLimitServoMovement(float &current, float target) {
    float difference = target - current;
    if (abs(difference) > SERVO_RATE_LIMIT) {
        current += (difference > 0 ? SERVO_RATE_LIMIT : -SERVO_RATE_LIMIT);
    } else {
        current = target;
    }
}

void logData() {
    // Create log string
    String logString = "";
    
    // Add timestamp
    logString += String(millis());
    logString += ",";
    
    // Add heading
    logString += String(cmpsData.heading, 1);
    logString += ",";
    
    // Add wind direction
    logString += String(windData.direction, 1);
    logString += ",";
    
    // Add wind speed
    logString += String(windData.speed, 1);
    logString += ",";
    
    // Add GPS data
    logString += String(gpsData.latitude, 6);
    logString += ",";
    logString += String(gpsData.longitude, 6);
    logString += ",";
    logString += String(gpsData.satellites);
    logString += ",";
    
    // Add rudder angle
    logString += String(servoData.rudderAngle, 1);
    logString += ",";
    
    // Add sail angle and target sail angle
    logString += String(servoData.sailAngle, 1);
    logString += ",";
    logString += String(servoData.targetSail, 1);
    logString += ",";
    
    // Add status
    if (!cmpsData.valid) {
        logString += "COMPASS_ERROR";
    } else if (!cmpsData.isCalibrated) {
        logString += "NEEDS_CALIBRATION";
    } else if (!windData.valid) {
        logString += "WIND_SENSOR_ERROR";
    } else if (!gpsData.valid) {
        logString += "GPS_ERROR";
    } else if (!servoData.valid) {
        logString += "SERVO_ERROR";
    } else {
        logString += "OK";
    }
    
    // Write to SD card
    logFile.println(logString);
    logFile.flush(); // Ensure data is written to card
    
    // Also print to Serial for monitoring
    Serial.print(millis());
    Serial.print(" | ");
    Serial.print(cmpsData.heading, 1);
    Serial.print("° | ");
    Serial.print(windData.direction, 1);
    Serial.print("° | ");
    Serial.print(windData.speed, 1);
    Serial.print(" mph | ");
    Serial.print(gpsData.latitude, 6);
    Serial.print(" | ");
    Serial.print(gpsData.longitude, 6);
    Serial.print(" | ");
    Serial.print(gpsData.satellites);
    Serial.print(" | ");
    Serial.print(servoData.rudderAngle, 1);
    Serial.print("° | ");
    Serial.print(servoData.sailAngle, 1);
    Serial.print("° (T: ");
    Serial.print(servoData.targetSail, 1);
    Serial.print("°) | ");
    
    if (!cmpsData.valid) {
        Serial.println("COMPASS ERROR");
    } else if (!cmpsData.isCalibrated) {
        Serial.println("NEEDS CALIBRATION");
    } else if (!windData.valid) {
        Serial.println("WIND SENSOR ERROR");
    } else if (!gpsData.valid) {
        Serial.println("GPS ERROR");
    } else if (!servoData.valid) {
        Serial.println("SERVO ERROR");
    } else {
        Serial.println("OK");
    }
}

void readLogFile() {
    // Open the file for reading
    File logFile = SD.open("sailboat.csv");
    
    if (!logFile) {
        Serial.println("Error opening sailboat.csv for reading");
        return;
    }
    
    // Send start marker
    Serial.println("===LOG_FILE_START===");
    
    // Read from the file until there's nothing else in it
    while (logFile.available()) {
        Serial.write(logFile.read());
    }
    
    // Send end marker
    Serial.println("\n===LOG_FILE_END===");
    
    // Close the file
    logFile.close();
} 