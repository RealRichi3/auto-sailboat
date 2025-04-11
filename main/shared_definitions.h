#ifndef SHARED_DEFINITIONS_H
#define SHARED_DEFINITIONS_H

// Navigation Constants
#define WAYPOINT_RADIUS 10     // meters
#define MAX_RUDDER_ANGLE 45    // degrees from neutral
#define COMPASS_TIMEOUT 5000   // milliseconds
#define COMPASS_READ_INTERVAL 100  // milliseconds between compass readings
#define MIN_CALIBRATION_LEVEL 2  // Require at least "Mostly Calibrated" (0-3 scale)
#define MIN_SATELLITES 4       // Minimum satellites for valid GPS fix

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

// Data structures
struct CMPS12Data {
    float heading;      // 0-360 degrees
    float pitch;        // -90 to +90 degrees
    float roll;         // -90 to +90 degrees
    uint8_t calibration; // 0-3 (3 is fully calibrated)
    bool valid;         // Data validity flag
    bool isCalibrated;  // Whether compass meets minimum calibration level
};

struct WindData {
    float speed;        // Wind speed in mph
    float direction;    // Wind direction in degrees (0-360)
    volatile unsigned int pulseCount; // Anemometer pulse count
    unsigned long lastPulseTime;     // Last pulse timestamp
    bool valid;         // Data validity flag
};

struct GPSData {
    float latitude;     // Current latitude
    float longitude;    // Current longitude
    float speed;        // Speed in km/h
    float course;       // Course over ground in degrees
    int satellites;     // Number of satellites in view
    float hdop;         // Horizontal dilution of precision
    bool valid;         // Data validity flag
    int currentWaypoint; // Current waypoint index
};

struct ServoData {
    float rudderAngle;  // Current rudder angle (degrees)
    float sailAngle;    // Current sail angle (degrees)
    float targetRudder; // Target rudder angle (degrees)
    float targetSail;   // Target sail angle (degrees)
    bool valid;         // Data validity flag
};

struct GPSSimulation {
    float currentLat;    // Current simulated latitude
    float currentLon;    // Current simulated longitude
    float speed;         // Simulated speed in km/h
    float heading;       // Simulated heading
    unsigned long lastUpdate; // Last simulation update time
};

#endif // SHARED_DEFINITIONS_H 