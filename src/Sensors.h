/*
 * Sensors.h
 * SIMPLIFIED: No intersection detection, just path availability
 * For LSRB: Check left, straight, right paths directly
 */

#pragma once
#include "Pins.h"
#include <Arduino.h>

// Path availability at junction
struct PathOptions {
    bool left = false;
    bool straight = false;
    bool right = false;
};

class Sensors {
public:
    Sensors();
    void setup();
    
    // Get line position error for PID (-4 to +4)
    int16_t getLineError();
    
    // Check which paths are available (for LSRB at junctions)
    PathOptions getAvailablePaths();
    
    // Detect if we've completely lost the line (dead end)
    bool isLineEnd();
    
    // Read raw sensor values
    void readRaw(bool* values);
    
    // Get weighted position (-4 to +4)
    int16_t getPosition();
    
    // Get sensor array for WiFi monitoring
    void getSensorArray(bool* arr);
    
    // Check if currently on a line (any sensor active)
    bool onLine();

private:
    bool sensorValues[SensorCount];
    const int16_t weights[SensorCount] = {-4, -2, 0, 2, 4};
    const int16_t setpoint = 0;
    int16_t lastPosition;
};