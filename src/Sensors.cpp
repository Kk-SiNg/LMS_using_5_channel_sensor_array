/*
 * Sensors.cpp
 * SIMPLIFIED: Direct path detection, no complex intersection logic
 */

#include "Sensors.h"

Sensors::Sensors() {
    lastPosition = 0;
}

void Sensors::setup() {
    pinMode(SENSOR_PIN_1, INPUT);
    pinMode(SENSOR_PIN_2, INPUT);
    pinMode(SENSOR_PIN_3, INPUT);
    pinMode(SENSOR_PIN_4, INPUT);
    pinMode(SENSOR_PIN_5, INPUT);
    
    Serial.println("\n╔════════════════════════════════════════╗");
    Serial.println("║  Sensor Calibration                   ║");
    Serial.println("╚════════════════════════════════════════╝");
    Serial.println("5-Channel Digital TCRT5000 Array");
    Serial.println("Calibrating for 2 seconds...");
    
    pinMode(ONBOARD_LED, OUTPUT);
    digitalWrite(ONBOARD_LED, HIGH);
    
    unsigned long startTime = millis();
    int detectionCount[SensorCount] = {0};
    
    while (millis() - startTime < 2000) {
        readRaw(sensorValues);
        for (uint8_t i = 0; i < SensorCount; i++) {
            if (sensorValues[i]) detectionCount[i]++;
        }
        delay(10);
    }
    
    digitalWrite(ONBOARD_LED, LOW);
    Serial.println("✓ Calibration Complete");
    
    Serial.println("\nDetection counts:");
    for (uint8_t i = 0; i < SensorCount; i++) {
        Serial.print("  S");
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.print(detectionCount[i]);
        if (detectionCount[i] > 0) {
            Serial.println(" ✓");
        } else {
            Serial.println(" ✗ WARNING");
        }
    }
    Serial.println();
}

void Sensors::readRaw(bool* values) {
    // Digital sensors: HIGH = line detected (white) and LOW = no line (black)
    values[0] = digitalRead(SENSOR_PIN_1);  // Right-most
    values[1] = digitalRead(SENSOR_PIN_2);
    values[2] = digitalRead(SENSOR_PIN_3);  // Center
    values[3] = digitalRead(SENSOR_PIN_4);
    values[4] = digitalRead(SENSOR_PIN_5);  // Left-most
}

int16_t Sensors::getPosition() {
    readRaw(sensorValues);
    
    // Weighted average algorithm
    long weightedSum = 0;
    int activeCount = 0;
    
    for (uint8_t i = 0; i < SensorCount; i++) {
        if (sensorValues[i]) {
            weightedSum += weights[i];
            activeCount++;
        }
    }
    
    // imp ---> Line recovery: keep last position if all sensors lose line
    if (activeCount > 0) {
        lastPosition = weightedSum / activeCount;
    }
    
    return lastPosition;
}

int16_t Sensors::getLineError() {
    int16_t position = getPosition();
    return position - setpoint;
}

bool Sensors::onLine() {
    /*
     * Check if robot is currently on a line
     * Returns true if ANY sensor detects the line
     */
    readRaw(sensorValues);
    
    for (uint8_t i = 0; i < SensorCount; i++) {
        if (sensorValues[i]) {
            return true;  // Found line
        }
    }
    
    return false;  // No line detected
}

PathOptions Sensors::getAvailablePaths() {
    readRaw(sensorValues);
    
    PathOptions paths;
    
    // LEFT path: Need BOTH leftmost sensors OR clear left junction pattern
    paths.left = (sensorValues[0] && sensorValues[1] && sensorValues[2]);
    
    // STRAIGHT path: Center sensor(s) detect line
    paths.straight = (sensorValues[2] || sensorValues[1] || sensorValues[3]);
    
    // RIGHT path: Need BOTH rightmost sensors OR clear right junction pattern
    paths.right = (sensorValues[2] && sensorValues[3] && sensorValues[4]);
    
    return paths;
}

bool Sensors::isLineEnd() {
    /*
     * CORRECTED LINE END DETECTION
     * ============================
     * 
     * Line end = ALL sensors see WHITE (background)
     * 
     * When does this happen?
     * ----------------------
     * 1. Dead end: Robot reaches end of line
     * 2. Finish line: After last turn, line just stops
     * 3. Lost line: Robot went off track (shouldn't happen with PID)
     * 
     * Visual:
     * -------
     * Before line end:
     *    [·][·][█][·][·]  ← Following line
     *           ║
     *           ║  (moving forward)
     *           ║
     *           ▼
     * 
     * At line end:
     *    [·][·][·][·][·]  ← All sensors see black
     *    (line has ended)
     * 
     * IMPORTANT:
     * ----------
     * - Returns true when ALL sensors are LOW (no line detected)
     * 
     * Usage in LSRB:
     * --------------
     * - In MAPPING mode: Line end = dead end → turn back (B)
     * - In SOLVING mode: Line end = maze complete → stop!
     */
    
    readRaw(sensorValues);
    
    // Check if ALL sensors see black (no line)
    for (uint8_t i = 0; i < SensorCount; i++) {
        if (sensorValues[i]) {
            return false;  // Found a sensor that sees line → NOT line end
        }
    }
    // All sensors see black → LINE END
    return true;
}

// to provide sensor info to others
void Sensors::getSensorArray(bool* arr) {
    readRaw(arr);
}