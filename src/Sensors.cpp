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
    Serial.println("Calibrating for 5 seconds...");
    
    pinMode(ONBOARD_LED, OUTPUT);
    digitalWrite(ONBOARD_LED, HIGH);
    
    unsigned long startTime = millis();
    int detectionCount[SensorCount] = {0};
    
    while (millis() - startTime < 5000) {
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
    // Digital sensors: HIGH = line detected (black), LOW = no line (white)
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
    
    // Line recovery: keep last position if all sensors lose line
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
    /*
     * SIMPLIFIED PATH DETECTION
     * ========================
     * 
     * Check which directions have line visible
     * Works at T-junctions, cross-junctions, 90° turns
     * 
     * Sensor Layout (5 sensors, 2cm spacing):
     *    S1   S2   S3   S4   S5
     *   Right     Center     Left
     * 
     * Path Detection Logic:
     * - LEFT path:     S4 OR S5 active (left-most 2 sensors)
     * - STRAIGHT path: S2 OR S3 OR S4 active (center 3 sensors)
     * - RIGHT path:    S1 OR S2 active (right-most 2 sensors)
     * 
     * Why overlapping sensors?
     * - 3cm line width with 2cm sensor spacing means sensors overlap coverage
     * - More reliable detection of available paths
     * 
     * Examples:
     * =========
     * 
     * T-Junction (Left + Straight):
     *     ███████████
     *         ║
     *         ║
     *    [·][█][█][█][·]  ← S2,S3,S4 active
     *    Result: left=true, straight=true, right=false
     * 
     * 90° Right Turn:
     *         ║
     *         ║
     *         ╚═════════
     *    [█][█][·][·][·]  ← S1,S2 active
     *    Result: left=false, straight=false, right=true
     * 
     * Cross Junction (All paths):
     *     ═══╬═══
     *         ║
     *    [█][█][█][█][█]  ← All sensors active
     *    Result: left=true, straight=true, right=true
     */
    
    readRaw(sensorValues);
    
    PathOptions paths;
    
    // LEFT path: Left-most 2 sensors
    paths.left = (sensorValues[4]);
    
    // STRAIGHT path: Center 3 sensors
    // This is the most important - must detect line ahead
    paths.straight = (sensorValues[1] || sensorValues[2] || sensorValues[3]);
    
    // RIGHT path: Right-most 2 sensors
    paths.right = (sensorValues[0]);
    
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
     *    [·][·][·][·][·]  ← All sensors see white
     *    (line has ended)
     * 
     * IMPORTANT:
     * ----------
     * - Returns true when ALL sensors are LOW (no line detected)
     * - This is the opposite of your original code!
     * - Original checked for all HIGH (all black) which was wrong
     * 
     * Usage in LSRB:
     * --------------
     * - In MAPPING mode: Line end = dead end → turn back (B)
     * - In SOLVING mode: Line end = maze complete → stop!
     */
    
    readRaw(sensorValues);
    
    // Check if ALL sensors see white (no line)
    for (uint8_t i = 0; i < SensorCount; i++) {
        if (sensorValues[i]) {
            return false;  // Found a sensor that sees line → NOT line end
        }
    }
    
    // All sensors see white → LINE END
    return true;
}

void Sensors::getSensorArray(bool* arr) {
    readRaw(arr);
}