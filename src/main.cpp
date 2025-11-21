/*
 * main.cpp
 * SIMPLIFIED: Direct path detection at junctions
 * No intersection detection needed!
 */

#include <WiFi.h>
#include "Pins.h"
#include "Sensors.h"
#include "Motors.h"
#include "PathOptimization.h"
#include "PIDController.h"

// === WiFi Server ===
WiFiServer server(TELNET_PORT);
WiFiClient client;

// === Tuning Constants ===
#define MAX_PATH_LENGTH 50
#define HIGH_SPEED 255
#define SLOWDOWN_TICKS 500

// === Global Objects ===
Sensors sensors;
Motors motors;
PathOptimization optimizer;
PIDController pid(45.0, 0.5, 25.0);

// === Robot State Machine ===
enum RobotState {
    CALIBRATING,
    WAIT_FOR_RUN_1,
    MAPPING,
    OPTIMIZING,
    WAIT_FOR_RUN_2,
    SOLVING,
    FINISHED
};
RobotState currentState = CALIBRATING;

// === Path & Distance Storage ===
String rawPath = "";
long pathSegments[100];
int pathIndex = 0;

String optimizedPath = "";
long optimizedSegments[100];
int optimizedPathLength = 0;
int solvePathIndex = 0;

// === Solving Sub-State Machine ===
enum SolvingSubState {
    SOLVE_TURN,
    SOLVE_FAST_RUN,
    SOLVE_SLOW_RUN,
    SOLVE_FINAL_RUN
};
SolvingSubState solveState = SOLVE_TURN;

// === Control Variables ===
bool robotRunning = false;
unsigned long lastWiFiUpdate = 0;
unsigned long lastDebugPrint = 0;

// === Junction Detection Variables ===
unsigned long lastJunctionTime = 0;
const unsigned long junctionDebounce = 500;  // 500ms between junctions

// === Function Declarations ===
void setupWiFi();
void handleWiFiClient();
void processCommand(String cmd);
void printMenu();
void printStatus();
void printMotorParams();
void runPID(int currentBaseSpeed);

void setup() {
    Serial.begin(9600);
    Serial.println("\n╔════════════════════════════════════════╗");
    Serial.println("║  Mesmerize Line Maze Solver          ║");
    Serial.println("║  IIT Bombay Competition Edition      ║");
    Serial.println("║  SIMPLIFIED Junction Detection       ║");
    Serial.println("╚════════════════════════════════════════╝\n");
    
    pinMode(ONBOARD_LED, OUTPUT);
    pinMode(USER_BUTTON, INPUT_PULLUP);
    
    motors.setup();
    setupWiFi();
    
    // Calibrate while rotating
    currentState = CALIBRATING;
    motors.rotate();
    sensors.setup();
    motors.stopBrake();
    
    pid.setOutputLimits(-BASE_SPEED, BASE_SPEED);
    
    Serial.println("╔════════════════════════════════════════╗");
    Serial.println("║  Configuration                        ║");
    Serial.println("╠════════════════════════════════════════╣");
    Serial.print("║  Kp = ");
    Serial.print(pid.getKp(), 1);
    Serial.print("  Ki = ");
    Serial.print(pid.getKi(), 1);
    Serial.print("  Kd = ");
    Serial.print(pid.getKd(), 1);
    Serial.println("   ║");
    Serial.print("║  BASE_SPEED = ");
    Serial.print(BASE_SPEED);
    Serial.print("  TICKS_90° = ");
    Serial.print(TICKS_FOR_90_DEG);
    Serial.println("  ║");
    Serial.println("╚════════════════════════════════════════╝\n");
    
    currentState = WAIT_FOR_RUN_1;
    Serial.println("✓ Ready for Run 1 (Mapping)");
    Serial.println("Press button or type START via WiFi\n");
}

void loop() {
    handleWiFiClient();
    
    // WiFi updates every 200ms
    if (millis() - lastWiFiUpdate > 200 && client && client.connected()) {
        bool sensorVals[5];
        sensors.getSensorArray(sensorVals);
        
        client.print("S:[");
        for (int i = 0; i < 5; i++) {
            client.print(sensorVals[i] ? "█" : "·");
        }
        client.print("] Err:");
        client.print(sensors.getLineError());
        client.print(" Out:");
        client.print((int)pid.getOutput());
        client.print(" | ");
        
        switch(currentState) {
            case WAIT_FOR_RUN_1: client.print("WAIT_RUN1"); break;
            case MAPPING: client.print("MAPPING"); break;
            case OPTIMIZING: client.print("OPTIMIZING"); break;
            case WAIT_FOR_RUN_2: client.print("WAIT_RUN2"); break;
            case SOLVING: client.print("SOLVING"); break;
            case FINISHED: client.print("FINISHED"); break;
            default: client.print("CALIBRATING");
        }
        client.println();
        
        lastWiFiUpdate = millis();
    }
    
    switch (currentState) {
        
        case WAIT_FOR_RUN_1:
            if (digitalRead(USER_BUTTON) == LOW || robotRunning) {
                delay(50);
                if (digitalRead(USER_BUTTON) == LOW || robotRunning) {
                    Serial.println("\n>>> Run 1 MAPPING Started <<<");
                    if (client && client.connected()) {
                        client.println(">>> Run 1 MAPPING Started!");
                    }
                    currentState = MAPPING;
                    robotRunning = true;
                    pathIndex = 0;
                    rawPath = "";
                    lastJunctionTime = 0;
                    pid.reset();
                    while(digitalRead(USER_BUTTON) == LOW);
                }
            }
            break;
            
        case MAPPING:
        {
            if (!robotRunning) {
                motors.stopBrake();
                break;
            }
            
            // Check for line end FIRST (dead end detection)
            if (sensors.isLineEnd()) {
                motors.stopBrake();
                
                long segmentTicks = motors.getAverageCount();
                
                if (pathIndex >= 100) {
                    Serial.println("ERROR: Path array full!");
                    currentState = FINISHED;
                    break;
                }
                
                Serial.println("→ DEAD END (U-turn)");
                motors.turn_180_back();
                rawPath += 'B';
                
                pathSegments[pathIndex] = segmentTicks;
                pathIndex++;
                motors.clearEncoders();
                pid.reset();
                lastJunctionTime = millis();
                delay(200);
                break;  // Exit switch to restart loop
            }
            
            // Normal line following with PID
            runPID(BASE_SPEED);
            
            // Debug output every 500ms
            if (millis() - lastDebugPrint > 500) {
                Serial.print("Err:");
                Serial.print(sensors.getLineError());
                Serial.print(" Out:");
                Serial.print((int)pid.getOutput());
                Serial.print(" Path:");
                Serial.print(rawPath);
                Serial.print("(");
                Serial.print(pathIndex);
                Serial.println(")");
                lastDebugPrint = millis();
            }
            
            // Check for junction (debounced)
            if (millis() - lastJunctionTime > junctionDebounce) {
                PathOptions paths = sensors.getAvailablePaths();
                
                // Junction detected if we have multiple path options
                // OR if we only have left OR only right (90° turn)
                bool isJunction = false;
                int pathCount = 0;
                if (paths.left) pathCount++;
                if (paths.straight) pathCount++;
                if (paths.right) pathCount++;
                
                // Junction = more than just straight, OR only left/right (90° turn)
                if (pathCount > 1 || (pathCount == 1 && !paths.straight)) {
                    isJunction = true;
                }
                
                if (isJunction) {
                    motors.stopBrake();
                    
                    long segmentTicks = motors.getAverageCount();
                    
                    if (pathIndex >= 100) {
                        Serial.println("ERROR: Path array full!");
                        currentState = FINISHED;
                        break;
                    }
                    
                    // Move to center of junction
                    motors.moveForward(TICKS_TO_CENTER);
                    delay(100);
                    
                    // Re-check paths after centering
                    paths = sensors.getAvailablePaths();
                    
                    // LSRB (Left-Straight-Right-Back) Logic
                    if (paths.left) {
                        Serial.println("→ LEFT");
                        motors.turn_90_left();
                        rawPath += 'L';
                    }
                    else if (paths.straight) {
                        Serial.println("→ STRAIGHT");
                        rawPath += 'S';
                    }
                    else if (paths.right) {
                        Serial.println("→ RIGHT");
                        motors.turn_90_right();
                        rawPath += 'R';
                    }
                    else {
                        // No paths available (shouldn't happen at junction)
                        Serial.println("→ NO PATH (U-turn)");
                        motors.turn_180_back();
                        rawPath += 'B';
                    }
                    
                    pathSegments[pathIndex] = segmentTicks;
                    pathIndex++;
                    motors.clearEncoders();
                    pid.reset();
                    lastJunctionTime = millis();
                    delay(100);
                }
            }
            
            break;
        }
            
        case OPTIMIZING:
        {
            motors.stopBrake();
            robotRunning = false;
            
            Serial.println("\n╔════════════════════════════════════════╗");
            Serial.println("║  Run 1 Complete - Optimizing         ║");
            Serial.println("╚════════════════════════════════════════╝");
            Serial.print("Raw Path: ");
            Serial.print(rawPath);
            Serial.print(" (");
            Serial.print(rawPath.length());
            Serial.println(" moves)");
            
            if (rawPath.length() == 0) {
                Serial.println("ERROR: No path recorded!");
                currentState = FINISHED;
                break;
            }
            
            optimizedPath = rawPath;
            optimizedPathLength = rawPath.length();
            
            // Copy segments for optimization
            for (int i = 0; i < pathIndex && i < 100; i++) {
                optimizedSegments[i] = pathSegments[i];
            }
            
            Serial.println("\nOptimizing...");
            bool changesMade = true;
            int iterations = 0;
            
            while(changesMade && iterations < 10) {
                int oldLength = optimizedPathLength;
                optimizer.optimize(optimizedPath, optimizedSegments, optimizedPathLength);
                changesMade = (oldLength != optimizedPathLength);
                iterations++;
            }
            
            Serial.print("Optimized: ");
            Serial.print(optimizedPath);
            Serial.print(" (");
            Serial.print(optimizedPath.length());
            Serial.println(" moves)");
            Serial.print("Saved: ");
            Serial.print(rawPath.length() - optimizedPath.length());
            Serial.println(" moves");
            
            if (client && client.connected()) {
                client.println("\n=== Path Optimized ===");
                client.print("Raw: ");
                client.println(rawPath);
                client.print("Optimized: ");
                client.println(optimizedPath);
            }
            
            Serial.println("\n✓ Ready for Run 2 (Solving)");
            Serial.println("Press button or type START\n");
            currentState = WAIT_FOR_RUN_2;
            solvePathIndex = 0;
            break;
        }
        
        case WAIT_FOR_RUN_2:
            if (digitalRead(USER_BUTTON) == LOW || (robotRunning && optimizedPath.length() > 0)) {
                delay(50);
                if (digitalRead(USER_BUTTON) == LOW || robotRunning) {
                    Serial.println("\n>>> Run 2 SOLVING Started <<<");
                    if (client && client.connected()) {
                        client.println(">>> Run 2 SOLVING Started!");
                    }
                    currentState = SOLVING;
                    robotRunning = true;
                    solvePathIndex = 0;
                    solveState = SOLVE_TURN;
                    pid.reset();
                    while(digitalRead(USER_BUTTON) == LOW);
                }
            }
            break;
            
        case SOLVING:
            if (!robotRunning) {
                motors.stopBrake();
                break;
            }
            
            if (solveState == SOLVE_TURN) {
                if (solvePathIndex >= optimizedPathLength) {
                    // Finished all turns
                    solveState = SOLVE_FINAL_RUN;
                    Serial.println("→ Final segment");
                }
                else {
                    char turn = optimizedPath[solvePathIndex];
                    
                    Serial.print("Seg ");
                    Serial.print(solvePathIndex + 1);
                    Serial.print("/");
                    Serial.print(optimizedPathLength);
                    Serial.print(": ");
                    Serial.println(turn);
                    
                    if (turn == 'L') motors.turn_90_left();
                    else if (turn == 'R') motors.turn_90_right();
                    // 'S' = no turn needed
                    
                    if (solvePathIndex >= optimizedPathLength - 1) {
                        solveState = SOLVE_FINAL_RUN;
                    }
                    else {
                        motors.clearEncoders();
                        solveState = SOLVE_FAST_RUN;
                    }
                    pid.reset();
                    delay(100);
                }
            }
            else if (solveState == SOLVE_FAST_RUN) {
                long currentTicks = motors.getAverageCount();
                long targetTicks = optimizedSegments[solvePathIndex + 1];
                
                if (currentTicks < (targetTicks - SLOWDOWN_TICKS)) {
                    runPID(HIGH_SPEED);
                }
                else {
                    solveState = SOLVE_SLOW_RUN;
                }
            }
            else if (solveState == SOLVE_SLOW_RUN) {
                long currentTicks = motors.getAverageCount();
                long targetTicks = optimizedSegments[solvePathIndex + 1];
                
                if (currentTicks < targetTicks) {
                    runPID(BASE_SPEED);
                }
                else {
                    motors.stopBrake();
                    motors.moveForward(TICKS_TO_CENTER);
                    solvePathIndex++;
                    solveState = SOLVE_TURN;
                }
            }
            else if (solveState == SOLVE_FINAL_RUN) {
                runPID(BASE_SPEED);
                
                // Check for line end (finish line)
                if (sensors.isLineEnd()) {
                    motors.stopBrake();
                    robotRunning = false;
                    
                    Serial.println("\n╔════════════════════════════════════════╗");
                    Serial.println("║                                        ║");
                    Serial.println("║      🏆  MAZE SOLVED!  🏆              ║");
                    Serial.println("║                                        ║");
                    Serial.println("║   IIT Bombay Mesmerize Complete!     ║");
                    Serial.println("║                                        ║");
                    Serial.println("╚════════════════════════════════════════╝\n");
                    
                    if (client && client.connected()) {
                        client.println("╔════════════════════════════════════════╗");
                        client.println("║      🏆  MAZE SOLVED!  🏆              ║");
                        client.println("╚════════════════════════════════════════╝");
                    }
                    
                    currentState = FINISHED;
                }
            }
            break;
            
        case FINISHED:
            digitalWrite(ONBOARD_LED, !digitalRead(ONBOARD_LED));
            delay(200);
            break;
            
        case CALIBRATING:
            break;
    }
}

void runPID(int currentBaseSpeed) {
    int16_t error = sensors.getLineError();
    float correction = pid.compute(error);
    
    int leftSpeed = currentBaseSpeed - (int)correction;
    int rightSpeed = currentBaseSpeed + (int)correction;
    
    motors.setSpeeds(leftSpeed, rightSpeed);
}

// WiFi functions (same as before)
void setupWiFi() {
    Serial.println("╔════════════════════════════════════════╗");
    Serial.println("║  WiFi Setup                           ║");
    Serial.println("╚════════════════════════════════════════╝");
    Serial.print("Connecting to: ");
    Serial.println(WIFI_SSID);
    
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(500);
        Serial.print(".");
        attempts++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\n✓ WiFi Connected!");
        Serial.print("IP: ");
        Serial.println(WiFi.localIP());
        Serial.print("Telnet: ");
        Serial.print(WiFi.localIP());
        Serial.print(":");
        Serial.println(TELNET_PORT);
        server.begin();
    } else {
        Serial.println("\n✗ WiFi Failed! Standalone mode.");
    }
    Serial.println();
}

void handleWiFiClient() {
    if (server.hasClient()) {
        if (!client || !client.connected()) {
            if (client) client.stop();
            client = server.available();
            Serial.println("✓ WiFi client connected");
            client.println("\n╔════════════════════════════════════════╗");
            client.println("║  Mesmerize WiFi Tuning                ║");
            client.println("╚════════════════════════════════════════╝");
            printMenu();
        }
    }
    
    if (client && client.connected() && client.available()) {
        String command = client.readStringUntil('\n');
        command.trim();
        processCommand(command);
    }
}

void processCommand(String cmd) {
    cmd.toUpperCase();
    
    if (cmd.startsWith("KP ")) {
        float newKp = cmd.substring(3).toFloat();
        pid.setTunings(newKp, pid.getKi(), pid.getKd());
        client.print("✓ Kp = ");
        client.println(newKp, 2);
    }
    else if (cmd.startsWith("KI ")) {
        float newKi = cmd.substring(3).toFloat();
        pid.setTunings(pid.getKp(), newKi, pid.getKd());
        client.print("✓ Ki = ");
        client.println(newKi, 4);
    }
    else if (cmd.startsWith("KD ")) {
        float newKd = cmd.substring(3).toFloat();
        pid.setTunings(pid.getKp(), pid.getKi(), newKd);
        client.print("✓ Kd = ");
        client.println(newKd, 2);
    }
    else if (cmd.startsWith("BASE ")) {
        int newBase = cmd.substring(5).toInt();
        Motors::updateSpeeds(newBase, TURN_SPEED, MAX_SPEED);
        pid.setOutputLimits(-BASE_SPEED, BASE_SPEED);
        client.print("✓ BASE_SPEED = ");
        client.println(BASE_SPEED);
    }
    else if (cmd.startsWith("TURN ")) {
        int newTurn = cmd.substring(5).toInt();
        Motors::updateSpeeds(BASE_SPEED, newTurn, MAX_SPEED);
        client.print("✓ TURN_SPEED = ");
        client.println(TURN_SPEED);
    }
    else if (cmd.startsWith("MAX ")) {
        int newMax = cmd.substring(4).toInt();
        Motors::updateSpeeds(BASE_SPEED, TURN_SPEED, newMax);
        client.print("✓ MAX_SPEED = ");
        client.println(MAX_SPEED);
    }
    else if (cmd.startsWith("TICKS90 ")) {
        int newTicks = cmd.substring(8).toInt();
        Motors::updateTurnTicks(newTicks);
        client.print("✓ TICKS_FOR_90_DEG = ");
        client.println(TICKS_FOR_90_DEG);
    }
    else if (cmd.startsWith("CENTER ")) {
        int newCenter = cmd.substring(7).toInt();
        Motors::updateCenterTicks(newCenter);
        client.print("✓ TICKS_TO_CENTER = ");
        client.println(TICKS_TO_CENTER);
    }
    else if (cmd == "START" || cmd == "ST") {
        robotRunning = true;
        client.println("✓ Robot STARTED");
    }
    else if (cmd == "STOP" || cmd == "S") {
        robotRunning = false;
        motors.stopBrake();
        pid.reset();
        client.println("✓ Robot STOPPED");
    }
    else if (cmd == "RESET") {
        pid.reset();
        client.println("✓ PID Reset");
    }
    else if (cmd == "STATUS") {
        printStatus();
    }
    else if (cmd == "PID") {
        client.println("\n=== PID Values ===");
        client.print("Kp = ");
        client.println(pid.getKp(), 2);
        client.print("Ki = ");
        client.println(pid.getKi(), 4);
        client.print("Kd = ");
        client.println(pid.getKd(), 2);
        client.print("Output = ");
        client.println((int)pid.getOutput());
        client.println("==================\n");
    }
    else if (cmd == "MOTOR" || cmd == "PARAMS") {
        printMotorParams();
    }
    else if (cmd == "HELP" || cmd == "MENU") {
        printMenu();
    }
    else {
        client.println("✗ Unknown. Type HELP");
    }
}

void printMenu() {
    client.println("\n=== Commands ===");
    client.println("PID: KP/KI/KD [val]");
    client.println("Speed: BASE/TURN/MAX [val]");
    client.println("Encoder: TICKS90/CENTER [val]");
    client.println("Control: START/STOP/RESET");
    client.println("Info: STATUS/PID/MOTOR/HELP");
    client.println("================\n");
}

void printStatus() {
    client.println("\n=== Status ===");
    client.print("State: ");
    client.println(robotRunning ? "RUNNING" : "STOPPED");
    client.print("Error: ");
    client.println(sensors.getLineError());
    client.print("PID Out: ");
    client.println((int)pid.getOutput());
    client.print("On Line: ");
    client.println(sensors.onLine() ? "YES" : "NO");
    client.print("Path: ");
    client.print(rawPath);
    client.print(" (");
    client.print(pathIndex);
    client.println(")");
    client.println("==============\n");
}

void printMotorParams() {
    client.println("\n=== Motor Params ===");
    client.print("BASE = ");
    client.println(BASE_SPEED);
    client.print("TURN = ");
    client.println(TURN_SPEED);
    client.print("TICKS90 = ");
    client.println(TICKS_FOR_90_DEG);
    client.print("CENTER = ");
    client.println(TICKS_TO_CENTER);
    client.println("====================\n");
}