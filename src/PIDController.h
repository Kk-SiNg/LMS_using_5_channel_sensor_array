/*
 * PIDController.h
 * Wrapper class around QuickPID library for line following
 * Optimized for digital 5-sensor TCRT5000 array
 * 
 * WHY A WRAPPER?
 * ==============
 * 1. Encapsulates PID complexity
 * 2. Pre-configures QuickPID with optimal settings
 * 3. Provides WiFi tuning interface
 * 4. Makes main.cpp cleaner and more maintainable
 */

#pragma once
#include <QuickPID.h>
#include <Arduino.h>

class PIDController {
public:
    /*
     * Constructor
     * ===========
     * Parameters:
     * - kp: Proportional gain (how strongly to react to current error)
     * - ki: Integral gain (how strongly to react to accumulated error)
     * - kd: Derivative gain (how strongly to react to rate of change)
     * 
     * Typical values for digital 5-sensor array:
     * - Kp = 40-50   (higher than analog because digital has less precision)
     * - Ki = 0.1-1.0 (small, to correct steady-state drift)
     * - Kd = 20-30   (strong, to dampen oscillations)
     */
    PIDController(float kp, float ki, float kd);
    
    /*
     * Compute PID Output
     * ==================
     * Input: error value from sensors (-4 to +4 for 5 sensors)
     * Output: correction value to apply to motors
     * 
     * The QuickPID library automatically:
     * - Calculates P, I, D terms
     * - Applies anti-windup to integral
     * - Filters derivative to reduce noise
     * - Updates at configured sample rate
     */
    float compute(float error);
    
    /*
     * Update PID Constants (WiFi Tuning Support)
     * ===========================================
     * Allows live tuning of PID parameters without recompiling
     * Useful during competition setup and testing
     */
    void setTunings(float kp, float ki, float kd);
    
    /*
     * Reset PID State
     * ===============
     * Clears integral accumulation and derivative history
     * 
     * WHEN TO CALL:
     * - After stopping the robot
     * - After making a turn (to prevent integral windup from affecting next segment)
     * - When switching between run modes
     */
    void reset();
    
    /*
     * Set Output Limits
     * =================
     * Constrains PID output to valid motor speed range
     * Typically: -baseSpeed to +baseSpeed
     * 
     * Example: If baseSpeed = 150, limits are -150 to +150
     * This ensures motor speeds stay within 0-255 PWM range
     */
    void setOutputLimits(float min, float max);
    
    /*
     * Getters for WiFi Reporting
     * ==========================
     * Allow remote monitoring of PID state via telnet
     */
    float getKp() { return Kp; }
    float getKi() { return Ki; }
    float getKd() { return Kd; }
    float getOutput() { return pidOutput; }

private:
    // PID tuning constants
    float Kp, Ki, Kd;
    
    // PID working variables (required by QuickPID)
    float pidInput;      // Current error value
    float pidOutput;     // Calculated correction
    float pidSetpoint;   // Target error (always 0 for line following)
    
    // QuickPID library object
    QuickPID* pid;
};