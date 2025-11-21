/*
 * PIDController.cpp
 * Implementation of PID wrapper with QuickPID library
 * Configured optimally for ESP32 line following robot
 */

#include "PIDController.h"

/*
 * ====================================================================
 * CONSTRUCTOR - Initialize QuickPID with Optimal Settings
 * ====================================================================
 */
PIDController::PIDController(float kp, float ki, float kd) 
    : Kp(kp), Ki(ki), Kd(kd), pidInput(0), pidOutput(0), pidSetpoint(0) {
    
    /*
     * CREATE QuickPID INSTANCE
     * ========================
     * 
     * Parameters:
     * - &pidInput: Pointer to input variable (sensor error)
     * - &pidOutput: Pointer to output variable (motor correction)
     * - &pidSetpoint: Pointer to setpoint (always 0 for line following)
     * - Kp, Ki, Kd: Tuning constants
     * - QuickPID::Action::direct: Direct action (increase output for positive error)
     * 
     * NOTE: QuickPID modifies pidOutput automatically when Compute() is called
     */
    pid = new QuickPID(&pidInput, &pidOutput, &pidSetpoint, 
                       Kp, Ki, Kd, 
                       QuickPID::Action::direct);
    
    /*
     * CONFIGURE QuickPID MODE
     * =======================
     * 
     * SetMode(automatic):
     * - Enables automatic PID computation
     * - PID calculates output every time Compute() is called
     * - Alternative: manual mode (user controls when PID runs)
     */
    pid->SetMode(QuickPID::Control::automatic);
    
    /*
     * CONFIGURE SAMPLE TIME
     * =====================
     * 
     * SetSampleTimeUs(10000):
     * - Sets PID update rate to 10,000 microseconds = 10ms
     * - This means PID runs at 100Hz
     * - Fast enough for line following (typical: 50-100Hz)
     * - Too fast (<5ms) = noisy, Too slow (>50ms) = sluggish
     * 
     * WHY 10ms?
     * - ESP32 can easily handle 100Hz updates
     * - Matches typical motor PWM frequency (5kHz)
     * - Fast enough to respond to line curves
     * - Slow enough to avoid noise amplification
     */
    pid->SetSampleTimeUs(10000);
    
    /*
     * CONFIGURE PROPORTIONAL MODE
     * ===========================
     * 
     * SetProportionalMode(pOnMeas):
     * - "Proportional on Measurement" mode
     * - Calculates P term based on change in measurement, not setpoint
     * 
     * BENEFIT:
     * - Reduces "derivative kick" when setpoint changes
     * - Smoother response to sudden errors
     * - Better for our use case (setpoint never changes)
     * 
     * ALTERNATIVE:
     * - pOnError: Traditional P term (proportional to error)
     * - We use pOnMeas for smoother motor control
     */
    pid->SetProportionalMode(QuickPID::pMode::pOnMeas);
    
    /*
     * CONFIGURE DERIVATIVE MODE
     * =========================
     * 
     * SetDerivativeMode(dOnMeas):
     * - "Derivative on Measurement" mode
     * - Calculates D term based on change in sensor reading
     * 
     * BENEFIT:
     * - Eliminates "derivative kick" on setpoint changes
     * - Reduces noise amplification from sensor readings
     * - Smoother motor control (no sudden jerks)
     * 
     * WHY IMPORTANT FOR DIGITAL SENSORS?
     * - Digital sensors have discrete values (0 or 1)
     * - Derivative can be noisy with sudden transitions
     * - dOnMeas mode filters this noise internally
     */
    pid->SetDerivativeMode(QuickPID::dMode::dOnMeas);
    
    /*
     * CONFIGURE ANTI-WINDUP MODE
     * ==========================
     * 
     * SetAntiWindupMode(iAwClamp):
     * - "Integral Anti-Windup Clamp" mode
     * - Prevents integral term from accumulating when output is saturated
     * 
     * THE WINDUP PROBLEM:
     * ==================
     * Imagine robot is stuck against a wall:
     * 1. Error is large and constant
     * 2. Integral keeps accumulating: error × time
     * 3. Integral becomes HUGE (thousands)
     * 4. When obstacle removed, robot overshoots badly
     * 5. Takes long time to "unwind" the integral
     * 
     * ANTI-WINDUP SOLUTION:
     * =====================
     * - When output hits limits (min/max), stop accumulating integral
     * - Prevents integral from growing unbounded
     * - Robot responds normally when obstacle cleared
     * 
     * CLAMPING METHOD (iAwClamp):
     * - Most common and effective
     * - Simply stops integral accumulation at output limits
     * 
     * ALTERNATIVES:
     * - iAwCondition: Conditional integration
     * - iAwOff: No anti-windup (not recommended)
     */
    pid->SetAntiWindupMode(QuickPID::iAwMode::iAwClamp);
    
    /*
     * SET INITIAL OUTPUT LIMITS
     * =========================
     * 
     * Default: -255 to +255 (full PWM range)
     * Will be updated by setOutputLimits() to match base speed
     * 
     * Example: If baseSpeed = 150
     * - Output limited to -150 to +150
     * - Motor speeds: (150-150) to (150+150) = 0 to 300
     * - Then constrained again to 0-255 in setSpeeds()
     */
    pid->SetOutputLimits(-255, 255);
}

/*
 * ====================================================================
 * COMPUTE - Calculate PID Output
 * ====================================================================
 */
float PIDController::compute(float error) {
    /*
     * HOW IT WORKS:
     * =============
     * 
     * 1. Update pidInput with current sensor error
     *    Example: error = -2 (line is to the right)
     * 
     * 2. Call QuickPID's Compute() method
     *    - Calculates: output = Kp×error + Ki×∫error + Kd×(Δerror/Δt)
     *    - Applies anti-windup
     *    - Filters derivative
     *    - Updates pidOutput automatically
     * 
     * 3. Return the calculated correction
     * 
     * EXAMPLE CALCULATION:
     * ====================
     * Assume: Kp=45, Ki=0.5, Kd=25
     * Current error = -2 (line to the right)
     * Previous error = -1
     * Time since last call = 10ms = 0.01s
     * Accumulated integral = 0.5
     * 
     * P term = 45 × (-2) = -90
     * I term = 0.5 × (0.5 + (-2 × 0.01)) = 0.5 × 0.48 = 0.24
     * D term = 25 × ((-2) - (-1)) / 0.01 = 25 × (-100) = -2500... but filtered!
     * 
     * Total output ≈ -90 + 0.24 - 25 = -114.76
     * 
     * Motor speeds:
     * Left = 150 - (-115) = 265 → constrained to 255
     * Right = 150 + (-115) = 35
     * 
     * Result: Robot turns right to follow the line!
     */
    pidInput = error;
    pid->Compute();
    return pidOutput;
}

/*
 * ====================================================================
 * SET TUNINGS - Update PID Constants (WiFi Tuning)
 * ====================================================================
 */
void PIDController::setTunings(float kp, float ki, float kd) {
    /*
     * UPDATE TUNING CONSTANTS
     * =======================
     * 
     * Can be called at runtime via WiFi commands
     * Example: "KP 50" changes Kp to 50.0
     * 
     * QuickPID's SetTunings() updates internal calculations
     * without resetting integral or derivative terms
     * 
     * WHEN TO TUNE:
     * =============
     * - Oscillation (too much swaying) → Decrease Kp, Increase Kd
     * - Sluggish response → Increase Kp
     * - Steady-state error (drifts off line) → Increase Ki
     * - Overshoot on curves → Increase Kd
     */
    Kp = kp;
    Ki = ki;
    Kd = kd;
    pid->SetTunings(Kp, Ki, Kd);
}

/*
 * ====================================================================
 * RESET - Clear PID State
 * ====================================================================
 */
void PIDController::reset() {
    /*
     * WHAT GETS RESET:
     * ================
     * 1. Integral accumulator → 0
     * 2. Previous error (for derivative) → 0
     * 3. Last sample time → current time
     * 4. Output → 0
     * 
     * WHY RESET?
     * ==========
     * Scenario 1: After a turn
     * - During turn, error was large and to the left
     * - Integral accumulated significantly
     * - After turn, line is centered but integral is still large
     * - Without reset: robot overshoots to compensate for old error
     * - With reset: robot starts fresh with clean slate
     * 
     * Scenario 2: After stopping
     * - Robot was correcting for error when stopped
     * - Integral and derivative have values from that error
     * - On restart, old values cause initial jerk
     * - Reset prevents this
     * 
     * WHEN TO CALL:
     * =============
     * ✓ After turn_90_left()
     * ✓ After turn_90_right()
     * ✓ After turn_180_back()
     * ✓ When switching from MAPPING to SOLVING
     * ✓ When stopping robot (STOP command)
     * ✗ NOT during normal line following (defeats purpose of I and D terms)
     */
    pid->Reset();
    pidInput = 0;
    pidOutput = 0;
}

/*
 * ====================================================================
 * SET OUTPUT LIMITS - Constrain PID Output Range
 * ====================================================================
 */
void PIDController::setOutputLimits(float min, float max) {
    /*
     * PURPOSE:
     * ========
     * Prevent PID from requesting impossible motor speeds
     * 
     * LOGIC:
     * ======
     * If baseSpeed = 150:
     * - Correction range: -150 to +150
     * - Left motor: 150 + (-150 to +150) = 0 to 300
     * - Right motor: 150 - (-150 to +150) = 0 to 300
     * - Both constrained to 0-255 in Motors::setSpeeds()
     * 
     * WHY NOT JUST USE -255 TO +255?
     * ==============================
     * - If baseSpeed = 100, correction of 200 would give:
     *   Left = 100 + 200 = 300 (invalid!)
     * - Better to limit correction to ±baseSpeed
     * 
     * UPDATES AUTOMATICALLY:
     * =====================
     * When WiFi command changes BASE speed:
     * 1. BASE_SPEED global variable updated
     * 2. Call setOutputLimits(-BASE_SPEED, BASE_SPEED)
     * 3. PID correction now scaled to new base speed
     */
    pid->SetOutputLimits(min, max);
}