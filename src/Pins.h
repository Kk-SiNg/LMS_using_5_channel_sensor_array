/*
 * Pins.h
 * Hardware pin definitions for ESP32 with 5-channel TCRT5000 sensor array
 * Optimized for IIT Bombay Mesmerize Competition
 * 
 * Pin Selection Rules:
 * - GPIO 34,35,36,39 are INPUT-ONLY (perfect for digital sensors)
 * - Safe PWM pins: GPIO 25,26
 * - Safe digital I/O: GPIO 21,22,23,19
 * - Encoder pins need interrupt capability: GPIO 18,5,17,16
 */

#pragma once
#include <cstdint>

// === SENSOR ARRAY (TCRT5000 5-Channel Digital) ===
// Digital sensors: HIGH = line detected (black), LOW = no line (white)
#define SENSOR_PIN_1 36  // Right-most sensor (INPUT ONLY - OK for digital read)
#define SENSOR_PIN_2 39  // Right sensor (INPUT ONLY - OK for digital read)
#define SENSOR_PIN_3 34  // Center sensor (INPUT ONLY - OK for digital read)
#define SENSOR_PIN_4 35  // Left sensor (INPUT ONLY - OK for digital read)
#define SENSOR_PIN_5 32  // Left-most sensor (Safe bidirectional pin)

const uint8_t SensorCount = 5;

// === MOTOR CONTROL (L298N) ===
// Left Motor (Motor A)
#define MOTOR_L_IN1 15   // Direction control 1
#define MOTOR_L_IN2 4   // Direction control 2
#define MOTOR_L_ENA 25   // PWM Speed Control

// Right Motor (Motor B)
#define MOTOR_R_IN3 19   // Direction control 1
#define MOTOR_R_IN4 21   // Direction control 2
#define MOTOR_R_ENB 26   // PWM Speed Control

// === ENCODERS (N-20 Motors) ===
// Left Encoder
#define ENCODER_L_A 18   // Encoder A phase
#define ENCODER_L_B 5    // Encoder B phase

// Right Encoder
#define ENCODER_R_A 17   // Encoder A phase
#define ENCODER_R_B 16   // Encoder B phase

// === MISC ===
#define ONBOARD_LED 2    // Built-in LED
#define USER_BUTTON 15    // Boot button (shared with MOTOR_L_IN1, but INPUT_PULLUP mode during wait states)

// === WIFI CONFIGURATION ===
#define WIFI_SSID "Redmi A2"
#define WIFI_PASSWORD "kvsandkks"
#define TELNET_PORT 23