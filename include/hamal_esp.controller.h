#pragma once
#define USE_HWCDC // For ESP32-S3
#include <SPI.h>
#include "MCP2515.h"

#include <string.h>
#include <ros.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/BatteryState.h>
#include <hamal_led_controller/Led.h>  

#include "FastLED.h"

#define CS_PIN 10
#define INT_PIN 6

#define NUM_LEDS 400
#define LED_PIN 37
#define BRIGHTNESS 100

struct TPDO1 {
  uint8_t bsc; // Battery state of charge
  uint8_t bmc; // BMS state of charge
  uint8_t bmf; // BMS status flags
  uint8_t bms; // BMS operational state
};

extern TPDO1 tpdo1;
extern CRGB leds[NUM_LEDS];
extern ros::NodeHandle nh;

// Function declarations
void fadeInOut(int firstCounter, int lastCounter, int multiplier, bool colors[], int stepDelay, int mode);
void showStrip();
void lights_off();
void setPixel(int Pixel, byte red, byte green, byte blue);
void setAll(byte red, byte green, byte blue);
void setRight(byte red, byte green, byte blue);
void setLeft(byte red, byte green, byte blue);
void CANBus_BMS(void* parameter);
void led_controller(void* parameter);
void idleMode();
void taskMode();
void warningMode();
void switchMode();
void goingForward(byte red, byte green, byte blue, int waveDelay);
void goingBackward(byte red, byte green, byte blue, int waveDelay);
void turningRight(byte red, byte green, byte blue, int waveDelay);
void turningLeft(byte red, byte green, byte blue, int waveDelay);
void fadeRight(int firstCounter, int lastCounter, int multiplier, bool colors[], int stepDelay, int mode);
void fadeLeft(int firstCounter, int lastCounter, int multiplier, bool colors[], int stepDelay, int mode);
void publishBMSStatus();