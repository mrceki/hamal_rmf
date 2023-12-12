/**
 * @file hamal_esp_controller.cpp
 * @brief This file contains the implementation of the hamal_esp_controller module.
 * 
 * This module is responsible for controlling the LED lights and communicating with the BMS (Battery Management System) via CAN bus.
 * It subscribes to the "led_status" topic to receive commands for controlling the LED lights and publishes the battery state information to the "battery_state" topic.
 * The LED lights can be controlled in different modes such as idle mode, task mode, going forward, going backward, turning right, and turning left.
 * The module uses the FastLED library to control the WS2812 LED strip.
 * The CAN communication is handled using the MCP2515 CAN controller.
 * 
 * @author Cenk Cetin
 */

#include "hamal_esp.controller.h"

ros::NodeHandle nh;

void light_msg_subCB(const hamal_led_controller::Led& msg);
ros::Subscriber<hamal_led_controller::Led> light_msg_sub("led_status", light_msg_subCB);

sensor_msgs::BatteryState battery_state_msg;
ros::Publisher battery_state_pub("battery_state", &battery_state_msg);

int currentMode = -1;
int currentSpeed = 100;

MCP2515 CAN(CS_PIN, INT_PIN);

TPDO1 tpdo1;
CRGB leds[NUM_LEDS];

uint8_t i = 0;
Frame message;
Frame message1;
uint8_t TX_Array[50];
int Iterator = 0;

void setup() {
  nh.initNode();
  nh.subscribe(light_msg_sub);
  nh.advertise(battery_state_pub);
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(BRIGHTNESS);

  SPI.setClockDivider(SPI_CLOCK_DIV2);
  SPI.setDataMode(SPI_MODE3);
  SPI.setBitOrder(MSBFIRST);
  SPI.begin();

  // Initialise MCP2515 CAN controller at the specified speed and clock frequency
  // In this case 250kbps with a 8MHz oscillator
  // (Note:  This is the oscillator attached to the MCP2515, not the Arduino oscillator)
  int baudRate = CAN.Init(250, 8);
  if (baudRate > 0) {
    Serial.println("MCP2515 Init OK ...");
    Serial.print("Baud Rate (kbps): ");
    Serial.println(baudRate, DEC);
  } else {
    Serial.println("MCP2515 Init Failed ...");
    // Implement error handling here
    while (1);  // Halt the program in case of initialization failure
  }
  xTaskCreatePinnedToCore(
    CANBus_BMS,   /* Task function. */
    "Canbus Task",     /* name of task. */
    10000,       /* Stack size of task */
    NULL,        /* parameter of the task */
    1,           /* priority of the task */
    NULL,        /* Task handle to keep track of created task */
    0);          /* pin task to core 0 */                  
  delay(500); 

  xTaskCreatePinnedToCore(
    led_controller,   /* Task function. */
    "Led Controller Task",     /* name of task. */
    10000,       /* Stack size of task */
    NULL,        /* parameter of the task */
    1,           /* priority of the task */
    NULL,        /* Task handle to keep track of created task */
    1);          /* pin task to core 1 */
  delay(500); 

  Serial.println("Ready ...");
}

void light_msg_subCB(const hamal_led_controller::Led& msg) {
  if (msg.mode != currentMode) {
    currentMode = msg.mode;
  }
  if(msg.speed == 0){
    currentSpeed = 30;
  }
  else if(msg.speed > 100){
    currentSpeed = 100;
  }
  else{
    currentSpeed = map(msg.speed, 0, 100, 20, 1);
  }
}

void switchMode(int mode) {

  switch (mode) {
    case 0:
      idleMode();
      break;
    case 1:
      taskMode();
      break;
    case 2:
      warningMode();
      break;
    case 3:
      goingForward(0, 255, 255, int(100/currentSpeed));
      break;
    case 4:
      goingBackward(0, 255, 255, int(100/currentSpeed));
      break;
    case 5:
      turningRight(0, 255, 0, int(100/currentSpeed));
      break;
    case 6:
      turningLeft(0, 255, 0, int(100/currentSpeed));
      break;
  }
}

void goingForward(byte red, byte green, byte blue, int waveDelay) {
  // Turn off all LEDs
  lights_off();

  // Light up LEDs in forward direction
  for(uint16_t i = 0; i < NUM_LEDS/2; i+=2) {
    setPixel(i, red, green, blue);
    setPixel(NUM_LEDS - i, red, green, blue);

    showStrip();
    delay(waveDelay);

    // Update ROS node every 10 LEDs
    if(i%10 == 0){
      nh.spinOnce();
    }

    // Log current speed
    char logMsg[50];
    sprintf(logMsg, "Speed is %d", waveDelay);
    nh.loginfo(logMsg);

    // Update speed
    waveDelay = currentSpeed;

    // Break if mode changes
    if (currentMode == 2) {
      break;
    }
  }

  if (currentMode != 3) {
    return;
  }

  // Turn off LEDs in forward direction
  for(uint16_t i = 0; i < NUM_LEDS/2; i+=2) {
    setPixel(i, 0, 0, 0);
    setPixel(NUM_LEDS - i, 0, 0, 0);

    showStrip();
    delay(waveDelay);

    // Update ROS node every 10 LEDs
    if(i%10 == 0){
      nh.spinOnce();
    }

    // Log current speed
    char logMsg[50];
    sprintf(logMsg, "Speed is %d", waveDelay);
    nh.loginfo(logMsg);

    waveDelay = currentSpeed;

    // Break if mode changes
    if (currentMode == 2) {
      break;
    }
  }

  if (currentMode != 3) {
    return;
  }
}

void goingBackward(byte red, byte green, byte blue, int waveDelay) {
  // Turn off all LEDs
  lights_off();

  // Light up LEDs in backward direction
  for(uint16_t i = NUM_LEDS/2; i < NUM_LEDS; i+=2) {
    setPixel(i, red, green, blue);
    setPixel(NUM_LEDS - i, red, green, blue);
    showStrip();
    delay(waveDelay);

    // Update ROS node every 10 LEDs
    if(i%10 == 0){
      nh.spinOnce();
    }

    // Log current speed
    char logMsg[50];
    sprintf(logMsg, "Speed is %d", waveDelay);
    nh.loginfo(logMsg);

    // Update speed
    waveDelay = currentSpeed;

    // Break if mode changes
    if (currentMode == 2) {
      break;
    }
  }

  if (currentMode != 4) {
    return;
  }
  
    // Light up LEDs in backward direction
  for(uint16_t i = NUM_LEDS/2; i < NUM_LEDS; i+=2) {
    setPixel(i, 0, 0, 0);
    setPixel(NUM_LEDS - i, 0, 0, 0);
    showStrip();
    delay(waveDelay);

    // Update ROS node every 10 LEDs
    if(i%10 == 0){
      nh.spinOnce();
    }

    // Log current speed
    char logMsg[50];
    sprintf(logMsg, "Speed is %d", waveDelay);
    nh.loginfo(logMsg);

    // Update speed
    waveDelay = currentSpeed;

    // Break if mode changes
    if (currentMode == 2) {
      break;
    }
  }
  if (currentMode != 4) {
    return;
  }
}

void turningRight(byte red, byte green, byte blue, int waveDelay) {
  bool colors[3] = {0,1,1};
  fadeRight(3, 75, 3, colors, 1, 6);
}

void turningLeft(byte red, byte green, byte blue, int waveDelay) {
  bool colors[3] = {0,1,1};
  fadeLeft(3, 75, 3, colors, 1, 5);
}

void fadeInOut(int firstCounter, int lastCounter, int multiplier, bool colors[], int stepDelay, int mode){
  // Fade IN
  for (int k = firstCounter; k < lastCounter; k++) {
    if (k % 50 == 0) {
      nh.spinOnce();
      if (currentMode != mode) {
        break;
      }
    }
    setAll(k * colors[0] * multiplier, k * colors[1] * multiplier, k * colors[2]*  multiplier);
    vTaskDelay(stepDelay);
  }

  if (currentMode != mode) {
    return;
  }

  // Fade OUT
  for (int k = lastCounter -1; k >= firstCounter; k--) {
    if (k % 50 == 0) {
      nh.spinOnce();
      if (currentMode != mode) {
        break;
      }
    }
    setAll(k * colors[0] * multiplier, k * colors[1] * multiplier, k * colors[2]*  multiplier);
    vTaskDelay(stepDelay);
  }
  
  if (currentMode != mode) {
    return;
  }
}

void fadeRight(int firstCounter, int lastCounter, int multiplier, bool colors[], int stepDelay, int mode){
  // Fade IN
  for (int k = firstCounter; k < lastCounter/2; k++) {
    
    nh.spinOnce();
    
    if (currentMode == 2) {
      break;
    }

    setRight(k * colors[0] * multiplier, k * colors[1] * multiplier, k * colors[2]*  multiplier);
    vTaskDelay(stepDelay);
  }

  if (currentMode != mode) {
    return;
  }

  // Fade OUT
  for (int k = lastCounter -1/2; k >= firstCounter; k--) {
    if (k % 50 == 0) {
      nh.spinOnce();
      if (currentMode == 2) {
        break;
      }
    }
    setRight(k * colors[0] * multiplier, k * colors[1] * multiplier, k * colors[2]*  multiplier);
    vTaskDelay(stepDelay);
  }
  if (currentMode != mode) {
    return;
  }
}

void fadeLeft(int firstCounter, int lastCounter, int multiplier, bool colors[], int stepDelay, int mode){
  // Fade IN
  for (int k = firstCounter; k < lastCounter/2; k++) {
    if (k % 50 == 0) {
      nh.spinOnce();
      if (currentMode == 2) {
        break;
      }
    }
    setLeft(k * colors[0] * multiplier, k * colors[1] * multiplier, k * colors[2]*  multiplier);
    vTaskDelay(stepDelay);
  }
  if (currentMode != mode) {
    return;
  }
  // Fade OUT
  for (int k = lastCounter -1/2; k >= firstCounter; k--) {
    if (k % 50 == 0) {
      nh.spinOnce();
      if (currentMode == 2) {
        break;
      }
    }
    setLeft(k * colors[0] * multiplier, k * colors[1] * multiplier, k * colors[2]*  multiplier);
    vTaskDelay(stepDelay);
  }
  if (currentMode != mode) {
    return;
  }
}

void idleMode() {
  bool colors[3] = {0,1,0};
  fadeInOut(10, 256, 1, colors, 1, 0);
}

void taskMode() {
  bool colors[3] = {1,1,0};
  fadeInOut(30, 128, 2, colors, 1, 1);
}

void led_controller(void* parameter) {
  for (;;) {
  switchMode(currentMode);
  nh.spinOnce();
  vTaskDelay(1);
  }
}

void publishBMSStatus(uint8_t bsc, uint8_t bmc, uint8_t bmf, uint8_t bms) {

  battery_state_msg.percentage = (float(bsc))/100;
  battery_state_msg.design_capacity = 102;
  battery_state_msg.charge = bmc;
  battery_state_msg.power_supply_technology = sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_LIFE;
  battery_state_msg.present = true;

  if (bmf==0){
    battery_state_msg.power_supply_health = sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
  }
  else{
    battery_state_msg.power_supply_health = sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
  }

  
  if (bms==23){
    battery_state_msg.power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_CHARGING;
  }
  else if (bms==19){
    battery_state_msg.power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
  }
  else{
    battery_state_msg.power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
  }

  


  battery_state_pub.publish(&battery_state_msg);
}

void CANBus_BMS(void* parameter) {
  for(;;){
    message.id = 0;
    if (CAN.Interrupt()) {
      uint8_t interruptFlags = CAN.Read(CANINTF);
      
      if (interruptFlags & RX0IF) {
        message = CAN.ReadBuffer(RXB0);
      }
      if (interruptFlags & RX1IF) {
        message = CAN.ReadBuffer(RXB1);
      }
      if (interruptFlags & TX0IF) {
        // TX buffer 0 sent
      }
      if (interruptFlags & TX1IF) {
        // TX buffer 1 sent
      }
      if (interruptFlags & TX2IF) {
        // TX buffer 2 sent
      }
      if (interruptFlags & ERRIF) {
        // error handling code
      }
      if (interruptFlags & MERRF) {
        // error handling code
      }
    }
  
    if (message.id > 0) {
      if (message.id == 0x186) {
        
        tpdo1.bsc = message.data[0];
        tpdo1.bmc = message.data[1];
        tpdo1.bmf = message.data[2];
        tpdo1.bms = message.data[3];
  
        Serial.print("ID: ");
        Serial.println(message.id, HEX);
        Serial.print("Extended: ");
        if (message.ide) {
          Serial.println("Yes");
        } else {
          Serial.println("No");
        }
        Serial.print("DLC: ");
        Serial.println(message.dlc, DEC);
        Serial.print("Battery state of charge: %");
        Serial.print(tpdo1.bsc);
        Serial.println("BMS state of charge: ");
        Serial.print(tpdo1.bmc);
        Serial.println(" Ah");
        Serial.print("BMS status flags: ");
        Serial.println(tpdo1.bmf, DEC);
        Serial.print("BMS operational state: ");
        Serial.println(tpdo1.bms, DEC);
        publishBMSStatus(tpdo1.bsc, tpdo1.bmc, tpdo1.bmf, tpdo1.bms);
      }
  
      message.id++;
      for (i = 0; i < message.dlc; i++) {
        message.data[i]++;
      }
    }  
    vTaskDelay(100);
  }
}

void loop() {
  // Empty. Tasks are handled by FreeRTOS.
}
