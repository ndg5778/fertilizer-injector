#ifndef MAIN_HEADER_H
#define MAIN_HEADER_H

#include <Arduino.h>
#include <math.h>
#include <Adafruit_Sensor.h>
#include <EEPROM.h>
#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "DFRobot_PH.h"
#include "DFRobot_EC.h"
#include "DHT.h"

void taskPH_EC(void* pvParameters);
void taskDHT(void* pvParameters);
void taskStandardValuePH(void* pvParameters);
void taskStandardValueEC(void* pvParameters);
void taskSetupValue(void* pvParameters);
void taskPrintValue(void* pvParameters);

TaskHandle_t phEcTaskHandle;
TaskHandle_t dhtTaskHandle;
TaskHandle_t standardValuePHTaskHandle;
TaskHandle_t standardValueECTaskHandle;
TaskHandle_t setupValueTaskHandle;
TaskHandle_t printValueTaskHandle;

const uint16_t STACK_SIZE_PH_EC = 2048;
const uint16_t STACK_SIZE_DHT = 1024;
const uint16_t STACK_SIZE_VALUE = 1024;
const uint16_t STACK_SIZE_SETUP = 512;
const uint16_t STACK_SIZE_PRINT = 512;

const TickType_t PH_EC_DELAY = pdMS_TO_TICKS(1000);
const TickType_t DHT_DELAY = pdMS_TO_TICKS(10);
const TickType_t BTN_DELAY = pdMS_TO_TICKS(10);
const TickType_t PRINT_DELAY = pdMS_TO_TICKS(10);
const TickType_t MOTOR_DELAY = pdMS_TO_TICKS(10);

#define PH_PIN A0
#define EC_PIN A1
// #define DHTPIN 8
#define PH_UP_BTN 43
#define PH_DOWN_BTN 45
#define EC_UP_BTN 33
#define EC_DOWN_BTN 35
#define LED_PH_PIN 41
#define LED_EC_PIN 31
#define PHMotor_Relay 26
#define ECMotor_Relay 28
#define Pump_Motor_Relay 12

// #define EEPROM_SIZE 8
// #define EEPROM_PH 0
// #define EEPROM_EC 3
#define DHTTYPE DHT11 // DHT 11
DHT dht(DHTPIN, DHTTYPE);
LiquidCrystal_I2C lcd(0x27,16,2); 

float voltagePH, voltageEC, phValue, ecValue, temperature = 25;
float standardPH = 6.00, standardEC = 1.20;
bool state = LOW;
bool MotorOn = HIGH;
bool MotorOff = LOW;
bool PumpMotorOn = LOW;
bool PumpMotorOff = HIGH;
DFRobot_PH ph;
DFRobot_EC ec;

float readTemperature();


#endif
