#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFi.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1 //-1 inidicates no reset pin



void Initialize_SSD1306();
void DisplayDroneBattery();
void DisplayDroneAngle();
void DisplayThrottle();