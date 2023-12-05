#pragma once

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>

#define OLED_RESET 4

extern Adafruit_SSD1306 display;

extern const unsigned char MrSonBitMap[];
void SSD1306_Setup();
void DrawMrSonBitMap();