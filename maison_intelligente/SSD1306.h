#pragma once    //a liam jali pris car pratique cas durgence

#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <Wire.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

extern unsigned long currentTime;

class SSD1306 {
private:
  uint8_t _address;
  Adafruit_SSD1306 _display;
  unsigned long lastTime = 0;
  int displayTime = 3000;

public:
  SSD1306(uint8_t address);

  void begin();

  void displaySuccess();

  void displayError();

  void displayUnknown();

  void update();
};