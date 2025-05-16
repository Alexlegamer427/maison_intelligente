#pragma once

#include <LCD_I2C.h>

class Lcd {
public:

  Lcd();

  void begin();

  void update();

  void printL1(String msg);

  void printL2(String msg);

  void clear();

private:

LCD_I2C _lcd = LCD_I2C(0x27, 16, 2);

//int _lcdRate;

unsigned long _currentTime = 0;

unsigned long _lastTime = 0;

String _blank = "                ";

};