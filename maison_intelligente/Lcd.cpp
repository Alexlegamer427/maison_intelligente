#include "Lcd.h"

Lcd::Lcd() {
}

void Lcd::begin() {
  _lcd.begin();
  _lcd.backlight();
  _lcd.clear();
}

void Lcd::update() {
 
}

void Lcd::printL1(String msg) {
  _lcd.setCursor(0, 0);
  _lcd.print(_blank);
  _lcd.setCursor(0, 0);
  _lcd.print(msg);
}

void Lcd::printL2(String msg) {
  _lcd.setCursor(0, 1);
  _lcd.print(_blank);
  _lcd.setCursor(0, 1);
  _lcd.print(msg);
}

void Lcd::clear() {
  _lcd.clear();
}