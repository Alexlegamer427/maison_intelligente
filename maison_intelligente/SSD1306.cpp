#include "SSD1306.h" //a liam jali pris car pratique cas durgence

SSD1306::SSD1306(uint8_t address)
  : _address(address), _display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET) {}

void SSD1306::begin() {
  if (!_display.begin(SSD1306_SWITCHCAPVCC, _address)) {
    while (true)
      ;
  }
  _display.clearDisplay();
  _display.display();
}

void SSD1306::displaySuccess() {
  _display.clearDisplay();
  _display.drawLine(20, 20, 50, 50, SSD1306_WHITE);
  _display.drawLine(50, 50, 100, 0, SSD1306_WHITE);
  _display.display();
  lastTime = currentTime;
}

void SSD1306::displayError() {
  _display.clearDisplay();
  _display.drawCircle(64, 32, 20, SSD1306_WHITE);
  _display.drawLine(50, 18, 78, 46, SSD1306_WHITE);
  _display.display();
  lastTime = currentTime;
}

void SSD1306::displayUnknown() {
  _display.clearDisplay();
  _display.drawLine(49, 17, 79, 47, SSD1306_WHITE);
  _display.drawLine(49, 47, 79, 17, SSD1306_WHITE);
  _display.display();
  lastTime = currentTime;
}

void SSD1306::update() {
  if (currentTime - lastTime >= displayTime) {
    lastTime = currentTime;
    _display.clearDisplay();
    _display.display();
  }
}