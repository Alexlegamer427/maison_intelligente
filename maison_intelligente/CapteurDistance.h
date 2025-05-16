#pragma once

#include <HCSR04.h>

class CapteurDistance {
public:

  CapteurDistance(int trigPin, int echoPin);

  void update();

  float getDistance();

private:

  HCSR04 hc;

  unsigned long _currentTime = 0;
  unsigned long _lastTime = 0;
  int _rate = 50;

  float _distance;
  float _newDistance;
};