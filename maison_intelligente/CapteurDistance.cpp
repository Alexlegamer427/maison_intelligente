#include "CapteurDistance.h"

CapteurDistance::CapteurDistance(int trigPin, int echoPin)
  : hc(trigPin, echoPin) {
}

void CapteurDistance::update() {
  _currentTime = millis();
  if (_currentTime - _lastTime >= _rate) {
    _lastTime = _currentTime;
    _newDistance = hc.dist();
    if (_newDistance != 0) {
      _distance = _newDistance;
    }
  }
}

float CapteurDistance::getDistance() {
  return _distance;
}