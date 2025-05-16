#include "Alarm.h"
#include <Arduino.h>

// Constructeur

// rPin, gPin, bPin : broches pour la DEL RGB

// buzzerPin : broche du buzzer

// distancePtr : pointeur vers la variable de distance partagée

Alarm::Alarm(int rPin, int gPin, int bPin, int buzzerPin, float* distancePtr) {
  _rPin = rPin;
  _gPin = gPin;
  _bPin = bPin;
  _buzzerPin = buzzerPin;
  _distance = distancePtr;
  pinMode(_rPin, OUTPUT);
  pinMode(_gPin, OUTPUT);
  pinMode(_bPin, OUTPUT);
  pinMode(_buzzerPin, OUTPUT);
  setColourA(255, 0, 0);
  setColourB(0, 0, 255);
  _setRGB(0, 0, 0);
  digitalWrite(_buzzerPin, LOW);
}

// Méthode à appeler continuellement dans loop()

void Alarm::update() {

  _currentTime = millis();

  if (_turnOnFlag) {
    _turnOnFlag = false;
    _state = WATCHING;
  } else if (_turnOffFlag) {
    _turnOffFlag = false;
    _state = OFF;
  }

  switch (_state) {
    case OFF: _offState(); break;
    case WATCHING: _watchState(); break;
    case ON: _onState(); break;
    case TESTING: _testingState(); break;
  }
}


// Régle les deux couleurs du gyrophare

void Alarm::setColourA(int r, int g, int b) {
  _colA[0] = r;
  _colA[1] = g;
  _colA[2] = b;
}

void Alarm::setColourB(int r, int g, int b) {
  _colB[0] = r;
  _colB[1] = g;
  _colB[2] = b;
}


// Régle la fréquence de variation du gyrophare (en ms)

void Alarm::setVariationTiming(unsigned long ms) {
  _variationRate = ms;
}


// Régle la distance de déclenchement (en cm)

void Alarm::setDistance(float d) {
  _distanceTrigger = d;
}


// Régle le délai avant l'extinction après éloignement (en ms)

void Alarm::setTimeout(unsigned long ms) {
  _timeoutDelay = ms;
}


// Éteint/Allume l'alarme manuellement

void Alarm::turnOff() {
  _turnOffFlag = true;
}

void Alarm::turnOn() {
  _turnOnFlag = true;
}

// Déclenche un test de 3 secondes

void Alarm::test() {
  _state = TESTING;
  _testStartTime = _currentTime;
}


// Retourne l'état courant de l'alarme

AlarmState Alarm::getState() const {
  return _state;
}

void Alarm::_setRGB(int r, int g, int b) {
  analogWrite(_rPin, r);
  analogWrite(_gPin, g);
  analogWrite(_bPin, b);
}

void Alarm::_turnOff() {
  _setRGB(0, 0, 0);
  digitalWrite(_buzzerPin, LOW);
}  // Éteint DEL et buzzer


// Gestion des états

void Alarm::_offState() {
  _turnOff();
}

void Alarm::_watchState() {
  _turnOff();
  if (*_distance <= _distanceTrigger) {
    _state = ON;
    _lastDetectedTime = _currentTime;
  }
}

void Alarm::_onState() {
  digitalWrite(_buzzerPin, HIGH);
  if (_currentTime - _lastUpdate >= _variationRate) {
    _lastUpdate = _currentTime;
    _currentColor = !_currentColor;
    if (_currentColor) {
      _setRGB(_colA[0], _colA[1], _colA[2]);
    } else {
      _setRGB(_colB[0], _colB[1], _colB[2]);
    }
  }
  if (*_distance <= _distanceTrigger) {
    _lastDetectedTime = _currentTime;
  }
  if (*_distance > _distanceTrigger && (_currentTime - _lastDetectedTime >= _timeoutDelay)) {
    _state = WATCHING;
  }
}

void Alarm::_testingState() {
  digitalWrite(_buzzerPin, HIGH);
  if (_currentTime - _lastUpdate >= _variationRate) {
    _lastUpdate = _currentTime;
    _currentColor = !_currentColor;
    if (_currentColor) {
      _setRGB(_colA[0], _colA[1], _colA[2]);
    } else {
      _setRGB(_colB[0], _colB[1], _colB[2]);
    }
  }
  if (_currentTime - _testStartTime >= _timeoutDelay) {
    _state = WATCHING;
  }
}