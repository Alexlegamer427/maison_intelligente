#include "PorteAutomatique.h"
#include <Arduino.h>
#include <AccelStepper.h>





bool _moteurActive=false;
PorteAutomatique::PorteAutomatique(int p1, int p2, int p3, int p4, float& distanceRef)
  : _stepper(AccelStepper::HALF4WIRE, p1, p3, p2, p4), _distance(distanceRef) {
  _stepper.setMaxSpeed(800);
  _stepper.setAcceleration(400);
  _stepper.setCurrentPosition(_angleEnSteps(_angleFerme));
  _stepper.moveTo(_angleEnSteps(_angleFerme));
  _stepper.enableOutputs();
}

void PorteAutomatique::update() {
  _currentTime = millis();
  _mettreAJourEtat();

  switch (_etat) {
    case FERMEE: _fermeState(); break;
    case OUVERTE: _ouvertState(); break;
    case EN_OUVERTURE: _ouvertureState(); break;
    case EN_FERMETURE: _fermetureState(); break;
  }

  if (_moteurActive) {
  _stepper.run();
  }
}

void PorteAutomatique::setAngleOuvert(float angle) {
  _angleOuvert = angle;
}

void PorteAutomatique::setAngleFerme(float angle) {
  _angleFerme = angle;
}

void PorteAutomatique::setPasParTour(int steps) {
  _stepsPerRev = steps;
}

void PorteAutomatique::setDistanceOuverture(float distance) {
  _distanceOuverture = distance;
}

void PorteAutomatique::setDistanceFermeture(float distance) {
  _distanceFermeture = distance;
}

const char* PorteAutomatique::getEtatTexte() const {
  switch (_etat) {
    case FERMEE: return "Fermee";
    case OUVERTE: return "Ouverte";
    case EN_OUVERTURE: return "Ouverture";
    case EN_FERMETURE: return "Fermeture";
    default: return "Inconnu";
  }
}

float PorteAutomatique::getAngle() const {
  return _stepper.currentPosition() / (_stepsPerRev / 360.0);
}

void PorteAutomatique::_mettreAJourEtat() {
  if (_etat == FERMEE && _distance < _distanceOuverture && _distance > 0) {
    _stepper.moveTo(_angleEnSteps(_angleOuvert));   // Move motor ONCE here
    _etat = EN_OUVERTURE;
  } else if (_etat == OUVERTE && _distance > _distanceFermeture) {
    _stepper.moveTo(_angleEnSteps(_angleFerme));    // Move motor ONCE here
    _etat = EN_FERMETURE;
  }

  if (_etat == EN_OUVERTURE && _stepper.distanceToGo() == 0) {
    _etat = OUVERTE;
  } else if (_etat == EN_FERMETURE && _stepper.distanceToGo() == 0) {
    _etat = FERMEE;
  }
}

void PorteAutomatique::_ouvrir() {
  _stepper.enableOutputs();
  _moteurActive = true;
  Serial.println("Moteur Active");
}

void PorteAutomatique::_fermer() {
  _stepper.disableOutputs();
  _moteurActive = false;
  Serial.println("Moteur Stoppe");
}

void PorteAutomatique::_ouvertState() {
  // No action needed here
}

void PorteAutomatique::_fermeState() {
  // No action needed here
}

void PorteAutomatique::_ouvertureState() {
  // Just run the motor: moveTo already set in _mettreAJourEtat()
  // Avoid calling moveTo here to prevent reset
  // _stepper.run() is called in update()
}

void PorteAutomatique::_fermetureState() {
  // Just run the motor: moveTo already set in _mettreAJourEtat()
  // Avoid calling moveTo here to prevent reset
  // _stepper.run() is called in update()
}

long PorteAutomatique::_angleEnSteps(float angle) const {
  return static_cast<long>((angle / 360.0) * _stepsPerRev);
}


