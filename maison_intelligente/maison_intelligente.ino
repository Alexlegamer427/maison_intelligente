#include <HCSR04.h>
#include <LCD_I2C.h>
#include <AccelStepper.h>


LCD_I2C lcd(0x27, 16, 2);
#define MOTOR_INTERFACE_TYPE 4
#define TRIGGER_PIN 3  // Capteur distance pin
#define ECHO_PIN 2     // Capteur distance pin
#define IN_1 10        //pins du accel stepper
#define IN_2 9
#define IN_3 7
#define IN_4 6
#define buzzer 4

AccelStepper myStepper(MOTOR_INTERFACE_TYPE, IN_1, IN_3, IN_2, IN_4);  //une révolution =2038 pas
float degres;
int distance;
int currentStep;
int minStep = 2038 * (170.0 / 360.0);
int maxStep = 2038 * (10.0 / 360.0);
int setupPrintDelay = 2000;
int off = LOW;

class led_rgb {
private:
  int redLed = 11;
  int blueLed = 5;
  int greenLed = 8;

public:
  void setColor(int redReplace, int greenReplace, int blueReplace) {

    analogWrite(redLed, redReplace);
    analogWrite(greenLed, greenReplace);
    analogWrite(blueLed, blueReplace);
  }

  void off_led() {

    setColor(0, 0, 0);
  }
};
led_rgb RGB;
enum Alarm { rClose,
             far };
Alarm alarm;

enum Porte { Ouverture,
             Ouvert,
             Fermeture,
             Fermer };
Porte etat;
HCSR04 hc(TRIGGER_PIN, ECHO_PIN);

void setup() {
  Serial.begin(9600);
  pinMode(buzzer, OUTPUT);
  myStepper.setMaxSpeed(500);
  myStepper.setAcceleration(300);

  lcd.begin();
  lcd.backlight();
  lcd.print("6308958");
  lcd.setCursor(0, 1);
  lcd.print("Labo 4A");
  delay(setupPrintDelay);
}

void loop() {
  // put your main code here, to run repeatedly:

  myStepper.run();
  unsigned long currentTime = millis();
  distanceTask(currentTime);
  lcdTask(currentTime);
  serialTask(currentTime);
  stateManager();
  runAlarm(currentTime);
}
void distanceTask(unsigned long currentTime) {

  static unsigned long lastDistance;
  int rate = 50;
  int temp;

  if (currentTime - lastDistance >= rate) {
    temp = hc.dist();
    if (temp != 0) {
      distance = temp;
    }
    lastDistance = currentTime;
  }
}

void serialTask(unsigned long currentTime) {
  static unsigned long lastSerial;
  int serialRate = 100;

  if (currentTime - lastSerial >= serialRate) {
    lastSerial = currentTime;
    Serial.print("etd:6308958,dist:");
    Serial.print(distance);
    Serial.print(",deg:");
    Serial.println(degres);
  }
}

void lcdTask(unsigned long currentTime) {

  static unsigned long lastPrint;
  int lcd_rate = 150;

  if (currentTime - lastPrint >= lcd_rate) {

    lastPrint = currentTime;
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Dist : ");
    lcd.print(distance);
    lcd.print("cm");
    lcd.setCursor(0, 1);
    lcd.print("Porte : ");

    switch (etat) {
      case Ouverture:
        lcd.print(degres);
        break;
      case Ouvert:
        lcd.print("Ouvert");
        break;
      case Fermeture:
        lcd.print(degres);
        break;
      case Fermer:
        lcd.print("Fermer");
        break;
    }
  }
}
void stateManager() {
  const int openingDistance = 30;
  const int closingDistance = 60;
  int maxAngle = 170;
  int minAngle = 10;
  degres = map(myStepper.currentPosition(), minStep, maxStep, minAngle, maxAngle);

  if (distance < openingDistance) {

    etat = Ouverture;
  } else if (distance > closingDistance) {
    etat = Fermeture;
  } else if (degres >= maxAngle) {
    etat = Ouvert;
  } else if (degres <= minAngle) {
    etat = Fermer;
  }

  switch (etat) {
    case Ouverture:

      Opening(maxStep);
      break;
    case Fermeture:

      Closing(minStep);
      break;
    case Ouvert:
      open();
      break;
    case Fermer:
      close();
      break;
  }
}

void Opening(int maxStep) {
  myStepper.moveTo(maxStep);
}
void Closing(int minStep) {
  myStepper.moveTo(minStep);
}
void close() {
  if (myStepper.distanceToGo() == 0) {
    myStepper.disableOutputs();
  }
}
void open() {
  if (myStepper.distanceToGo() == 0) {
    myStepper.disableOutputs();
  }
}

void runAlarm(unsigned long currentTime) {
  int alarmWaiter = 3000;
  static bool colorFlag = false;
  static unsigned long lastAlarm;
  int reallyClose = 15;
  int blinkRate = 200;
  static unsigned long lastFlash;

  if (distance <= reallyClose) {
    alarm = rClose;
    lastAlarm = currentTime;
  }
  if (alarm == rClose) {
    digitalWrite(buzzer, HIGH);

    if (currentTime - lastFlash > blinkRate) {
      colorFlag = !colorFlag;
      lastFlash = currentTime;
    }

    if (colorFlag) {
      RGB.setColor(0, 0, 255);
    } else {
      RGB.setColor(255, 0, 0);
    }
  }


  if ((currentTime - lastAlarm >= alarmWaiter)) {
    digitalWrite(buzzer, LOW);
    RGB.off_led();
    alarm = far;
  }
}
