#include <HCSR04.h>
#include <LCD_I2C.h>
#include <AccelStepper.h>
#include <U8g2lib.h>

LCD_I2C lcd(0x27, 16, 2);
#define MOTOR_INTERFACE_TYPE 4
#define TRIGGER_PIN 3  // Capteur distance pin
#define ECHO_PIN 2     // Capteur distance pin
#define IN_1 10        //pins du accel stepper
#define IN_2 9
#define IN_3 7
#define IN_4 6
#define buzzer 4
HCSR04 hc(TRIGGER_PIN, ECHO_PIN);
AccelStepper myStepper(MOTOR_INTERFACE_TYPE, IN_1, IN_3, IN_2, IN_4);  //une révolution =2038 pas
#define CLK_PIN 30
#define DIN_PIN 34
#define CS_PIN 32  // Chip Select

// Pour un module unique 8×8
//  - Si la documentation indique 8×8, U8g2 utilise l'appellation 8 de haut × 8 multiples de large
//  - Parfois, on choisit U8G2_MAX7219_8X8_F_4W_SW_SPI ou un modèle équivalent
U8G2_MAX7219_8X8_F_4W_SW_SPI u8g2(
  U8G2_R0,             // rotation
  /* clock=*/CLK_PIN,  // pin Arduino reliée à CLK (horloge)
  /* data=*/DIN_PIN,   // pin Arduino reliée à DIN (données)
  /* cs=*/CS_PIN,      // pin Arduino reliée à CS (chip select)
  /* dc=*/U8X8_PIN_NONE,
  /* reset=*/U8X8_PIN_NONE);

float degres;
int distance;
int currentStep;
int minStep = 2038 * (170.0 / 360.0);
int maxStep = 2038 * (10.0 / 360.0);
int setupPrintDelay = 2000;
int off = LOW;
String currentCommand;
char currentChar;

int reallyClose = 15;
//motor
int openingDistance = 30;
int closingDistance = 60;
const int maxAngle = 170;
const int minAngle = 10;
int wayToFar = 180;
int wayToClose = 0;



//lab6
enum MaxIcon { ICON_NONE,
               ICON_OK,
               ICON_ERROR,
               ICON_UNKNOWN };
MaxIcon currentIcon;
;
MaxIcon lastIcon;
unsigned long iconStartTime = 0;
bool iconTemporary = false;
String input;

//

#pragma region class


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

#pragma endregion class

enum Alarm { rClose,
             far };
Alarm alarm;

enum Porte { Ouverture,
             Ouvert,
             Fermeture,
             Fermer };
Porte etat;


void setup() {
  Serial.begin(9600);
  u8g2.begin();
  u8g2.setContrast(40);
  u8g2.setFont(u8g2_font_4x6_tr);
  u8g2.clearBuffer();
  u8g2.sendBuffer();

  pinMode(buzzer, OUTPUT);
  myStepper.setMaxSpeed(500);
  myStepper.setAcceleration(300);

  Serial.println("Commandes disponibles : ");
  Serial.println("gDist , donne la distance ");
  Serial.println("cfg;alm;X , Configure la distance limite du système d'alarme (x = distance en cm) ");
  Serial.println("cfg;lim_sup;X , configure la limite supérieure du moteur  ");
  Serial.println("cfg;lim_inf;X , configure la limite inférieure du moteur ");

  lcd.begin();
  lcd.backlight();
  lcd.print("6308958");
  lcd.setCursor(0, 1);
  lcd.print("Labo 4A");
  delay(setupPrintDelay);
}

void loop() {
  myStepper.run();
  unsigned long currentTime = millis();
  distanceTask(currentTime);
  lcdTask(currentTime);
  stateManager();
  runAlarm(currentTime);
  commandAccept(currentTime);
  showIcon(currentTime);
}

#pragma region labo4

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

// void serialTask(unsigned long currentTime) {
//   static unsigned long lastSerial;
//   int serialRate = 100;

//   if (currentTime - lastSerial >= serialRate) {
//     lastSerial = currentTime;
//     Serial.print("etd:6308958,dist:");
//     Serial.print(distance);
//     Serial.print(",deg:");
//     Serial.println(degres);
//   }
// }

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
#pragma endregion

#pragma region labo5
void runAlarm(unsigned long currentTime) {
  int alarmWaiter = 3000;
  static bool colorFlag = false;
  static unsigned long lastAlarm;
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
#pragma endregion

#pragma region labo6

void commandAccept(unsigned long currentTime) {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      processCommand(input, currentTime);
      input = "";
    } else {
      input += c;
    }
  }
}

void processCommand(String cmd, unsigned long currentTime) {
  cmd.trim();

  if (cmd == "gDist") {
    Serial.print("Arduino : ");
    Serial.println(distance);
    setIcon(ICON_OK, false);
  } else if (cmd.startsWith("cfg;alm;")) {
    int val = cmd.substring(8).toInt();

    if (val >= wayToFar || val <= wayToClose) {
      Serial.print("mettez des limites réaliste ");
      setIcon(ICON_ERROR, true);
    } else {
      reallyClose = val;
      Serial.print("Configure la distance de détection de l’alarme à ");
      Serial.print(val);
      Serial.println(" cm");
      setIcon(ICON_OK, false);
    }

  } else if (cmd.startsWith("cfg;lim_inf;")) {
    int val = cmd.substring(12).toInt();
    if (val <= wayToClose) {
      setIcon(ICON_ERROR, true);
      Serial.println("limite trop petite!");
    } else {
      if (val >= closingDistance) {
        Serial.println("Erreur  Limite inférieure plus grande que limite supérieure donnée");
        setIcon(ICON_ERROR, true);

      } else {
        openingDistance = val;
        Serial.print("Limite inférieure configurée à ");
        Serial.println(val);
        setIcon(ICON_OK, false);
      }
    }
  } else if (cmd.startsWith("cfg;lim_sup;")) {
    int val = cmd.substring(12).toInt();
    if (val >= wayToFar) {
      Serial.println("Erreur  Limite trop grande");
      setIcon(ICON_ERROR, true);

    } else {
      if (val <= openingDistance) {
        Serial.println("Erreur  Limite supérieure plus petite que limite inférieure");
        setIcon(ICON_ERROR, true);
      } else {
        closingDistance = val;
        Serial.print("Limite supérieure configurée à ");
        Serial.println(val);
        setIcon(ICON_OK, false);
      }
    }
  } else {
    Serial.println("Commande inconnue.");
    setIcon(ICON_UNKNOWN, true);
  }
}


void showIcon(unsigned long currentTime) {

  if (iconTemporary && currentIcon != ICON_NONE && (currentTime - iconStartTime >= 3000)) {
    currentIcon = ICON_NONE;
    iconTemporary = false;
  }

  drawIcon();
}
void setIcon(MaxIcon icon, bool temporary) {
  if (icon != currentIcon || temporary != iconTemporary) {
    currentIcon = icon;
    iconTemporary = temporary;
    if (temporary) {
      iconStartTime = millis();
    }
  }
}

void drawIcon() {
  u8g2.clearBuffer();

  switch (currentIcon) {
    case ICON_OK:
      u8g2.drawLine(3, 1, 1, 3);
      u8g2.drawLine(1, 3, 7, 7);
      break;
    case ICON_ERROR:
      u8g2.drawCircle(3, 3, 3);
      u8g2.drawLine(0, 0, 7, 7);
      break;
    case ICON_UNKNOWN:
      u8g2.drawLine(0, 0, 7, 7);
      u8g2.drawLine(7, 0, 0, 7);
      break;
    case ICON_NONE:
      // rien à afficher
      break;
  }

  u8g2.sendBuffer();
}

#pragma endregion labo6
