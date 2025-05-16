//#include <LCD_I2C.h>
//#include <AccelStepper.h>
#include <ArduinoJson.h>
#include <HCSR04.h>
#include "SSD1306.h"
#include "Alarm.h"
#include "PorteAutomatique.h"
#include <WiFiEspAT.h>
#include <PubSubClient.h>
#include "Lcd.h"
#include "CapteurDistance.h"
//#include "Potentiometre.h"



#define TRIGGER_PIN 9
#define ECHO_PIN 10


#define HAS_SECRETS 0

#if HAS_SECRETS
#include "arduino_secrets.h"
/////// SVP par soucis de sécurité, mettez vos informations dans le fichier arduino_secrets.h

// Nom et mot de passe du réseau wifi
const char ssid[] = SECRET_SSID;
const char pass[] = SECRET_PASS;

#else
const char ssid[] = "TechniquesInformatique-Etudiant";  // your network SSID (name)
const char pass[] = "shawi123";                         // your network password (use for WPA, or use as key for WEP)

#endif

#if HAS_SECRETS
#define DEVICE_NAME "6308958"
#else
#define DEVICE_NAME "6308958"
#endif

#define MQTT_PORT 1883
#define MQTT_USER "etdshawi"
#define MQTT_PASS "shawi123"

#define AT_BAUD_RATE 115200

// Serveur MQTT du prof
const char* mqttServer = "216.128.180.194";

WiFiClient wifiClient;
PubSubClient client(wifiClient);

unsigned long lastWifiAttempt = 0;
const unsigned long wifiRetryInterval = 10000;
int IN_1 = 10;
int IN_2 = 9;
int IN_3 = 8;
int IN_4 = 7;
int BUZZER_PIN = 2;

int LED_RED = 5;
int LED_GREEN = 4;
int LED_BLUE = 3;
int WIFI_TX = 18;
int WIFI_RX = 19;
int potentiometre = 0;
float temp;
int pot;
bool motor;
String line1;
String line2;
char* name = "Alexis Grenier";
char* number = "6308958";
int lcdSendRate = 1100;


bool dirtyName = false;
bool dirtyNumber = false;
bool dirtyDistance = false;
bool dirtyAngle = false;
bool dirtyTemp = false;
bool dirtyPot = false;
bool dirtyLine1 = false;
bool dirtyLine2 = false;


SSD1306 display(SCREEN_ADDRESS);
Lcd lcd;

enum Commands {
  NONE,
  G_DIST,
  CFG_ALM,
  CFG_LIM_INF,
  CFG_LIM_SUP,
  UNKNOWN
};

Commands command = NONE;

unsigned long currentTime;
unsigned long lcdLastTime = 0;
unsigned long serialLastTime = 0;
unsigned long lastAlarmTriggerTime = 0;

float stepsByDegree = 2038.0 / 360.0;
long angle = 0;
int currentAngle = 0;
int steps = 0;

float distance = 0;
int newDistance = 0;
int minDistance = 30;
int maxDistance = 60;
int alarmTriggerDistance = 15;
int minDegree = 10;
int maxDegree = 170;

int stepperMaxSpeed = 1000;
int stepperAcceleration = 1000;
int stepperSpeed = 1000;

int lcdDelay = 100;
int serialDelay = 100;
int distanceDelay = 50;
int alarmBlinkDelay = 250;
int startingPrintTime = 2000;

bool ledState = false;

String input = "";

PorteAutomatique porte(IN_1, IN_2, IN_3, IN_4, distance);
Alarm alarm(LED_RED, LED_GREEN, LED_BLUE, BUZZER_PIN, &distance);
CapteurDistance hc(12, 13);

void setup() {
  Serial.begin(115200);

  // u8g2.begin();
  // u8g2.setContrast(40);
  // u8g2.setFont(u8g2_font_4x6_tr);
  // u8g2.clearBuffer();
  // u8g2.sendBuffer();
  pinMode(A1, INPUT);

  lcd.begin();
  lcd.printL1("6308958");
  lcd.printL2("Labo 4B");
  delay(startingPrintTime);
  lcd.clear();

  alarm.turnOn();
  alarm.test();
  alarm.setDistance(15);
  alarm.setColourA(255, 0, 0);
  alarm.setColourB(0, 0, 255);
  porte._ouvrir();
  wifiInit();

  client.setServer(mqttServer, MQTT_PORT);
  client.setCallback(mqttEvent);

  if (!client.connect(DEVICE_NAME, MQTT_USER, MQTT_PASS)) {
    Serial.println("Incapable de se connecter sur le serveur MQTT");
    Serial.print("client.state : ");
    Serial.println(client.state());
  } else {
    Serial.println("Connecté sur le serveur MQTT");
  }
  client.subscribe("etd/13/motor", 0);
  client.subscribe("etd/13/color", 0);
}

void loop() {
  pot = analogRead(A1);
  porte.update();
  angle = porte.getAngle();
  currentTime = millis();

  //display.update();

  handleSerialCommands();
  hc.update();
  distance = hc.getDistance();

  //getDistance();
  alarm.update();
  wifiCheck(currentTime);

  lcdTask();
  serialiser(currentTime);

  client.loop();
}

void manageCommand(String input) {
  if (input == "gDist" || input == "g_dist") {
    command = G_DIST;
  } else if (input.startsWith("cfg;alm;")) {
    command = CFG_ALM;
  } else if (input.startsWith("cfg;lim_inf;")) {
    command = CFG_LIM_INF;
  } else if (input.startsWith("cfg;lim_sup;")) {
    command = CFG_LIM_SUP;
  } else {
    command = UNKNOWN;
  }
}

void handleSerialCommands() {
  if (Serial.available() > 0) {
    input = Serial.readStringUntil('\n');
    input.trim();
    //Serial.println("You typed: " + input);

    manageCommand(input);

    switch (command) {
      case G_DIST:
        Serial.println(distance);
        display.displaySuccess();
        break;

      case CFG_ALM:
        {
          int val = input.substring(input.lastIndexOf(';') + 1).toInt();
          alarmTriggerDistance = val;
          display.displaySuccess();
          break;
        }

      case CFG_LIM_INF:
        {
          int val = input.substring(input.lastIndexOf(';') + 1).toInt();
          if (val >= maxDistance) {
            Serial.println("Erreur : Limite inférieure plus grande que limite supérieure");
            display.displayError();
          } else {
            minDistance = val;
            display.displaySuccess();
          }
          break;
        }

      case CFG_LIM_SUP:
        {
          int val = input.substring(input.lastIndexOf(';') + 1).toInt();
          if (val <= minDistance) {
            Serial.println("Erreur : Limite supérieure plus petite que limite inférieure");
            display.displayError();
          } else {
            maxDistance = val;
            display.displaySuccess();
          }
          break;
        }

      case UNKNOWN:
        display.displayUnknown();
        break;

      default:
        break;
    }

    input = "";
    command = NONE;
  }
}

void lcdTask() {
  static unsigned long lastTime = 0;
  static String lastLine1 = "";
  static String lastLine2 = "";
  const int lcdRate = 500;

  if (currentTime - lastTime >= lcdRate) {
    lastTime = currentTime;

    line1 = "Dist: " + String((int)distance) + " cm";
    if (lastLine1 != line1) {
      lastLine1 = line1;
      lcd.printL1(line1);
    }

    AlarmState alarmState = alarm.getState();
    const char* etatPorte = porte.getEtatTexte();
    const char* FERMEE = "FERMEE";
    const char* OUVERTE = "OUVERTE";

    if (alarmState == ON) {
      line2 = "Angle: " + String((int)angle);
    } else if (alarmState == WATCHING && strcmp(etatPorte, FERMEE) == 0) {
      line2 = "Porte : fermer";
    } else if (alarmState == WATCHING && strcmp(etatPorte, OUVERTE) == 0) {
      line2 = "Porte : ouvert";
    } else {
      //Serial.println(angle);
      line2 = "Angle: " + String((int)angle);
    }
    if (lastLine2 != line2) {
      lastLine2 = line2;
      lcd.printL2(line2);
    }
  }
}

void wifiInit() {
  while (!Serial)
    ;

  Serial1.begin(AT_BAUD_RATE);
  WiFi.init(Serial1);

  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println();
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true)
      ;
  }

  WiFi.disconnect();  // to clear the way. not persistent

  WiFi.setPersistent();  // set the following WiFi connection as persistent

  WiFi.endAP();  // to disable default automatic start of persistent AP at startup

  //  uncomment this lines for persistent static IP. set addresses valid for your network
  //  IPAddress ip(192, 168, 1, 9);
  //  IPAddress gw(192, 168, 1, 1);
  //  IPAddress nm(255, 255, 255, 0);
  //  WiFi.config(ip, gw, gw, nm);

  Serial.println();
  Serial.print("Attempting to connect to SSID: ");
  Serial.println(ssid);

  //  use following lines if you want to connect with bssid
  //  const byte bssid[] = {0x8A, 0x2F, 0xC3, 0xE9, 0x25, 0xC0};
  //  int status = WiFi.begin(ssid, pass, bssid);

  int status = WiFi.begin(ssid, pass);

  if (status == WL_CONNECTED) {
    Serial.println();
    Serial.println("Connected to WiFi network.");
    printWifiStatus();
  } else {
    WiFi.disconnect();  // remove the WiFi connection
    Serial.println();
    Serial.println("Connection to WiFi network failed.");
  }
}

bool reconnect() {
  if (client.connect(DEVICE_NAME, MQTT_USER, MQTT_PASS)) {
    Serial.println("Connecté au serveur MQTT");
    client.subscribe("etd/13/motor");
    client.subscribe("etd/13/color");
    return true;
  } else {
    Serial.println("Impossible de se connecter au serveur MQTT");
    return false;
  }
}
void printWifiStatus() {

  // print the SSID of the network you're attached to:
  char ssid[33];
  WiFi.SSID(ssid);
  Serial.print("SSID: ");
  Serial.println(ssid);

  // print the BSSID of the network you're attached to:
  uint8_t bssid[6];
  WiFi.BSSID(bssid);
  Serial.print("BSSID: ");
  printMacAddress(bssid);

  uint8_t mac[6];
  WiFi.macAddress(mac);
  Serial.print("MAC: ");
  printMacAddress(mac);

  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}
void printMacAddress(byte mac[]) {
  for (int i = 5; i >= 0; i--) {
    if (mac[i] < 16) {
      Serial.print("0");
    }
    Serial.print(mac[i], HEX);
    if (i > 0) {
      Serial.print(":");
    }
  }
  Serial.println();
}
void mqttEvent(char* topic, byte* payload, unsigned int length) {
  String msg;

  Serial.print("Message reçu [");
  Serial.print(topic);
  Serial.print("] ");

  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
    msg += (char)payload[i];
  }

  JsonDocument doc;
  deserializeJson(doc, msg);

  if (strcmp(topic, "etd/13/motor") == 0) {
    motor = doc["motor"];

    motor ? porte._ouvrir() : porte._fermer();
  } else if (strcmp(topic, "etd/13/color") == 0) {
    String colorHex = msg;

    Serial.print("Couleur reçue : ");
    Serial.println(colorHex);
    //color = doc["color"];
    SetLedColour(doc["color"]);
  }

  Serial.println();


  if (strcmp(topic, "etd/13/color") == 0) {
    String colorHex = msg;

    Serial.print("Couleur reçue : ");
    Serial.println(colorHex);
  }
}

void SetLedColour(const char* hexColor) {
  Serial.println(hexColor);
  if (hexColor[0] == '#') {
    hexColor++;
  }
  // Assurez-vous que la chaîne hexColor commence par '#' et a une longueur de 7 caractères (ex: #FF5733)
  if (strlen(hexColor) == 6) {
    // Extraction des valeurs hexadécimales pour rouge, vert et bleu
    long number = strtol(hexColor, NULL, 16);  // Convertit hex à long

    int red = number >> 16;            // Décale de 16 bits pour obtenir le rouge
    int green = (number >> 8) & 0xFF;  // Décale de 8 bits et masque pour obtenir le vert
    int blue = number & 0xFF;          // Masque pour obtenir le bleu

    // Définissez les couleurs sur les broches de la DEL
    alarm.setColourA(red, green, blue);
  }
}
void verifyDirty() {
  static char lastName[20] = "";
  static char lastNumber[20] = "";
  static float lastDistance = -1;
  static float lastAngle = -1;
  static float lastTemp = -100;
  static int lastPot = -1;

  if (strcmp(lastName, name) != 0) {
    strcpy(lastName, name);
    dirtyName = true;
  }

  if (strcmp(lastNumber, number) != 0) {
    strcpy(lastNumber, number);
    dirtyNumber = true;
  }

  if (lastDistance != distance) {
    lastDistance = distance;
    dirtyDistance = true;
  }

  if (lastAngle != angle) {
    lastAngle = angle;
    dirtyAngle = true;
  }

  if (lastTemp != temp) {
    lastTemp = temp;
    dirtyTemp = true;
  }

  if (lastPot != pot) {
    potentiometre = map(pot, 0, 1023, 0, 100);
    lastPot = pot;

    dirtyPot = true;
  }
}
void wifiCheck(unsigned long currentTime) {
  if (WiFi.status() != WL_CONNECTED) {
    if (currentTime - lastWifiAttempt >= wifiRetryInterval) {
      Serial.println("WiFi perdu. Tentative de reconnexion...");
      WiFi.begin(ssid, pass);
      lastWifiAttempt = currentTime;
    }
  }
}
void serialiser(unsigned long currentTime) {
  static unsigned long lastTime = 0;
  static unsigned long uptime;

  if (currentTime - lastTime > 2500) {
    lastTime = currentTime;


    uptime = currentTime / 1000;

    // th.update();
    // temp = th.getTemperature();
    // hum = th.getHumidity();

    // char message[200];
    // sprintf(message, "{\"name\":\"%s\", \"number\":\"%s\", \"uptime\":%lu, \"dist\":%d, \"angle\":%d, \"temp\":%d, \"hum\":%d, \"line1\":\"%s\"}", name, number, uptime, (int)distance, (int)angle, (int)temp, (int)hum, line1);

    StaticJsonDocument<200> doc;

    verifyDirty();

    if (dirtyName) doc["name"] = name;
    if (dirtyNumber) doc["number"] = number;
    doc["uptime"] = uptime;
    if (dirtyDistance) doc["dist"] = (int)distance;
    if (dirtyAngle) doc["angle"] = (int)angle;
    if (dirtyPot) doc["pot"] = (int)potentiometre;


    char message[200];
    serializeJson(doc, message, sizeof(message));

    if (!client.publish("etd/13/data", message)) {
      Serial.println("Erreur lors de l'envoi du message");
    } else {
      //Serial.println("Message envoyé");
    }
    dirtyName = dirtyNumber = dirtyDistance = dirtyAngle = dirtyTemp = dirtyPot = false;
  }

  static unsigned long lastLcdTime = 0;
  if (currentTime - lastLcdTime >= lcdSendRate) {
    lastLcdTime = currentTime;
    StaticJsonDocument<200> lcdDoc;
    static String lastLine1 = "";
    static String lastLine2 = "";

    if (lastLine1 != line1) {
      lastLine1 = line1;
      dirtyLine1 = true;
    }

    if (lastLine2 != line2) {
      lastLine2 = line2;
      dirtyLine2 = true;
    }

    if (dirtyLine1) lcdDoc["line1"] = line1;
    if (dirtyLine2) lcdDoc["line2"] = line2;

    if (dirtyLine1 || dirtyLine2) {
      char lcdMessage[200];
      serializeJson(lcdDoc, lcdMessage, sizeof(lcdMessage));

      // Send the LCD data to MQTT
      if (!client.publish("etd/13/data", lcdMessage)) {
        Serial.println("Erreur lors de l'envoi du message LCD");
      } else {
        // Serial.println("Message LCD envoyé");
      }
      dirtyLine1 = dirtyLine2 = false;
    }
  }
}
