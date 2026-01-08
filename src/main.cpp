#include <Arduino.h>

#define DEBUG true


#include <Wire.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>
#include <Adafruit_TMP117.h>
#include <BH1750.h>
#include <math.h>
#include <PID_v2.h>
#include <Preferences.h>
#include <ESP32RotaryEncoder.h>
#include "menu.h"
#include "parameters.h"  // Inclusion de parameters.h

// https://www.upesy.fr/blogs/tutorials/esp32-pinout-reference-gpio-pins-ultimate-guide

// Stockage en mémoire
Preferences preferences;

// LCD 20x4 (I2C backpack)
#define LCD_COLS 20
#define LCD_ROWS 4
hd44780_I2Cexp lcd;

// I2C sensors
Adafruit_TMP117 tmp117;
BH1750 lightMeter;
bool tmp117Initialized = false;
bool bh1750Initialized = false;
uint8_t tmp117Address = 0;
uint8_t bh1750Address = 0;

// PWM settings
const int mosfetPin = 5;
const int pwmChannel = 0;
const int pwmFrequency = 500; // 100 % with 5000Hz I had interference and poor thermistor reading !
const int pwmResolution = 8;

// Thermistor settings
int thermistorPin = 15;
float divisorValue = 8400.0; // recalibrated using room sensor
float Bvalue = 3920.0; // extracted from datsheet values : https://www.ovenind.com/pdf/datasheets/DS-TR136.pdf

// Déclaration des paramètres
float Setpoint;
float Kp, Ki, Kd;
float resistorValue;
double Input, Output;
float roomTemperatureC;

// Tableau unique contenant tous les paramètres
Parameter parameters[] = {
    {"Setpoint", &Setpoint, 32.0, 20.0, 40.0, 0.1},
    {"Kp", &Kp, 40.0, 0.0, 100.0, 0.1},
    {"Ki", &Ki, 8.0, 0.0, 50.0, 0.1},
    {"Kd", &Kd, 0.0, 0.0, 10.0, 0.1},
    {"resistorValue", &resistorValue, 15000.0, 1000.0, 30000.0, 500.0}
};

// Nombre total de paramètres
//const int numParameters = sizeof(parameters) / sizeof(Parameter);
int numParameters = sizeof(parameters) / sizeof(Parameter);

// PID controller
PID_v2 myPID( Kp, Ki, Kd, PID::Direct);

// Déclaration des broches pour l'encodeur
#define encoder0PinA  4  // Pin A de l'encodeur
#define encoder0PinB  19 // Pin B de l'encodeur
#define encoder0Press 23  // Pin du bouton de l'encodeur

RotaryEncoder rotaryEncoder(encoder0PinA, encoder0PinB, encoder0Press);

unsigned long lastPIDTime = 0;
unsigned long lastDisplayTime = 0;
const long PIDInterval = 500;
const long DisplayInterval = 500;
bool displayNeedsUpdate=true;

// Function declarations
void setupPWM();
void setupLCD();
void setupPID();
bool readThermistor();
void updatePID();
void debugPrint(const char* message);
void scanI2C();
bool initTMP117();
bool initBH1750();
void handleSerialCommand();
void processCommand(String command);

// PWM helpers (compat core 2.x / 3.x)
bool pwmInit(uint8_t pin, uint32_t freqHz, uint8_t resolutionBits, int8_t preferredChannel = -1);
void pwmWriteDuty(uint8_t pin, uint32_t duty);
void pwmDetach(uint8_t pin);
uint32_t pwmMaxDuty(uint8_t resolutionBits);


bool initSuccess = true;
String serialBuffer = ""; // Tampon pour stocker les caractères reçus


void setup() {
  Serial.begin(9600);
  debugPrint("Init components setup");

  // Setup LCD
  setupLCD();

  debugPrint("Get parameters");
  preferences.begin("my-app", false);
  for (int i = 0; i < numParameters; i++) {
    *(parameters[i].value) = preferences.getFloat(parameters[i].name, parameters[i].defaultValue);
  }

  debugPrint("Init PID");
  myPID.SetTunings(Kp, Ki, Kd);
  myPID.SetOutputLimits(0, 255);
  bool error=readThermistor();

  if (error) {
    debugPrint("Thermistor sensor failed!");
  } else {
    debugPrint("Thermistor OK");
  }


  myPID.Start(Input, 0, Setpoint);

  debugPrint("Setup Encoder");

    pinMode(encoder0Press, INPUT_PULLUP);  // Assurez-vous que la résistance pull-up interne est activée pour le bouton
    rotaryEncoder.setEncoderType(EncoderType::HAS_PULLUP);
    rotaryEncoder.setBoundaries(0, numParameters, true); // Limite le range de 0 à numParameters
    rotaryEncoder.onTurned(&knobCallback);
    rotaryEncoder.onPressed(&buttonCallback);
    rotaryEncoder.begin();

  // rotaryEncoder.setEncoderType(EncoderType::HAS_PULLUP);
  // rotaryEncoder.setBoundaries(0, 5, true);
  // rotaryEncoder.onTurned(&knobCallback);
  // rotaryEncoder.onPressed(&buttonCallback);
  // rotaryEncoder.begin();
  debugPrint("Encoder OK");

  setupPWM();

  debugPrint("TMP117 sensor setup");
  tmp117Initialized = initTMP117();
  if (!tmp117Initialized) {
    debugPrint("TMP117 not detected!");
  } else {
    Serial.print("TMP117 addr: 0x");
    Serial.println(tmp117Address, HEX);
  }

  debugPrint("BH1750 sensor setup");
  bh1750Initialized = initBH1750();
  if (!bh1750Initialized) {
    debugPrint("BH1750 not detected!");
  } else {
    Serial.print("BH1750 addr: 0x");
    Serial.println(bh1750Address, HEX);
  }

  roomTemperatureC = NAN;

  debugPrint("Init procedure OK");
  delay(1000);


   updateDisplay(error);
}

void loop() {
    if (!initSuccess) return;
    unsigned long currentTime = millis();

    // Gérer les commandes reçues sur le port série
    if (Serial.available() > 0) {
        handleSerialCommand();
    }

    // Lire la thermistance et mettre à jour le PID, sauf si en mode d'édition
    bool error = false;
    if (!menuActive || (menuActive && !editing)) {
        if (currentTime - lastPIDTime >= PIDInterval) {
            lastPIDTime = currentTime;
            error = readThermistor();
            if (!error) {
                updatePID();
            } else {
                // Forcer le PWM à 0% en cas d'erreur
                Output = 0;
                pwmWriteDuty((uint8_t)mosfetPin, 0);
            }
            if (tmp117Initialized) {
                sensors_event_t tempEvent;
                if (tmp117.getEvent(&tempEvent)) {
                    roomTemperatureC = tempEvent.temperature;
                } else {
                    roomTemperatureC = NAN;
                }
            } else {
                roomTemperatureC = NAN;
            }

            if (bh1750Initialized) {
                float lux = lightMeter.readLightLevel();
                Serial.print("Lux: ");
                Serial.println(lux);
            }
        }
    }

    // Gérer l'affichage et le menu
    if (menuActive) {
        if (editing) {
            if (displayNeedsUpdate) {
                showSingleParameter(editIndex);
                displayNeedsUpdate = false;
            }
        } else {
            if (displayNeedsUpdate) {
                showMenu();
                displayNeedsUpdate = false;
            }
        }
    } else {
        if (currentTime - lastDisplayTime >= DisplayInterval) {
            lastDisplayTime = currentTime;
            updateDisplay(error); // Passer l'erreur à updateDisplay()
        }
    }
}



void debugPrint(const char* message) {
  if (DEBUG) {
    Serial.println(message);
  }

  if(lcdInitialized) {
      displayTextLine(message);
      delay(500);
  }
}


void setupPWM() {
  if (DEBUG) Serial.println("Init PWM");

  // preferredChannel = pwmChannel (utile sur core 2.x, ignoré sur core 3.x si auto)
  if (!pwmInit((uint8_t)mosfetPin, (uint32_t)pwmFrequency, (uint8_t)pwmResolution, (int8_t)pwmChannel)) {
    initSuccess = false;
    debugPrint("PWM init failed!");
    return;
  }

  pwmWriteDuty((uint8_t)mosfetPin, 0);
}

void setupLCD() {
  debugPrint("Init LCD");

  Wire.begin();
  Wire.setClock(50000);   // 50 kHz (plus robuste sur breadboard)
  Wire.setTimeOut(50);    // évite les blocages infinis
  //Wire.setClock(100000);
  scanI2C();

  int status = lcd.begin(LCD_COLS, LCD_ROWS);
  Serial.print("LCD begin status: ");
  Serial.println(status);
  if (status) {
    lcdInitialized = false;
    debugPrint("LCD init failed!");
    return;
  }

  lcdInitialized = true;
  lcd.backlight();
  lcd.clear();
  debugPrint("LCD ready");
}

void scanI2C() {
  byte error;
  int nDevices = 0;

  Serial.println("I2C scan start");
  for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("I2C device found at 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
      nDevices++;
    }
  }
  if (nDevices == 0) {
    Serial.println("I2C scan: no devices found");
  }
  Serial.println("I2C scan done");
}

bool initTMP117() {
  const uint8_t addresses[] = {0x48, 0x49, 0x4A, 0x4B};
  for (size_t i = 0; i < sizeof(addresses) / sizeof(addresses[0]); ++i) {
    if (tmp117.begin(addresses[i])) {
      tmp117Address = addresses[i];
      return true;
    }
  }
  return false;
}

bool initBH1750() {
  const uint8_t addresses[] = {0x23, 0x5C};
  for (size_t i = 0; i < sizeof(addresses) / sizeof(addresses[0]); ++i) {
    if (lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, addresses[i])) {
      bh1750Address = addresses[i];
      return true;
    }
  }
  return false;
}

void setupPID() {
  pinMode(mosfetPin, OUTPUT);
  
  if (DEBUG) {
    Serial.println("Init PID");
  }
  myPID.SetOutputLimits(0, 255);
  myPID.Start(25, 0, Setpoint);
}

bool readThermistor() {
    int adcValue = analogRead(thermistorPin);
    float voltage = adcValue * (3.3 / 4095.0);

    if (voltage != 0) {
        float thermistorResistance = divisorValue * (3.3 / voltage - 1.0);

        if (thermistorResistance > 0) {
            float temp = 1.0 / (log(thermistorResistance / resistorValue) / Bvalue + 1.0 / 298.15) - 273.15;

            if (temp > 0) {
                Input = temp;

                // Vérifiez si la température dépasse 45°C
                if (temp > 45.0) {
                    if (DEBUG) {
                        Serial.println("Erreur : Température > 45°C !");
                    }
                    return true; // Erreur : température trop élevée
                }
            }
        }
        return false; // Pas d'erreur
    } else {
        if (DEBUG) {
            Serial.println("Thermistor: Tension est 0, erreur détectée !");
        }
        return true; // Erreur : tension de la thermistance est 0
    }
}

void updatePID() {
    // Mise à jour de la sortie PID
    Output = myPID.Run(Input);
    pwmWriteDuty((uint8_t)mosfetPin, (uint32_t)Output);

    // Affichage des informations PID et des paramètres stockés sur une seule ligne
    if (DEBUG) {
        Serial.print("Setpoint: ");
        Serial.print(Setpoint);
        Serial.print(", Input: ");
        Serial.print(Input);
        Serial.print(", Output: ");
        Serial.print(Output);

        // Afficher les paramètres stockés
        for (int i = 0; i < numParameters; i++) {
            float storedValue = preferences.getFloat(parameters[i].name, parameters[i].defaultValue);
            Serial.print(", ");
            Serial.print(parameters[i].name);
            Serial.print(": ");
            Serial.print(storedValue);
        }

        // Terminer la ligne
        Serial.println();
    }
}

void handleSerialCommand() {
    while (Serial.available() > 0) {
        char receivedChar = Serial.read(); // Lire un caractère
        if (receivedChar == '\n') {
            // Une commande complète a été reçue
            serialBuffer.trim(); // Supprimer les espaces inutiles

            if (serialBuffer.length() > 0) {
                // Traiter la commande reçue
                processCommand(serialBuffer);
            }

            // Réinitialiser le buffer pour la prochaine commande
            serialBuffer = "";
        } else {
            // Ajouter le caractère au buffer
            serialBuffer += receivedChar;
        }
    }
}

void processCommand(String command) {
    // Rechercher le délimiteur ":"
    int delimiterIndex = command.indexOf(':');
    if (delimiterIndex == -1) {
        Serial.println("Commande invalide. Format attendu : paramname : value");
        return;
    }

    // Extraire le nom du paramètre et la nouvelle valeur
    String paramName = command.substring(0, delimiterIndex);
    String paramValueStr = command.substring(delimiterIndex + 1);
    paramName.trim();
    paramValueStr.trim();

    // Convertir la valeur en float
    float paramValue = paramValueStr.toFloat();

    // Rechercher le paramètre dans le tableau
    for (int i = 0; i < numParameters; i++) {
        if (paramName == parameters[i].name) {
            // Vérifier si la valeur est dans les limites définies
            if (paramValue >= parameters[i].minValue && paramValue <= parameters[i].maxValue) {
                // Mettre à jour la valeur du paramètre
                *(parameters[i].value) = paramValue;

                // Sauvegarder la nouvelle valeur dans les préférences
                preferences.putFloat(parameters[i].name, paramValue);

                // Si c'est un paramètre PID ou Setpoint, appliquer les changements immédiatement
                if (paramName == "Kp" || paramName == "Ki" || paramName == "Kd" || paramName == "Setpoint") {
                    applyUpdatedParameters();
                }

                // Confirmer la mise à jour
                Serial.print("Parametre ");
                Serial.print(paramName);
                Serial.print(" updated with value : ");
                Serial.println(paramValue);

                return;
            } else {
                Serial.print("Value is outside bounds: ");
                Serial.print(paramName);
                Serial.print(". Min : ");
                Serial.print(parameters[i].minValue);
                Serial.print(", Max : ");
                Serial.println(parameters[i].maxValue);
                return;
            }
        }
    }

    // Si le paramètre n'est pas trouvé
    Serial.print("Paramètre inconnu : ");
    Serial.println(paramName);
}

// -----------------------------------------------------------------------------
// PWM compatibility layer (Arduino-ESP32 core 2.x vs 3.x)
// -----------------------------------------------------------------------------
// Goal: keep main code stable: use pwmInit(pin,freq,res) + pwmWriteDuty(pin,duty)
// Core 3.x: ledcAttach(pin,freq,res) + ledcWrite(pin,duty) (channels auto-managed)
// Core 2.x: ledcSetup(ch,freq,res) + ledcAttachPin(pin,ch) + ledcWrite(ch,duty)

static int8_t g_pwmChannelForPin[40];  // ESP32 has <= 40 GPIO indices (not all exist)
static uint8_t g_pwmResolutionBits = 8;
static bool g_pwmMapInit = false;

uint32_t pwmMaxDuty(uint8_t resolutionBits) {
  if (resolutionBits >= 31) return 0x7FFFFFFFUL;
  return (1UL << resolutionBits) - 1UL;
}

static void pwmEnsureMapInit() {
  if (g_pwmMapInit) return;
  for (int i = 0; i < 40; ++i) g_pwmChannelForPin[i] = -1;
  g_pwmMapInit = true;
}

bool pwmInit(uint8_t pin, uint32_t freqHz, uint8_t resolutionBits, int8_t preferredChannel) {
  pwmEnsureMapInit();
  g_pwmResolutionBits = resolutionBits;

  // Basic guard
  if (pin >= 40) return false;

#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  // Core 3.x: channels can be auto-assigned to pin
  // If you want to force a channel, use ledcAttachChannel (available in 3.x).
  bool ok = false;
  if (preferredChannel >= 0) {
    // Force a given channel if user requested it
    ok = ledcAttachChannel(pin, freqHz, resolutionBits, preferredChannel);
    if (ok) g_pwmChannelForPin[pin] = preferredChannel;
  } else {
    ok = ledcAttach(pin, freqHz, resolutionBits);
    // channel auto-assigned; we don't know it -> keep -1
    g_pwmChannelForPin[pin] = -1;
  }
  return ok;

#else
  // Core 2.x: need an explicit channel.
  int8_t ch = preferredChannel;
  if (ch < 0) {
    // pick a default channel 0 if caller didn't care
    ch = 0;
  }
  ledcSetup((uint8_t)ch, freqHz, resolutionBits);
  ledcAttachPin(pin, (uint8_t)ch);
  g_pwmChannelForPin[pin] = ch;
  return true;
#endif
}

void pwmWriteDuty(uint8_t pin, uint32_t duty) {
  pwmEnsureMapInit();
  const uint32_t maxDuty = pwmMaxDuty(g_pwmResolutionBits);
  if (duty > maxDuty) duty = maxDuty;

#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  // Core 3.x: ledcWrite(pin, duty)
  ledcWrite(pin, duty);
#else
  // Core 2.x: ledcWrite(channel, duty)
  if (pin < 40 && g_pwmChannelForPin[pin] >= 0) {
    ledcWrite((uint8_t)g_pwmChannelForPin[pin], duty);
  } else {
    // Fallback: try channel 0 (best-effort)
    ledcWrite((uint8_t)0, duty);
  }
#endif
}

void pwmDetach(uint8_t pin) {
  pwmEnsureMapInit();
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  ledcDetach(pin);
#else
  ledcDetachPin(pin);
#endif
  if (pin < 40) g_pwmChannelForPin[pin] = -1;
}




