#include <Arduino.h>
#include <stdio.h>
#include <string.h>
#include "menu.h"
#include "parameters.h"

#define LCD_COLS 20
#define LCD_ROWS 4
#define MENU_NAME_WIDTH 8

// Déclaration des broches pour l'encodeur
#define encoder0PinA  4  // Pin A de l'encodeur
#define encoder0PinB  19 // Pin B de l'encodeur
#define encoder0Press 23  // Pin du bouton de l'encodeur

volatile int menuIndex = 0;
bool menuActive = false;
bool editing = false;
int editIndex = -1;
int lastEncoderPosition = 0;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 20;
int cursorRow = 0; // Position verticale du curseur sur l'ecran (0..3)
bool lcdInitialized=false;

static void lcdPrintLine(uint8_t row, const char* text) {
    char buf[LCD_COLS + 1];
    size_t len = strlen(text);
    if (len > LCD_COLS) {
        len = LCD_COLS;
    }
    memset(buf, ' ', LCD_COLS);
    memcpy(buf, text, len);
    buf[LCD_COLS] = '\0';
    lcd.setCursor(0, row);
    lcd.print(buf);
}

static void lcdClearRows() {
    for (uint8_t row = 0; row < LCD_ROWS; ++row) {
        lcdPrintLine(row, "");
    }
}

void knobCallback(long value) {
    unsigned long currentTime = millis();

    // if ((currentTime - lastDebounceTime) > debounceDelay) {
    //     lastDebounceTime = currentTime;

         int delta = value - lastEncoderPosition;
         lastEncoderPosition = value;

         if (delta==numParameters){
            delta=-1;
           }
         if (delta==-numParameters){
            delta=1;
           }
         

        if (editing) {
            adjustParameter(editIndex, delta > 0 ? 1 : -1);
            showSingleParameter(editIndex);
        } else if (menuActive) {
            menuIndex = value; //+= delta > 0 ? 1 : -1;

            Serial.println(menuIndex);

            //menuIndex = constrain(menuIndex, 0, numParameters);
            showMenu();
        }
   // }
}

void buttonCallback(unsigned long duration) {

  if (duration>1000) {
    return; // remove wrong calls to the function ; encoder bug
  }

  //Serial.println(duration);

    unsigned long currentTime = millis();

    if ((currentTime - lastDebounceTime) > debounceDelay) {
        lastDebounceTime = currentTime;

        // Indiquer que le bouton vient d'être pressé
       // buttonRecentlyPressed = true;

        if (!menuActive) {
            menuActive = true;
            menuIndex=0;
            showMenu();
            delay(100);
            showMenu();
        } else {
            if (!editing) {
                if (menuIndex == 0) {
                    menuActive = false;
                } else {
                    editing = true;
                    editIndex = menuIndex - 1;
                    showSingleParameter(editIndex);
                }
            } else {
                lastEncoderPosition = menuIndex;
                editing = false;
                editIndex = -1;
                showMenu();
            }
        }
    }
}

void showMenu() {
    int visibleItems = LCD_ROWS;
    int topItem = 0;

//Serial.println(menuIndex);

    // Ajuster topItem pour s'assurer que menuIndex est toujours visible
    if (menuIndex >= visibleItems) {
        topItem = menuIndex - visibleItems + 1;
    }

    // Afficher les éléments du menu
    for (int i = 0; i < visibleItems; i++) {
        int itemIndex = topItem + i;

        if (itemIndex > numParameters) {
            lcdPrintLine(i, "");
            continue;
        }

        char line[LCD_COLS + 1];
        char selector = (itemIndex == menuIndex) ? '>' : ' ';

        if (itemIndex == 0) {
            snprintf(line, sizeof(line), "%c Back", selector);
        } else {
            char nameBuf[MENU_NAME_WIDTH + 1];
            strncpy(nameBuf, parameters[itemIndex - 1].name, MENU_NAME_WIDTH);
            nameBuf[MENU_NAME_WIDTH] = '\0';
            snprintf(
                line,
                sizeof(line),
                "%c %-8s:%8.2f",
                selector,
                nameBuf,
                *(float*)parameters[itemIndex - 1].value
            );
        }

        lcdPrintLine(i, line);
    }
}

void showSingleParameter(int index) {
    char line[LCD_COLS + 1];

    snprintf(line, sizeof(line), "Set %s", parameters[index].name);
    lcdPrintLine(0, line);

    snprintf(line, sizeof(line), "Value: %.2f", *(parameters[index].value));
    lcdPrintLine(1, line);
    lcdPrintLine(2, "Rotate=Adj");
    lcdPrintLine(3, "Press=Back");
}

void applyUpdatedParameters() {
    // Forcer la sortie à zéro
    Output = 0;

    // Réinitialiser les paramètres PID
    myPID.SetTunings(Kp, Ki, Kd);
    myPID.SetOutputLimits(-1, 0); // artificially resests the integrator
    myPID.SetOutputLimits(0, 255);
   //myPID.SetMode(MANUAL);
   // myPID.SetMode(AUTOMATIC); // reini
    // Redémarrer le PID avec les paramètres actuels
    myPID.Start(Input, Output, Setpoint);

    // Afficher un message pour confirmer la réinitialisation
    Serial.println("PID réinitialisé avec les nouveaux paramètres.");
    Serial.print("Kp: "); Serial.print(Kp);
    Serial.print(", Ki: "); Serial.print(Ki);
    Serial.print(", Kd: "); Serial.print(Kd);
    Serial.print(", Setpoint: "); Serial.println(Setpoint);

    // Vérifier si Output est bien nul
    if (Output == 0) {
        Serial.println("Sortie PID réinitialisée à zéro.");
    } else {
        Serial.print("Avertissement : Sortie PID non nulle (");
        Serial.print(Output);
        Serial.println(")");
    }
}


void adjustParameter(int index, int direction) {
    if (index >= 0 && index < numParameters) {
        // Ajuster la valeur pointée par le paramètre
        float newValue = *(parameters[index].value) + direction * parameters[index].increment;
        newValue = constrain(newValue, parameters[index].minValue, parameters[index].maxValue);
        *(parameters[index].value) = newValue;

        // Sauvegarder la nouvelle valeur dans les préférences
        preferences.putFloat(parameters[index].name, newValue);

        // Si le paramètre modifié est l'un des paramètres PID, appliquer les changements
        if (parameters[index].name == "Kp" || parameters[index].name == "Ki" || 
            parameters[index].name == "Kd" || parameters[index].name == "Setpoint") {
            applyUpdatedParameters();
        }
    }
}

// void updateDisplay() {
//   display.clearDisplay();
//   display.setTextSize(2);
//   display.setTextColor(SSD1306_WHITE);
//   display.setCursor(0, 0);
//   display.print(Input, 1);
//  // float dhtTemperatureC = dht.readTemperature();
//   display.setCursor(70, 0);
//   if (isnan(dhtTemperatureC)) {
//     display.print("Err");
//   } else {
//     display.print(dhtTemperatureC, 1);
//   }
//   display.setCursor(0, 25);
//   display.print("Set: ");
//   display.print(Setpoint, 1);
//   int outputPercentage = (int)(Output / 255.0 * 100);
//   display.setCursor(0, 50);
//   display.print("PWM: ");
//   display.print(outputPercentage);
//   display.println("%");
//   display.display();
// }

void updateDisplay(bool error) {
    char line[LCD_COLS + 1];
    char inputBuf[8];
    char roomBuf[8];
    char degreeChar = (char)0xDF;

    if (error) {
        snprintf(inputBuf, sizeof(inputBuf), "Err");
    } else {
        snprintf(inputBuf, sizeof(inputBuf), "%.1f", Input);
    }

    if (isnan(roomTemperatureC)) {
        snprintf(roomBuf, sizeof(roomBuf), "Err");
    } else {
        snprintf(roomBuf, sizeof(roomBuf), "%.1f", roomTemperatureC);
    }

    snprintf(line, sizeof(line), "Set : %.1f%cC", Setpoint, degreeChar);
    lcdPrintLine(0, line);

    snprintf(line, sizeof(line), "Current : %-5s%cC", inputBuf, degreeChar);
    lcdPrintLine(1, line);

    snprintf(line, sizeof(line), "Room : %-5s%cC", roomBuf, degreeChar);
    lcdPrintLine(2, line);

    int outputPercentage = error ? 0 : (int)(Output / 255.0 * 100);
    snprintf(line, sizeof(line), "PWM: %3d%%", outputPercentage);
    lcdPrintLine(3, line);
}

void displayTextLine(const char* text) {
    // Vérifie si on dépasse la hauteur de l'écran
    if (cursorRow >= LCD_ROWS) {
        cursorRow = 0;
    }

    if (cursorRow == 0) {
        lcdClearRows();
    }

    // Affiche le texte à la position actuelle du curseur
    lcdPrintLine(cursorRow, text);

    // Passe à la ligne suivante
    cursorRow += 1;
}
