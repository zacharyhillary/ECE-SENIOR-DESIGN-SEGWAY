#include "Arduino.h"
#include <Adafruit_GFX.h>
#include <Adafruit_FT6206.h>
#include <Adafruit_ILI9341.h>
#include <SPI.h>
#include "utility_drivers.h"

// Common 16-bit color values:
#define BLACK 0x0000
#define BLUE 0x001F
#define RED 0xF800
#define GREEN 0x07E0
#define CYAN 0x07FF
#define MAGENTA 0xF81F
#define YELLOW 0xFFE0
#define WHITE 0xFFFF
#define LAPIS 0x2998
#define GRAY 0xdf1b

// keys
#define KEY_W 50
#define KEY_H 40

#define INITKEY_X 85
#define INITKEY_Y 50

#define INPUT_DELAY 2000

int lastInputMillis = millis();
int lastDrawMillis = millis();

char charInputs[] = { '0', '1', '2', '3', '4', '5', '6', '7', '8', '9' };

int charInputToInt(char input) {
    switch(input) {
       case '0': return 0; break;
       case '1': return 1; break;
       case '2': return 2; break;
       case '3': return 3; break;
       case '4': return 4; break;
       case '5': return 5; break;
       case '6': return 6; break;
       case '7': return 7; break;
       case '8': return 8; break;
       case '9': return 9; break;
       case '\0': return 0; break;
       default: Serial.println("Bad things happened here in charInputToInt"); return -1; break;
    }
}

void printArray(char array[]) {
  Serial.print("Array: [");
  for (int i = 0; i < 4; i++) {
    Serial.print(array[i]);
    if(i != 3) {
      Serial.print(", ");
    }
  }
  Serial.println("]");
}

double inputArrayToDecimal(char array[]) {
  printArray(array);
  double result = 0;
  for(int i=0; i<4; i++) {   
      result += pow(10, (1-i)) * charInputToInt(array[i]);
  }
  Serial.print("Result: ");
  Serial.println(result);
  return result;
}

void createKeyPad(Adafruit_ILI9341 tft) 
{ 
  int count = 1; 
 
  for (int i = 0; i < 4; i++) 
  { 
    for (int j = 0; j < 3; j++) 
    { 
      tft.fillRect(INITKEY_X + j * KEY_W, INITKEY_Y + i * KEY_H, KEY_W, KEY_H, GRAY); 
      tft.drawRect(INITKEY_X + j * KEY_W, INITKEY_Y + i * KEY_H, KEY_W, KEY_H, BLACK); 
      tft.setCursor(INITKEY_X + j * KEY_W + KEY_W * 0.3, INITKEY_Y + i * KEY_H + KEY_H / 2 - 10); 
      tft.setTextColor(BLACK); 
      if (i == 3 && j == 0) { 
        tft.setCursor(INITKEY_X + j * KEY_W + KEY_W * 0.2, INITKEY_Y + i * KEY_H + KEY_H / 2 - 10); 
        tft.setTextSize(2); 
        tft.print("CLR"); 
      } else if(i == 3 && j == 2) { 
        tft.setTextSize(3); 
        tft.setCursor(INITKEY_X + j * KEY_W + KEY_W * 0.2, INITKEY_Y + i * KEY_H + KEY_H / 2 - 10); 
        tft.print("OK"); 
      } else { 
        tft.setTextSize(3); 
        tft.print(count == 10 ? 0 : count); 
        count++; 
      } 
    } 
  } 
} 

void appendChar(char input, char array[]) {
  int currentMillis = millis();
  if (currentMillis - lastInputMillis < INPUT_DELAY) {
    return;
  } else {
    lastInputMillis = currentMillis;
  }
  int j = 0;
  while (j != 4 && array[j] != '\0') j++;
  if (j == 4) {
    return;
  } else {
    array[j] = input;
  }
  printArray(array);
}

int keypadHandler(Adafruit_ILI9341 tft, int pX, int pY, char keyInput[]) 
{ 
    if(millis() - lastDrawMillis >= 500) {
      tft.fillRect(INITKEY_X + KEY_W, INITKEY_Y + -1 * KEY_H - 5, KEY_W, KEY_H, GRAY); 
      tft.drawRect(INITKEY_X + KEY_W, INITKEY_Y + -1 * KEY_H - 5, KEY_W, KEY_H, BLACK); 
      tft.setCursor(INITKEY_X + KEY_W * 1.2, INITKEY_Y + -1 * KEY_H - 5 + KEY_H * 0.4); 
      for(int i=0; i<4; i++) {
          if (keyInput[i] == '\0') {
            tft.setTextColor(LAPIS);
          } else {
            tft.setTextColor(BLACK);
          }
          tft.print(charInputToInt(keyInput[i]));
          if(i == 1) {
            tft.setTextColor(BLACK);
            tft.print(".");
          }
      }
      lastDrawMillis = millis();
    }
    areaDebugger(tft, INITKEY_X + 1 * KEY_W, INITKEY_Y + -1 * KEY_H - 5, KEY_W, KEY_H); 
    for(int i=0; i<12; i++) { 
        int row = i / 3; 
        int column = i % 3; 
        if(checkAreaClicked(pX, pY, INITKEY_X + column * KEY_W, INITKEY_Y + row * KEY_H + 20, KEY_W, KEY_H)) { 
            if(row == 3) { 
              if(column == 0) { 
                  keyInput[0] = '\0';
                  keyInput[1] = '\0';
                  keyInput[2] = '\0';
                  keyInput[3] = '\0';
                  printArray(keyInput); 
              } else if(column == 1) { 
                  appendChar(charInputs[0], keyInput); 
              } else { 
                  return 0; 
              } 
            } else { 
                appendChar(charInputs[row * 3 + column + 1], keyInput);    
            } 
        } 
    } 
    return -1; 
} 
