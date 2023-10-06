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

// 32-bit color values
#define GRAY 0xdf1b

// keys
#define KEY_W 50
#define KEY_H 40

#define INITKEY_X 85
#define INITKEY_Y 50

#define INPUT_DELAY 2000

int lastInputMillis = millis();

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

void printArray(int array[]) {
    Serial.print("Array: [");
    for(int i=0; i<3; i++) {
      Serial.print(array[i]);
      Serial.print(", ");
    }
    Serial.println("]");
}

void appendInt(int input, int array[]) {
    int currentMillis = millis();
    if(currentMillis - lastInputMillis < INPUT_DELAY) {
      return;
    } else {
      lastInputMillis = currentMillis;
    }
    int j=0;
    while(j != 3 && array[j] != -1) j++;
    if(j == 3) {
      return;
    } else {
      array[j] = input;
    }
    printArray(array);
}

char inputValueStr[5];
int keypadHandler(Adafruit_ILI9341 tft, int pX, int pY, int keyInput[])
{
    areaDebugger(tft, INITKEY_X + 1 * KEY_W, INITKEY_Y + -1 * KEY_H - 5, KEY_W, KEY_H);
    tft.fillRect(INITKEY_X + KEY_W, INITKEY_Y + -1 * KEY_H - 5, KEY_W, KEY_H, GRAY);
    tft.drawRect(INITKEY_X + KEY_W, INITKEY_Y + -1 * KEY_H - 5, KEY_W, KEY_H, BLACK);
    tft.setCursor(INITKEY_X + KEY_W * 1.2, INITKEY_Y + -1 * KEY_H - 5 + KEY_H * 0.4);
    tft.setTextColor(BLACK);
    sprintf(inputValueStr, "%03d", (keyInput[0] == -1 ? 0 : keyInput[0]) * 100 + (keyInput[1] == -1 ? 0 : keyInput[1]) * 10 + (keyInput[2] == -1 ? 0 : keyInput[2]));
    tft.print(inputValueStr);
    for(int i=0; i<12; i++) {
        int row = i / 3;
        int column = i % 3;
        if(checkAreaClicked(pX, pY, INITKEY_X + column * KEY_W, INITKEY_Y + row * KEY_H + 20, KEY_W, KEY_H)) {
            if(row == 3) {
              if(column == 0) {
                  keyInput[0] = -1;
                  keyInput[1] = -1;
                  keyInput[2] = -1;
                  printArray(keyInput);
              } else if(column == 1) {
                  appendInt(0, keyInput);
              } else {
                  return 0;
              }
            } else {
                appendInt(row * 3 + column + 1, keyInput);   
            }
        }
    }
    return -1;
}
