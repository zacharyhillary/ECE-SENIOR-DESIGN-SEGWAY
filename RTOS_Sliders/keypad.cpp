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

void keypadHandler(Adafruit_ILI9341 tft, int pX, int pY)
{
  areaDebugger(tft, INITKEY_X + 0 * KEY_W, INITKEY_Y + 0 * KEY_H, KEY_W, KEY_H);
}
