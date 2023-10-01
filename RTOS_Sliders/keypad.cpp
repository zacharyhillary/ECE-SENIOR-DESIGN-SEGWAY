#include <Adafruit_GFX.h>
#include <Adafruit_FT6206.h>
#include <Adafruit_ILI9341.h>
#include <SPI.h>

// Common 16-bit color values:
#define BLACK 0x0000
#define BLUE 0x001F
#define RED 0xF800
#define GREEN 0x07E0
#define CYAN 0x07FF
#define MAGENTA 0xF81F
#define YELLOW 0xFFE0
#define WHITE 0xFFFF

// keys
#define KEY_W 50
#define KEY_H 40

#define INITKEY_X 85
#define INITKEY_Y 50

void createKeyPad(Adafruit_ILI9341 tft)
{
  tft.setTextSize(3);
  int count = 1;

  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      tft.fillRect(INITKEY_X + j * KEY_W, INITKEY_Y + i * KEY_H, KEY_W, KEY_H, RED);
      tft.drawRect(INITKEY_X + j * KEY_W, INITKEY_Y + i * KEY_H, KEY_W, KEY_H, BLACK);
      tft.setTextColor(WHITE, RED);
      tft.setCursor(INITKEY_X + j * KEY_W + KEY_W / 2 + 2, INITKEY_Y + i * KEY_H + KEY_H / 2 - 10);
      tft.println(count);
      count++;
    }
  }

  tft.fillRect(INITKEY_X + KEY_W, INITKEY_Y + 3 * KEY_H, KEY_W, KEY_H, RED);
  tft.drawRect(INITKEY_X + KEY_W, INITKEY_Y + 3 * KEY_H, KEY_W, KEY_H, BLACK);
  tft.setCursor(INITKEY_X + KEY_W + KEY_W / 2 + 2, INITKEY_Y + 3 * KEY_H + KEY_H / 2 - 10);
  tft.println(0);
}

void keypadHandler(int x, int y)
{
}
