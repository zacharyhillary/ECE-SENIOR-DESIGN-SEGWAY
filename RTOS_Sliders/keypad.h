#ifndef KEYPAD_H
#define KEYPAD_H
#include <Adafruit_ILI9341.h>

void createKeyPad(Adafruit_ILI9341 tft);
void appendInt(int input, int array[]);
int keypadHandler(Adafruit_ILI9341 tft, int x, int y, int keyInput[]);
int setValue(int value);

#endif