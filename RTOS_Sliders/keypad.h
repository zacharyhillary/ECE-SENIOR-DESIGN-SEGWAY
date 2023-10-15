#ifndef KEYPAD_H
#define KEYPAD_H
#include <Adafruit_ILI9341.h>

void createKeyPad(Adafruit_ILI9341 tft);
void appendInt(char input, char array[]);
int keypadHandler(Adafruit_ILI9341 tft, int pX, int pY, char keyInput[]);
int setValue(int value);
double inputArrayToDecimal(char array[]);
void printArray(char array[]);

#endif