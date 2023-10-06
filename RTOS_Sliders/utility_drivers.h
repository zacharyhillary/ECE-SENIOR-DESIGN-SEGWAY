#ifndef UTIL_DRIVER
#define utility_drivers
#include <Adafruit_ILI9341.h>

void areaDebugger(Adafruit_ILI9341 tft, int x, int y, int width, int height);
void areaDebugger(Adafruit_ILI9341 tft, TS_Point p, int width, int height);
void doClickOnArea(TS_Point p, int x, int y, int width, int height, void *func());
void doClickOnArea(int pX, int pY, int x, int y, int width, int height, void *func());
bool checkAreaClicked(int pX, int pY, int x, int y, int width, int height);

#endif