#include <Adafruit_ILI9341.h>
#include <Adafruit_GFX.h>
#include <Adafruit_FT6206.h>
// #include <functional>
// #include <iostream>
// using namespace std;
#define MAGENTA 0xF81F
void areaDebugger(Adafruit_ILI9341 tft, int x, int y, int width, int height) {
    tft.drawRect(x, y, width, height, MAGENTA);
}

// void TS_Point::doClickOnArea(int x, int y, int width, int height, std::function<void(int)> func) {
//     int pY = 240 - this.x;
//     int pX = this.y;
//     // if(pX > LabelX_Left && pX < LabelX_Left + Thumb_W * 1.2 && pY > (Thumb2_Y + Thumb_H / 4 - Thumb_H / 1.7) && pY < (Thumb2_Y + Thumb_H / 4)) {
//     if(x < pX && pX < x + width && y < pY && pY < y - height) {
//         func(0);
//     }
// }