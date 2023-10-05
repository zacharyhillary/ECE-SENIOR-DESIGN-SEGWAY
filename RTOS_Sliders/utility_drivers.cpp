#include <Adafruit_ILI9341.h>
#include <Adafruit_GFX.h>
#include <Adafruit_FT6206.h>

#define MAGENTA 0xF81F

void areaDebugger(Adafruit_ILI9341 tft, int x, int y, int width, int height) {
    tft.drawRect(x, y, width, height, MAGENTA);
}

void areaDebugger(Adafruit_ILI9341 tft, TS_Point p, int width, int height) {
    int x = p.y;
    int y = 240 - p.x;
    tft.drawRect(x, y, width, height, MAGENTA);
}

void doClickOnArea(TS_Point p, int x, int y, int width, int height, void *func()) {
    int pY = 240 - p.x;
    int pX = p.y;
    // if(pX > LabelX_Left && pX < LabelX_Left + Thumb_W * 1.2 && pY > (Thumb2_Y + Thumb_H / 4 - Thumb_H / 1.7) && pY < (Thumb2_Y + Thumb_H / 4)) {
    if(x < pX && pX < x + width && y - height < pY && pY < y) {
        func();
    }
}

void doClickOnArea(int pX, int pY, int x, int y, int width, int height, void *func()) {
    // if(pX > LabelX_Left && pX < LabelX_Left + Thumb_W * 1.2 && pY > (Thumb2_Y + Thumb_H / 4 - Thumb_H / 1.7) && pY < (Thumb2_Y + Thumb_H / 4)) {
    if(x < pX && pX < x + width && y - height < pY && pY < y) {
        func();
    }
}