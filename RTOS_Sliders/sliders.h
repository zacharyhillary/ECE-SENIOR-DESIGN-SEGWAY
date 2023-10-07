#ifndef SLIDERS_H
#define SLIDERS_H

#include <Adafruit_ILI9341.h>

void pageOne(Adafruit_ILI9341 tft);
void pageTwo(Adafruit_ILI9341 tft);
void pageThree(Adafruit_ILI9341 tft);
void buttons(Adafruit_ILI9341 tft);
void pageNumbers(Adafruit_ILI9341 tft);
void setLabelValue(Adafruit_ILI9341 tft,int value, int slider);
void sliderHandler(Adafruit_ILI9341 tft,int sliderYPos, int Thumb_Y, int x, int x1, int slider);
void mainSliderHandler(int pX, int pY);
void buttonHandler(int pX, int pY);

#endif