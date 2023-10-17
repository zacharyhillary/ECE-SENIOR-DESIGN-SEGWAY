#ifndef SLIDERS_H
#define SLIDERS_H

#include <Adafruit_ILI9341.h>

#define TFT_CS 10
#define TFT_DC 9

      // class varHolder{
      //   int pageNum;
      //   volatile bool pFlag;
      //   int currColor;
      //   char keyPadInput[4];
        
      //   public:
      //     varHolder(){
      //       pageNum = 0;
      //       pFlag = false;
      //       currColor = 0;
      //       keyPadInput[4] = { '\0', '\0', '\0', '\0' };
      //     }
          
      // }

      void pageOne(Adafruit_ILI9341 tft);
      void pageTwo(Adafruit_ILI9341 tft);
      void pageThree(Adafruit_ILI9341 tft);
      void buttons(Adafruit_ILI9341 tft);
      void pageNumber(Adafruit_ILI9341 tft);
      void setLabelValue(Adafruit_ILI9341 tft,int value, int slider);
      void sliderHandler(Adafruit_ILI9341 tft,int sliderYPos, int Thumb_Y, int x, int x1, int slider);
      void mainSliderHandler(Adafruit_ILI9341 tft,int pX, int pY);
      void mainLabelHandler(Adafruit_ILI9341 tft);
      void mainLabelHandler(Adafruit_ILI9341 tft, int pX, int pY);
      void buttonHandler(int pX, int pY);
      void updateSliderValue(int newSliderValue);
      void setPageNum(int newPageNum);
      int getPageNum();
      void setCurrColor(int newCurrColor);
      int getCurrColor();
      void setpFlag(bool newpFlag);
      bool getpFlag();
      void setKeyPadInput(char newKeyPadInput[4]);
      char* getKeyPadInput();




#endif