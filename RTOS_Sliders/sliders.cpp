#include <Adafruit_GFX.h>
#include <Adafruit_FT6206.h>
#include <Adafruit_ILI9341.h>
#include "utility_drivers.h"
#include "keypad.h"


#define TFT_CLK 52
#define TFT_MISO 50
#define TFT_MOSI 51
#define TFT_CS 10
//#define TFT_RST 8
#define TFT_DC 9

#define FRAME_X 10
#define FRAME_Y 180
#define FRAME_W 150
#define FRAME_H 50


#define NEXTBUTTON_X FRAME_X
#define NEXTBUTTON_Y FRAME_Y
#define NEXTBUTTON_W (FRAME_W / 2)
#define NEXTBUTTON_H FRAME_H

#define PREVBUTTON_X (NEXTBUTTON_X + NEXTBUTTON_W + 150)
#define PREVBUTTON_Y FRAME_Y
#define PREVBUTTON_W (FRAME_W / 2)
#define PREVBUTTON_H FRAME_H

#define PAGENUMBER_X (NEXTBUTTON_X + NEXTBUTTON_W + 55)
#define PAGENUMBER_Y (FRAME_Y + 20)

#define RED_LED 11
#define BLUE_LED 12
#define YELLOW_LED 13

// Common 16-bit color values:
#define BLACK 0x0000
#define BLUE 0x001F
#define RED 0xF800
#define GREEN 0x07E0
#define CYAN 0x07FF
#define MAGENTA 0xF81F
#define YELLOW 0xFFE0
#define WHITE 0xFFFF

//Sliders
#define SliderX_Right 40
#define SliderX_Left 240  // Larger x values == more left
#define SliderY_1 40      // smaller y values == more up + larger y values == more down
#define SliderY_2 92
#define SliderY_3 142

//Labels
#define LabelX_Right SliderX_Right + Slider_Width + Thumb_W / 2
#define LabelX_Left 215 - SliderX_Left + Thumb_W

#define Slider_Width 220
#define Slider_Height 5

// Labels
double sliderOne = 15.0;
double sliderTwo = 15.0;
double sliderThree = 15.0;
double sliderFour = 0.0;
double sliderFive = 0.0;
double sliderSix = 0.0;

//thumbs
#define Thumb_H 40
#define Thumb_W 25

#define Thumb1_Y SliderY_1 - Thumb_H / 2
#define Thumb2_Y SliderY_2 - Thumb_H / 2
#define Thumb3_Y SliderY_3 - Thumb_H / 2

int Thumb1_X = SliderX_Right - Thumb_W;
int Thumb2_X = SliderX_Right - Thumb_W;
int Thumb3_X = SliderX_Right - Thumb_W;
// double currentAngle;
// double kp = 0;
// double ki = 0;
// double kd = 0;
// int currentLED = 0;
// const int numLEDs = 3;
// const int ledPins[numLEDs] = { RED_LED, BLUE_LED, YELLOW_LED };

//boolean RecordOn = false;
volatile int pageNum = 0;
char keyPadInput[4] = { '\0', '\0', '\0', '\0' };
volatile int currColor;
volatile bool pFlag = 0;
// int randomNum = 0;

// volatile bool buttonPressed = false;
// volatile TS_Point p;

int sliderValue = 0;
int s1x = SliderX_Right, s1y, s2x = SliderX_Right, s2y, s3x = SliderX_Right, s3y;
int x1 = SliderX_Right, x2 = SliderX_Right, x3 = SliderX_Right;

char sliderValueStr[5];


tft(){

};

tft(int CS, int DC){
  Adafruit_ILI9341 tft = Adafruit_ILI9341(CS, DC);
};

// void drawFrame() {
//   tft.drawRect(FRAME_X, FRAME_Y, FRAME_W, FRAME_H, ILI9341_BLACK);
// }

void sliderHandler(Adafruit_ILI9341 tft, int sliderYPos, int Thumb_Y, int x, int x1, int slider);
void mainLabelHandler(Adafruit_ILI9341 tft);

void pageOne(Adafruit_ILI9341 tft) {
  tft.fillScreen(ILI9341_BLACK);
  mainLabelHandler(tft);
  sliderHandler(tft, SliderY_1, Thumb1_Y, s1x, x1, 1);
  sliderHandler(tft, SliderY_2, Thumb2_Y, s2x, x2, 2);
  sliderHandler(tft, SliderY_3, Thumb3_Y, s3x, x3, 3);
  //Serial.println(pageNum);
  currColor = ILI9341_BLACK;
}

void pageThree(Adafruit_ILI9341 tft) {
  tft.fillScreen(WHITE);
  //Serial.println(pageNum);
  currColor = WHITE;
}

void pageTwo(Adafruit_ILI9341 tft) {
  tft.fillScreen(ILI9341_MAROON);
  //Serial.println(pageNum);
  currColor = ILI9341_MAROON;
}

void buttons(Adafruit_ILI9341 tft) {
  tft.fillRect(PREVBUTTON_X, PREVBUTTON_Y, PREVBUTTON_W, PREVBUTTON_H, ILI9341_RED);
  tft.setCursor(PREVBUTTON_X + 6, PREVBUTTON_Y + (PREVBUTTON_H / 2));
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);
  tft.println("NEXT");
  tft.fillRect(NEXTBUTTON_X, NEXTBUTTON_Y, NEXTBUTTON_W, NEXTBUTTON_H, ILI9341_RED);
  tft.setCursor(NEXTBUTTON_X + 6, NEXTBUTTON_Y + (NEXTBUTTON_H / 2));
  tft.println("PREV");
}

void pageNumber(Adafruit_ILI9341 tft) {
  tft.setCursor(PAGENUMBER_X, PAGENUMBER_Y);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);
  tft.print(pageNum + 1);
  tft.println("/2");
}


void setLabelValue(Adafruit_ILI9341 tft, int value, int slider) {
  //Serial.println("Show Slider Value");
  int sliderX;
  int sliderY;
  switch (slider) {
    case 1: {
        sliderX = SliderX_Right;
        sliderY = SliderY_1;
        break;
      }
    case 2: {
        sliderX = SliderX_Right;
        sliderY = SliderY_2;
        break;
      }
    case 3: {
        sliderX = SliderX_Right;
        sliderY = SliderY_3;
        break;
      }
    case 4: {
        sliderX = SliderX_Left;
        sliderY = SliderY_1 - 15;
        break;
      }
    case 5: {
        sliderX = SliderX_Left;
        sliderY = SliderY_2 - 15;
        break;
      }
    case 6: {
        sliderX = SliderX_Left;
        sliderY = SliderY_3 - 15;
        break;
      }
  }
  sprintf(sliderValueStr, "%03d", value);
  tft.setCursor(sliderX + Slider_Width + 10, sliderY - 5);
  tft.setTextColor(WHITE, BLACK);
  tft.setTextSize(2);
  tft.print(sliderValueStr);
}

void sliderHandler(Adafruit_ILI9341 tft, int sliderYPos, int Thumb_Y, int x, int x1, int slider) {
  char sliderLabel;
  switch (slider) {
    case 1: sliderLabel = 'P'; break;
    case 2: sliderLabel = 'I'; break;
    case 3: sliderLabel = 'D'; break;
  }
  sliderValue = x - SliderX_Right;  //(x-tft.width()) + (SliderX_Right+Slider_Width-Thumb_W);
  if (sliderValue < 0) sliderValue = 0;
  // setLabelValue(sliderValue, slider);
  // erase previous thumb by redrawing with background color
  tft.fillRect(x1, Thumb_Y, Thumb_W, Thumb_H, BLACK);
  // then draw new thumb
  tft.fillRoundRect(SliderX_Right, sliderYPos, Slider_Width, Slider_Height, 2, RED);
  tft.fillRect(x, Thumb_Y, Thumb_W, Thumb_H, BLACK);
  tft.drawRect(x, Thumb_Y, Thumb_W, Thumb_H, WHITE);
  tft.setTextSize(3);
  tft.setTextColor(WHITE);
  tft.setCursor(x + 5, Thumb_Y + 10);
  tft.print(sliderLabel);
}

void mainSliderHandler(Adafruit_ILI9341 tft, int pX, int pY) {
  // slider1 handling
  if (pX > SliderX_Right && pX < (SliderX_Right + Slider_Width + Thumb_W)) {
    if (pY > Thumb1_Y && pY < (Thumb1_Y + Thumb_H)) {
      s1x = pX;
      s1y = pY;
      Thumb1_X = s1x - Thumb_W / 2;
      if (Thumb1_X < SliderX_Right) {
        s1x = SliderX_Right;
      } else if ((Thumb1_X + Thumb_W) > (SliderX_Right + Slider_Width)) {
        s1x = (SliderX_Right + Slider_Width) - Thumb_W;
      }
      sliderHandler(tft, SliderY_1, Thumb1_Y, s1x, x1, 1);
      x1 = s1x;
    }
  }
  //Slider 2 Handling
  if (pX > SliderX_Right && pX < (SliderX_Right + Slider_Width + Thumb_W)) {
    if (pY > Thumb2_Y && pY < (Thumb2_Y + Thumb_H)) {
      s2x = pX;
      s2y = pY;
      Thumb2_X = s2x - Thumb_W / 2;
      if (Thumb2_X < SliderX_Right) {
        s2x = SliderX_Right;
      } else if ((Thumb2_X + Thumb_W) > (SliderX_Right + Slider_Width)) {
        s2x = (SliderX_Right + Slider_Width) - Thumb_W;
      }
      sliderHandler(tft, SliderY_2, Thumb2_Y, s2x, x2, 2);
      x2 = s2x;
    }
  }
  //Slider 3 Handling
  if (pX > SliderX_Right && pX < (SliderX_Right + Slider_Width + Thumb_W)) {
    if (pY > Thumb3_Y && pY < (Thumb3_Y + Thumb_H)) {
      s3x = pX;
      s3y = pY;
      Thumb3_X = s3x - Thumb_W / 2;
      if (Thumb3_X < SliderX_Right) {
        s3x = SliderX_Right;
      } else if ((Thumb3_X + Thumb_W) > (SliderX_Right + Slider_Width)) {
        s3x = (SliderX_Right + Slider_Width) - Thumb_W;
      }
      sliderHandler(tft, SliderY_3, Thumb3_Y, s3x, x3, 3);
      x3 = s3x;
    }
  }
}

int selectedLabel;
void mainLabelHandler(Adafruit_ILI9341 tft) {
  setLabelValue(tft, sliderOne, 1);
  setLabelValue(tft, sliderTwo, 2);
  setLabelValue(tft, sliderThree, 3);
  setLabelValue(tft, sliderFour, 4);
  setLabelValue(tft, sliderFive, 5);
  setLabelValue(tft, sliderSix, 6);
}

#define LabelX_Right SliderX_Right + Slider_Width + Thumb_W / 2
#define LabelX_Left 215 - SliderX_Left + Thumb_W
#define Label_Width Thumb_W * 1.2
#define Label_Height Thumb_H / 1.7
#define LabelY_1 Thumb1_Y + (Thumb_H / 4.0)
#define LabelY_2 Thumb2_Y + (Thumb_H / 4.0)
#define LabelY_3 Thumb3_Y + (Thumb_H / 4.0)
void mainLabelHandler(Adafruit_ILI9341 tft, int pX, int pY) {
  areaDebugger(tft, LabelX_Right, LabelY_1, Label_Width, Label_Height);
  areaDebugger(tft, LabelX_Right, LabelY_2, Label_Width, Label_Height);
  areaDebugger(tft, LabelX_Right, LabelY_3, Label_Width, Label_Height);
  areaDebugger(tft, LabelX_Left, LabelY_1, Label_Width, Label_Height);
  areaDebugger(tft, LabelX_Left, LabelY_2, Label_Width, Label_Height);
  areaDebugger(tft, LabelX_Left, LabelY_3, Label_Width, Label_Height);
  selectedLabel = -1;
  doClickOnArea(pX, pY, LabelX_Right, LabelY_1, Label_Width, Label_Height, []() -> void {
    selectedLabel = 1;
    Serial.println("Label 1 pressed");
  });
  doClickOnArea(pX, pY, LabelX_Right, LabelY_2, Label_Width, Label_Height, []() -> void {
    selectedLabel = 2;
    Serial.println("Label 2 pressed");
  });
  doClickOnArea(pX, pY, LabelX_Right, LabelY_3, Label_Width, Label_Height, []() -> void {
    selectedLabel = 3;
    Serial.println("Label 3 pressed");
  });
  doClickOnArea(pX, pY, LabelX_Left, LabelY_1, Label_Width, Label_Height, []() -> void {
    selectedLabel = 4;
    Serial.println("Label 4 pressed");
  });
  doClickOnArea(pX, pY, LabelX_Left, LabelY_2, Label_Width, Label_Height, []() -> void {
    selectedLabel = 5;
    Serial.println("Label 5 pressed");
  });
  doClickOnArea(pX, pY, LabelX_Left, LabelY_3, Label_Width, Label_Height, []() -> void {
    selectedLabel = 6;
    Serial.println("Label 6 pressed");
  });
  if (selectedLabel != -1 && pageNum != 2) {
    Serial.print("Display pageThree with selected label: ");
    Serial.println(selectedLabel);
    pageNum = 2;
    keyPadInput[0] = -1;
    keyPadInput[1] = -1;
    keyPadInput[2] = -1;
    pFlag = true;
    // display pageThree somehow lol
  }
  setLabelValue(tft, sliderOne, 1);
  setLabelValue(tft, sliderTwo, 2);
  setLabelValue(tft, sliderThree, 3);
  setLabelValue(tft, sliderFour, 4);
  setLabelValue(tft, sliderFive, 5);
  setLabelValue(tft, sliderSix, 6);
}

void buttonHandler(int pX, int pY) {
  if (pageNum == 2) return;
  // button handling
  if ((pX > PREVBUTTON_X) && (pX < (PREVBUTTON_X + PREVBUTTON_W))) {
    if ((pY > PREVBUTTON_Y) && (pY <= (PREVBUTTON_Y + PREVBUTTON_H)) && pageNum != 2) {
      pageNum = pageNum == 0 ? 1 : 0;
      pFlag = true;
    }
  }
  if ((pX > NEXTBUTTON_X) && (pX < (NEXTBUTTON_X + NEXTBUTTON_W))) {
    if ((pY > NEXTBUTTON_Y) && (pY <= (NEXTBUTTON_Y + NEXTBUTTON_H)) && pageNum != 2) {
      pageNum = pageNum == 0 ? 1 : 0;
      pFlag = true;
    }
  }
}

void updateSliderValue(int newSliderValue){
  switch (selectedLabel) {
            case 1: 
                sliderOne = newSliderValue; 
                break;
            case 2: 
                sliderTwo = newSliderValue; 
                break;
            case 3: 
                sliderThree = newSliderValue; 
                break;
            case 4: 
                sliderFour = newSliderValue; 
                break;
            case 5: 
                sliderFive = newSliderValue; 
                break;
            case 6: 
                sliderSix = newSliderValue; 
                break;
          }
}

void setPageNum(int newPageNum){
  pageNum = newPageNum;
}
int getPageNum(){
  return pageNum;
}
void setCurrColor(int newCurrColor){
  currColor = newCurrColor;
}

int getCurrColor(){
  return currColor;
}
void setpFlag(bool newpFlag){
  pFlag = newpFlag;
}
bool getpFlag(){
  return pFlag;
}
void setKeyPadInput(char newKeyPadInput[4]){
  *keyPadInput = newKeyPadInput;
}
char* getKeyPadInput(){
  return keyPadInput;
}













