#include <Arduino_FreeRTOS.h>
#include <task.h>
#include <Adafruit_GFX.h>
#include <Adafruit_FT6206.h>
#include <Adafruit_ILI9341.h>
#include <SPI.h>
#include "semphr.h"
#include "keypad.h"
#include "mpu_6050_drivers.h"

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
#define SliderX_Left 200
#define SliderY_1 40
#define SliderY_2 92
#define SliderY_3 142

#define Slider_Width 220
#define Slider_Height 5


//thumbs
#define Thumb_H 40
#define Thumb_W 25

#define Thumb1_Y SliderY_1 - Thumb_H / 2
#define Thumb2_Y SliderY_2 - Thumb_H / 2
#define Thumb3_Y SliderY_3 - Thumb_H / 2

int Thumb1_X = SliderX_Right - Thumb_W;
int Thumb2_X = SliderX_Right - Thumb_W;
int Thumb3_X = SliderX_Right - Thumb_W;
double currentAngle;
int currentLED = 0;
const int numLEDs = 3;
const int ledPins[numLEDs] = { RED_LED, BLUE_LED, YELLOW_LED };

boolean RecordOn = false;
volatile int pageNum = 0;
volatile int currColor;
int randomNum = 0;

volatile bool buttonPressed = false;
volatile TS_Point p;

volatile unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 100;

Adafruit_FT6206 ts = Adafruit_FT6206();
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);

TaskHandle_t updateTaskHandle;
TaskHandle_t ledTaskHandle;
TaskHandle_t dataDisplayTask1Handle;
TaskHandle_t dataDisplayTask2Handle;
TaskHandle_t dataDisplayTask3Handle;
TaskHandle_t sliderDisplayTaskHandle;
TaskHandle_t Handle_Get_Angle_Task;
SemaphoreHandle_t xSemaphore;
SemaphoreHandle_t I2CSemaphore;

const int buttonPin = 2;

int sliderValue = 0;
int s1x = SliderX_Right, s1y, s2x = SliderX_Right, s2y, s3x = SliderX_Right, s3y;
int x1 = SliderX_Right, x2 = SliderX_Right, x3 = SliderX_Right;

char sliderValueStr[5];

void ledTask(void* pvParameters) {
  while (1) {
    // Turn off all LEDs
    for (int i = 0; i < numLEDs; i++) {
      digitalWrite(ledPins[i], LOW);
    }

    // Turn on the current LED
    digitalWrite(ledPins[currentLED], HIGH);

    // Increment the current LED index
    currentLED++;
    if (currentLED >= numLEDs) {
      currentLED = 0;
    }

    vTaskDelay(pdMS_TO_TICKS(250));  // Delay for 1 second
  }
}

void drawFrame() {
  tft.drawRect(FRAME_X, FRAME_Y, FRAME_W, FRAME_H, ILI9341_BLACK);
}

void pageOne() {
  tft.fillScreen(ILI9341_BLACK);
  sliderLabelHandler(SliderY_1, Thumb1_Y, s1x, x1, 1);
  sliderLabelHandler(SliderY_2, Thumb2_Y, s2x, x2, 2);
  sliderLabelHandler(SliderY_3, Thumb3_Y, s3x, x3, 3);
  //Serial.println(pageNum);
  currColor = ILI9341_BLACK;
}

void pageTwo() {
  tft.fillScreen(ILI9341_BLUE);
  //Serial.println(pageNum);
  currColor = ILI9341_BLUE;
}

void pageThree() {
  tft.fillScreen(ILI9341_MAROON);
  //Serial.println(pageNum);
  currColor = ILI9341_MAROON;
}

void buttons() {
  tft.fillRect(PREVBUTTON_X, PREVBUTTON_Y, PREVBUTTON_W, PREVBUTTON_H, ILI9341_RED);
  tft.setCursor(PREVBUTTON_X + 6, PREVBUTTON_Y + (PREVBUTTON_H / 2));
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);
  tft.println("NEXT");
  tft.fillRect(NEXTBUTTON_X, NEXTBUTTON_Y, NEXTBUTTON_W, NEXTBUTTON_H, ILI9341_RED);
  tft.setCursor(NEXTBUTTON_X + 6, NEXTBUTTON_Y + (NEXTBUTTON_H / 2));
  tft.println("PREV");
}

void pageNumber() {
  tft.setCursor(PAGENUMBER_X, PAGENUMBER_Y);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);
  tft.print(pageNum + 1);
  tft.println("/3");
}

volatile bool updateTapCounter = false;
void buttonInterrupt() {
  if (((millis() - lastDebounceTime) > debounceDelay) && !buttonPressed) {
    buttonPressed = true;
    //buttonPressed2 = true;
    updateTapCounter = true;

    //Serial.println("Pressed");
    //Serial.println(millis()-lastDebounceTime);
  }

  lastDebounceTime = millis();
}

// This sets the right number value depending on the
// x position of the slider
void setLabelValue(int value, int slider) {
  //Serial.println("Show Slider Value");
  switch (slider) {
    case 1:
      sprintf(sliderValueStr, "%03d", value);
      tft.setCursor(SliderX_Right + Slider_Width + 10, SliderY_1 - 5);
      tft.setTextColor(WHITE);
      tft.setTextColor(WHITE, BLACK);
      tft.setTextSize(2);
      tft.print(sliderValueStr);
      break;
    case 2:
      sprintf(sliderValueStr, "%03d", value);
      tft.setCursor(SliderX_Right + Slider_Width + 10, SliderY_2 - 5);
      tft.setTextColor(WHITE);
      tft.setTextColor(WHITE, BLACK);
      tft.setTextSize(2);
      tft.print(sliderValueStr);
      break;
    case 3:
      sprintf(sliderValueStr, "%03d", value);
      tft.setCursor(SliderX_Right + Slider_Width + 10, SliderY_3 - 5);
      tft.setTextColor(WHITE);
      tft.setTextColor(WHITE, BLACK);
      tft.setTextSize(2);
      tft.print(sliderValueStr);
      break;
    case 4:
      sprintf(sliderValueStr, "%03d", value);
      tft.setCursor(SliderX_Right + Slider_Width + 10, SliderY_3 - 5);
      tft.setTextColor(WHITE);
      tft.setTextColor(WHITE, BLACK);
      tft.setTextSize(2);
      tft.print(sliderValueStr);
      break;
    case 5:
      sprintf(sliderValueStr, "%03d", value);
      tft.setCursor(SliderX_Right + Slider_Width + 10, SliderY_3 - 5);
      tft.setTextColor(WHITE);
      tft.setTextColor(WHITE, BLACK);
      tft.setTextSize(2);
      tft.print(sliderValueStr);
      break;
    case 6:
      sprintf(sliderValueStr, "%03d", value);
      tft.setCursor(SliderX_Right + Slider_Width + 10, SliderY_3 - 5);
      tft.setTextColor(WHITE);
      tft.setTextColor(WHITE, BLACK);
      tft.setTextSize(2);
      tft.print(sliderValueStr);
      break;
  }
}

void sliderLabelHandler(int sliderYPos, int Thumb_Y, int x, int x1, int slider) {
  sliderValue = x - SliderX_Right;  //(x-tft.width()) + (SliderX_Right+Slider_Width-Thumb_W);
  if (sliderValue < 0) sliderValue = 0;
  setLabelValue(sliderValue, slider);
  // erase previous thumb by redrawing with background color
  tft.drawRect(x1, Thumb_Y, Thumb_W, Thumb_H, BLACK);
  // then draw new thumb
  tft.drawRect(x, Thumb_Y, Thumb_W, Thumb_H, WHITE);
  tft.fillRoundRect(SliderX_Right, sliderYPos, Slider_Width, Slider_Height, 2, RED);
}


void dataDisplayTask1(void* pvParameters) {
  while (1) {
    if (pageNum == 0) {
      if (xSemaphoreTake(xSemaphore, pdMS_TO_TICKS(10)) == pdTRUE) {


        tft.fillRect(FRAME_X + 75, FRAME_Y - 100, FRAME_W, FRAME_H, currColor);
        int randomNumber = random(10);
        tft.setTextSize(4);

        tft.setCursor(FRAME_X + 75, FRAME_Y - 100);
        tft.print(randomNumber);

        Serial.println("data task 1");

        xSemaphoreGive(xSemaphore);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(250));
  }
}

static void Get_Angle_Task(void* pvParameters) {
  while (1) {

    xSemaphoreTake(I2CSemaphore, pdMS_TO_TICKS(100));
    currentAngle = Get_Angle();  // set global variable to the current angle of the segway
    xSemaphoreGive(I2CSemaphore);

    Serial.print(currentAngle);
    Serial.print("\n");
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}




volatile int touchCount = 0;
void dataDisplayTask2(void* pvParameters) {
  while (1) {
    if (pageNum == 1 && updateTapCounter) {
      if ((xSemaphoreTake(xSemaphore, pdMS_TO_TICKS(100)) == pdTRUE)) {
        tft.fillRect(FRAME_X + 75, FRAME_Y - 100, FRAME_W, FRAME_H, currColor);
        //int randomNumber = random(10);
        tft.setTextSize(4);

        tft.setCursor(FRAME_X + 75, FRAME_Y - 100);
        tft.print(touchCount);

        buttonPressed = false;
        Serial.println("data task 2");
        updateTapCounter = false;
        xSemaphoreGive(xSemaphore);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

volatile int time = 0;
void dataDisplayTask3(void* pvParameters) {
  while (1) {
    if (pageNum == 2) {



      tft.fillRect(FRAME_X + 75, FRAME_Y - 100, FRAME_W, FRAME_H, currColor);

      tft.setTextSize(4);

      tft.setCursor(FRAME_X + 75, FRAME_Y - 100);
      tft.print(currentAngle);
      //Serial.println(Get_Angle());

      //Serial.println("data task 3");
    }
    vTaskDelay(pdMS_TO_TICKS(1000));

    time++;
  }
}

void mainSliderHandler(int pX, int pY) {
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
      sliderLabelHandler(SliderY_1, Thumb1_Y, s1x, x1, 1);
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
      sliderLabelHandler(SliderY_2, Thumb2_Y, s2x, x2, 2);
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
      sliderLabelHandler(SliderY_3, Thumb3_Y, s3x, x3, 3);
      x3 = s3x;
    }
  }
}

void mainLabelHandler(int pX, int pY) {
}


void updateScreenTask(void* pvParameters) {
  bool touchInProgress = false;
  while (1) {
    if (buttonPressed) {
      // Serial.println(millis() - lastDebounceTime);
      // Serial.println(buttonPressed);
      // Serial.println("Touched");
      touchCount++;


      // if((xSemaphoreTake(xSemaphore, pdMS_TO_TICKS(200)) == pdTRUE)){
      // Serial.println("update");
      xSemaphoreTake(I2CSemaphore, pdMS_TO_TICKS(100));
      p = ts.getPoint();
      xSemaphoreGive(I2CSemaphore);
      int pY = 240 - p.x;
      int pX = p.y;
      bool pFlag = 0;

      // Slider position parsing and then handling
      if (pageNum == 0) {
        mainSliderHandler(pX, pY);
        mainLabelHandler(pX, pY);
      }

      // button handling
      if ((pX > PREVBUTTON_X) && (pX < (PREVBUTTON_X + PREVBUTTON_W))) {
        if ((pY > PREVBUTTON_Y) && (pY <= (PREVBUTTON_Y + PREVBUTTON_H))) {
          pageNum = (pageNum + 1) % 3;
          pFlag = true;
        }
      }
      if ((pX > NEXTBUTTON_X) && (pX < (NEXTBUTTON_X + NEXTBUTTON_W))) {
        if ((pY > NEXTBUTTON_Y) && (pY <= (NEXTBUTTON_Y + NEXTBUTTON_H))) {
          pageNum = (pageNum + 2) % 3;
          pFlag = true;
        }
      }

      if (pageNum == 0 && pFlag) {
        pageOne();
      } else if (pageNum == 1 && pFlag) {
        pageTwo();
        createKeyPad(tft);
      } else if (pageNum == 2 && pFlag) {
        pageThree();
      }

      if (pFlag) {
        buttons();
        //pageNumber();
        buttonPressed = false;
      }
      pFlag = false;
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}





void setup() {
  Serial.begin(115200);
  tft.begin();
  tft.setRotation(3);
  tft.fillScreen(ILI9341_BLACK);

  ts.begin();
  mpu_setup();


  //pageOne();
  pageThree();
  pageNum = 2;
  pageNumber();
  buttons();
  Serial.println("Touchscreen Started");

  attachInterrupt(digitalPinToInterrupt(2), buttonInterrupt, FALLING);

  pinMode(RED_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);

  xSemaphore = xSemaphoreCreateBinary();
  I2CSemaphore = xSemaphoreCreateBinary();
  xSemaphoreGive(xSemaphore);
  Serial.println("Semaphore created");

  xTaskCreate(Get_Angle_Task, "Get_Angle_Task", configMINIMAL_STACK_SIZE * 4, NULL, 1, &Handle_Get_Angle_Task);  //create task
  // Create the update screen task
  xTaskCreate(updateScreenTask, "UpdateScreenTask", configMINIMAL_STACK_SIZE * 4, NULL, 2, &updateTaskHandle);
  Serial.println("update task created");

  xTaskCreate(ledTask, "LEDTask", configMINIMAL_STACK_SIZE * 4, NULL, 4, &ledTaskHandle);
  Serial.println("LED task created");

  //xTaskCreate(dataDisplayTask1, "dataDisplayTask1", configMINIMAL_STACK_SIZE*4, NULL, 2, &dataDisplayTask1Handle);
  //Serial.println("data display 1 task created");

  //xTaskCreate(dataDisplayTask2, "dataDisplayTask2", configMINIMAL_STACK_SIZE*4, NULL, 2, &dataDisplayTask2Handle);
  //Serial.println("data display 2 task created");

  xTaskCreate(dataDisplayTask3, "dataDisplayTask3", configMINIMAL_STACK_SIZE * 4, NULL, 3, &dataDisplayTask3Handle);
  Serial.println("data display 3 task created");

  //xTaskCreate(sliderDisplayTask, "sliderDisplayTask", configMINIMAL_STACK_SIZE*4, NULL, 4, &sliderDisplayTaskHandle);
  //Serial.println("slider display task created");

  vTaskStartScheduler();
}

void loop() {
  // Main Arduino loop, not used in this example
}
