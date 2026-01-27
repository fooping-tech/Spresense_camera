#include <SDHCI.h>
#include <stdio.h>  /* for sprintf */

#include <Camera.h>

#define BAUDRATE            (115200)
#define TOTAL_PICTURE_COUNT (1)

SDClass theSD;
int take_picture_count = 0;
int value = 0;
bool CamReady = true;
bool CamDone = false;
const int DIN_PIN = 7;

// TFT settings
#include <SPI.h>
#include "Adafruit_ILI9341.h"

/* LCD Settings */
#define TFT_RST 8
#define TFT_DC  9
#define TFT_CS  10

Adafruit_ILI9341 tft = Adafruit_ILI9341(&SPI, TFT_DC, TFT_CS, TFT_RST);

void putStringOnLcd(String str, int color)
{
  int len = str.length();
  tft.fillRect(0, 224, 320, 16, ILI9341_BLACK);
  tft.setTextSize(2);
  int sx = 160 - (len * 12) / 2;
  if (sx < 0) sx = 0;
  tft.setCursor(sx, 225);
  tft.setTextColor(color);
  tft.println(str);
}

/**
 * Print error message
 */
void printError(enum CamErr err)
{
  Serial.print("Error: ");
  switch (err)
  {
    case CAM_ERR_NO_DEVICE:
      Serial.println("No Device");
      break;
    case CAM_ERR_ILLEGAL_DEVERR:
      Serial.println("Illegal device error");
      break;
    case CAM_ERR_ALREADY_INITIALIZED:
      Serial.println("Already initialized");
      break;
    case CAM_ERR_NOT_INITIALIZED:
      Serial.println("Not initialized");
      break;
    case CAM_ERR_NOT_STILL_INITIALIZED:
      Serial.println("Still picture not initialized");
      break;
    case CAM_ERR_CANT_CREATE_THREAD:
      Serial.println("Failed to create thread");
      break;
    case CAM_ERR_INVALID_PARAM:
      Serial.println("Invalid parameter");
      break;
    case CAM_ERR_NO_MEMORY:
      Serial.println("No memory");
      break;
    case CAM_ERR_USR_INUSED:
      Serial.println("Buffer already in use");
      break;
    case CAM_ERR_NOT_PERMITTED:
      Serial.println("Operation not permitted");
      break;
    default:
      break;
  }
}

/**
 * Callback from Camera library when video frame is captured.
 */
void CamCB(CamImage img)
{
  /* Check the img instance is available or not. */
  if (img.isAvailable())
  {
    /* If you want RGB565 data, convert image data format to RGB565 */
    img.convertPixFormat(CAM_IMAGE_PIX_FMT_RGB565);

    Serial.print("Image data size = ");
    Serial.print(img.getImgSize(), DEC);
    Serial.print(" , ");

    Serial.print("buff addr = ");
    Serial.print((unsigned long)img.getImgBuff(), HEX);
    Serial.println("");

    String gStrResult = "Ready";
    if (CamDone)
    {
      gStrResult = "PIC=OK";
      CamDone = false;
    }

    // TFT draw
    tft.drawRGBBitmap(0, 0, (uint16_t *)img.getImgBuff(), 320, 224);
    putStringOnLcd(gStrResult, ILI9341_YELLOW);
  }
  else
  {
    Serial.println("Failed to get video stream image");
  }
}

/**
 * @brief Initialize camera
 */
void setup()
{
  // Switch input
  pinMode(DIN_PIN, INPUT_PULLUP);

  // TFT
  tft.begin();
  tft.setRotation(1);

  CamErr err;

  /* Open serial communications and wait for port to open */
  Serial.begin(BAUDRATE);
  while (!Serial)
  {
    ; /* wait for serial port to connect. Needed for native USB port only */
  }

  /* Initialize SD */
  while (!theSD.begin())
  {
    /* wait until SD card is mounted. */
    Serial.println("Insert SD card.");
  }

  /* begin() without parameters means that
   * number of buffers = 1, 30FPS, QVGA, YUV 4:2:2 format */
  Serial.println("Prepare camera");
  err = theCamera.begin();
  if (err != CAM_ERR_SUCCESS)
  {
    printError(err);
  }

  /* Start video stream.
   * If received video stream data from camera device,
   * camera library call CamCB. */
  Serial.println("Start streaming");
  err = theCamera.startStreaming(true, CamCB);
  if (err != CAM_ERR_SUCCESS)
  {
    printError(err);
  }

  /* Auto white balance configuration */
  Serial.println("Set Auto white balance parameter");
  err = theCamera.setAutoWhiteBalanceMode(CAM_WHITE_BALANCE_DAYLIGHT);
  if (err != CAM_ERR_SUCCESS)
  {
    printError(err);
  }

  Serial.println("Set still picture format");
  err = theCamera.setStillPictureImageFormat(
    CAM_IMGSIZE_QUADVGA_H,
    CAM_IMGSIZE_QUADVGA_V,
    CAM_IMAGE_PIX_FMT_JPG);
  if (err != CAM_ERR_SUCCESS)
  {
    printError(err);
  }
}

void CheckSW()
{
  value = digitalRead(DIN_PIN);
  Serial.println(value);
  if (value == 0)
  {
    CamReady = true;
  }
}

void loop()
{
  delay(500);
  CheckSW();

  if (CamReady)
  {
    Serial.println("call takePicture()");
    CamImage img = theCamera.takePicture();

    if (img.isAvailable())
    {
      /* Create file name */
      char filename[16] = {0};
      sprintf(filename, "PICT_%03d.JPG", take_picture_count);

      Serial.print("Save taken picture as ");
      Serial.print(filename);
      Serial.println("");

      theSD.remove(filename);
      File myFile = theSD.open(filename, FILE_WRITE);
      myFile.write(img.getImgBuff(), img.getImgSize());
      myFile.close();

      take_picture_count++;
      CamDone = true;
    }
    else
    {
      Serial.println("Failed to take picture");
    }

    CamReady = false;
  }
}
