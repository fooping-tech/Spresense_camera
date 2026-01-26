#include <Camera.h>
#include <SDHCI.h>

SDHCI theSD;

static void printCamErr(const char* label, CamErr err) {
  if (err == CAM_ERR_SUCCESS) {
    return;
  }
  Serial.print(label);
  Serial.print(": ");
  Serial.println((int)err);
}

static bool saveJpegToSD(const char* path, CamImage& img) {
  if (!img.isAvailable()) {
    Serial.println("Image not available");
    return false;
  }

  theSD.remove(path);
  File f = theSD.open(path, FILE_WRITE);
  if (!f) {
    Serial.println("Failed to open file");
    return false;
  }

  const uint8_t* buf = img.getImgBuff();
  const uint32_t size = img.getImgSize();
  const uint32_t written = f.write(buf, size);
  f.close();

  if (written != size) {
    Serial.println("Short write");
    return false;
  }

  return true;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  if (!theSD.begin()) {
    Serial.println("SD init failed");
    return;
  }

  CamErr err = theCamera.begin();
  if (err != CAM_ERR_SUCCESS) {
    printCamErr("Camera begin failed", err);
    return;
  }

  err = theCamera.setStillPictureImageFormat(
      CAM_IMGSIZE_VGA_H,
      CAM_IMGSIZE_VGA_V,
      CAM_IMAGE_PIX_FMT_JPG);
  if (err != CAM_ERR_SUCCESS) {
    printCamErr("setStillPictureImageFormat failed", err);
    return;
  }

  Serial.println("Ready to take still pictures");
}

void loop() {
  static int shot = 0;
  if (shot >= 5) {
    theCamera.end();
    Serial.println("Done");
    while (true) {
      delay(1000);
    }
  }

  CamImage img = theCamera.takePicture();
  if (!img.isAvailable()) {
    Serial.println("takePicture failed");
    delay(1000);
    return;
  }

  char path[32];
  snprintf(path, sizeof(path), "PICT%03d.JPG", shot);
  if (saveJpegToSD(path, img)) {
    Serial.print("Saved: ");
    Serial.println(path);
  }

  shot++;
  delay(1000);
}
