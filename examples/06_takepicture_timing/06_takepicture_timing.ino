#include <Camera.h>

static void printCamErr(const char* label, CamErr err) {
  if (err == CAM_ERR_SUCCESS) {
    return;
  }
  Serial.print(label);
  Serial.print(": ");
  Serial.println((int)err);
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  CamErr err = theCamera.begin();
  if (err != CAM_ERR_SUCCESS) {
    printCamErr("Camera begin failed", err);
    return;
  }

  err = theCamera.setJPEGQuality(50);
  if (err != CAM_ERR_SUCCESS) {
    printCamErr("setJPEGQuality failed", err);
    return;
  }

  err = theCamera.setStillPictureImageFormat(
      CAM_IMGSIZE_QVGA_H,
      CAM_IMGSIZE_QVGA_V,
      CAM_IMAGE_PIX_FMT_JPG);
  if (err != CAM_ERR_SUCCESS) {
    printCamErr("setStillPictureImageFormat failed", err);
    return;
  }

  Serial.println("Ready to measure takePicture()");
}

void loop() {
  const uint32_t start_ms = millis();
  CamImage img = theCamera.takePicture();
  const uint32_t elapsed_ms = millis() - start_ms;

  if (!img.isAvailable()) {
    Serial.print("takePicture failed, elapsed=");
    Serial.print(elapsed_ms);
    Serial.println(" ms");
    delay(1000);
    return;
  }

  Serial.print("takePicture ok, elapsed=");
  Serial.print(elapsed_ms);
  Serial.print(" ms, size=");
  Serial.print(img.getImgSize());
  Serial.println(" bytes");

  delay(1000);
}
