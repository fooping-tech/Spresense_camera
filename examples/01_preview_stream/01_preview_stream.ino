#include <Camera.h>

static volatile uint32_t g_frame_count = 0;
static volatile bool g_info_ready = false;
static uint16_t g_width = 0;
static uint16_t g_height = 0;
static int g_format = CAM_IMAGE_PIX_FMT_NONE;

static const char* formatToString(int fmt) {
  switch (fmt) {
    case CAM_IMAGE_PIX_FMT_RGB565:
      return "RGB565";
    case CAM_IMAGE_PIX_FMT_YUV422:
      return "YUV422";
    case CAM_IMAGE_PIX_FMT_JPG:
      return "JPEG";
    case CAM_IMAGE_PIX_FMT_GRAY:
      return "GRAY";
    default:
      return "UNKNOWN";
  }
}

static void CamCB(CamImage img) {
  if (!img.isAvailable()) {
    return;
  }

  if (!g_info_ready) {
    g_width = img.getWidth();
    g_height = img.getHeight();
    g_format = img.getPixFormat();
    g_info_ready = true;
  }

  g_frame_count++;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  CamErr err = theCamera.begin(
      1,
      CAM_VIDEO_FPS_30,
      CAM_IMGSIZE_QVGA_H,
      CAM_IMGSIZE_QVGA_V,
      CAM_IMAGE_PIX_FMT_YUV422);
  if (err != CAM_ERR_SUCCESS) {
    Serial.print("Camera begin failed: ");
    Serial.println((int)err);
    return;
  }

  err = theCamera.startStreaming(true, CamCB);
  if (err != CAM_ERR_SUCCESS) {
    Serial.print("startStreaming failed: ");
    Serial.println((int)err);
    return;
  }

  Serial.println("Streaming started");
}

void loop() {
  static uint32_t last_report_ms = 0;
  const uint32_t now = millis();

  if (g_info_ready) {
    g_info_ready = false;
    Serial.print("Preview: ");
    Serial.print(g_width);
    Serial.print("x");
    Serial.print(g_height);
    Serial.print(" ");
    Serial.println(formatToString(g_format));
  }

  if (now - last_report_ms >= 1000) {
    last_report_ms = now;
    Serial.print("Frames: ");
    Serial.println(g_frame_count);
  }

  delay(10);
}
