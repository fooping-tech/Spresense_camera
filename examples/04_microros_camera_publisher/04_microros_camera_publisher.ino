#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <rmw_microros/rmw_microros.h>

#include <sensor_msgs/msg/compressed_image.h>
#include <std_msgs/msg/header.h>
#include <rosidl_runtime_c/string_functions.h>

#include <Camera.h>
#include <string.h>

static const uint8_t kLedPins[] = {LED0, LED1, LED2, LED3};

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { error_loop(); } }
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { } }

static rcl_publisher_t publisher;
static rclc_support_t support;
static rcl_allocator_t allocator;
static rcl_node_t node;

static sensor_msgs__msg__CompressedImage msg;

// VGA + JPEG quality 50 is typically well under 160KB; increase if "Image too large" appears.
static uint8_t g_jpeg_buffer[160 * 1024];
static const char* kTopicName = "spresense/camera/compressed";
static const uint32_t kMinIntervalMs = 10;
static uint32_t g_last_capture_ms = 0;

enum LedMode {
  LED_WAIT_AGENT = 0,
  LED_READY = 1,
  LED_ERROR = 2
};

static volatile LedMode g_led_mode = LED_WAIT_AGENT;
static volatile uint8_t g_error_stage = 0;

static void setAllLeds(bool on) {
  const uint8_t val = on ? HIGH : LOW;
  for (size_t i = 0; i < 4; ++i) {
    digitalWrite(kLedPins[i], val);
  }
}

static void setStageLed(uint8_t stage) {
  for (size_t i = 0; i < 4; ++i) {
    digitalWrite(kLedPins[i], (i == stage) ? HIGH : LOW);
  }
}

static void updateLed() {
  static uint32_t last_toggle_ms = 0;
  static uint8_t chase_index = 0;
  static bool blink_state = false;
  const uint32_t now = millis();

  switch (g_led_mode) {
    case LED_READY:
      setAllLeds(true);
      return;
    case LED_ERROR:
      setStageLed(g_error_stage);
      return;
    case LED_WAIT_AGENT:
    default:
      if (now - last_toggle_ms >= 100) {
        last_toggle_ms = now;
        for (size_t i = 0; i < 4; ++i) {
          digitalWrite(kLedPins[i], (i == chase_index) ? HIGH : LOW);
        }
        chase_index = (uint8_t)((chase_index + 1) % 4);
      }
      return;
  }
}

static void setLedMode(LedMode mode) {
  g_led_mode = mode;
}

static void setErrorStage(uint8_t stage) {
  g_error_stage = stage;
}

static void error_loop() {
  setLedMode(LED_ERROR);
  while (1) {
    updateLed();
    delay(20);
  }
}

static void updateHeaderStamp(std_msgs__msg__Header* header) {
  const uint32_t now_ms = millis();
  header->stamp.sec = (int32_t)(now_ms / 1000);
  header->stamp.nanosec = (uint32_t)((now_ms % 1000) * 1000000UL);
}

static bool captureJpegToBuffer(uint32_t* out_size) {
  CamImage img = theCamera.takePicture();
  if (!img.isAvailable()) {
    Serial.println("takePicture failed");
    return false;
  }

  if (img.getPixFormat() != CAM_IMAGE_PIX_FMT_JPG) {
    Serial.println("Not JPEG format");
    return false;
  }

  const uint32_t size = img.getImgSize();
  if (size > sizeof(g_jpeg_buffer)) {
    Serial.print("Image too large: ");
    Serial.println(size);
    return false;
  }

  memcpy(g_jpeg_buffer, img.getImgBuff(), size);
  *out_size = size;
  return true;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  for (size_t i = 0; i < 4; ++i) {
    pinMode(kLedPins[i], OUTPUT);
    digitalWrite(kLedPins[i], LOW);
  }

  setLedMode(LED_WAIT_AGENT);

  set_microros_transports();

  delay(2000);

  setErrorStage(0);
  CamErr err = theCamera.begin();
  if (err != CAM_ERR_SUCCESS) {
    Serial.print("Camera begin failed: ");
    Serial.println((int)err);
    error_loop();
  }

  setErrorStage(1);
  err = theCamera.setJPEGQuality(50);
  if (err != CAM_ERR_SUCCESS) {
    Serial.print("setJPEGQuality failed: ");
    Serial.println((int)err);
    error_loop();
  }

  setErrorStage(2);
  err = theCamera.setStillPictureImageFormat(
      CAM_IMGSIZE_QQVGA_H,
      CAM_IMGSIZE_QQVGA_V,
      CAM_IMAGE_PIX_FMT_JPG);
  if (err != CAM_ERR_SUCCESS) {
    Serial.print("setStillPictureImageFormat failed: ");
    Serial.println((int)err);
    error_loop();
  }

  setErrorStage(3);
  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "spresense_camera_node", "", &support));
  RCCHECK(rclc_publisher_init_default(
      &publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, CompressedImage),
      kTopicName));

  sensor_msgs__msg__CompressedImage__init(&msg);
  msg.data.data = g_jpeg_buffer;
  msg.data.capacity = sizeof(g_jpeg_buffer);
  msg.data.size = 0;
  static char kFormat[] = "jpeg";
  static char kFrameId[] = "spresense_camera";
  msg.format.data = kFormat;
  msg.format.size = strlen(kFormat);
  msg.format.capacity = sizeof(kFormat);
  msg.header.frame_id.data = kFrameId;
  msg.header.frame_id.size = strlen(kFrameId);
  msg.header.frame_id.capacity = sizeof(kFrameId);

  setLedMode(LED_READY);
  Serial.println("micro-ROS camera publisher ready");
}

void loop() {
  updateLed();
  const uint32_t now = millis();
  if (now - g_last_capture_ms < kMinIntervalMs) {
    delay(5);
    return;
  }

  uint32_t jpeg_size = 0;
  if (captureJpegToBuffer(&jpeg_size)) {
    updateHeaderStamp(&msg.header);
    msg.data.data = g_jpeg_buffer;
    msg.data.size = jpeg_size;
    rcl_ret_t pub_rc = rcl_publish(&publisher, &msg, NULL);
    if (pub_rc != RCL_RET_OK) {
      Serial.print("rcl_publish failed: ");
      Serial.println((int)pub_rc);
    }
    g_last_capture_ms = millis();
  }

  delay(5);
}
