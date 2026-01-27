#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>

#include <sensor_msgs/msg/image.h>
#include <std_msgs/msg/header.h>

#include <Camera.h>
#include <string.h>

static const uint8_t kLedPins[] = {LED0, LED1, LED2, LED3};

#define DEBUG_SERIAL 0
#define DEBUG_SERIAL_ONLY 0
#define DEBUG_PRINT_EVERY_MS 1000

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { g_last_rcl_err = temp_rc; error_loop(); } }
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { } }

static rcl_publisher_t publisher;
static rclc_support_t support;
static rcl_allocator_t allocator;
static rcl_node_t node;

static sensor_msgs__msg__Image msg;

static uint8_t g_image_buffer[1024];
static volatile uint8_t g_error_stage = 0;
static volatile int g_last_cam_err = 0;
static volatile rcl_ret_t g_last_rcl_err = RCL_RET_OK;
static volatile bool g_buffer_locked = false;
static volatile bool g_frame_ready = false;
static volatile uint32_t g_frame_size = 0;
static volatile uint32_t g_frame_count = 0;
static volatile uint32_t g_cb_count = 0;
static volatile uint32_t g_format_fail = 0;
static volatile uint32_t g_scale_fail = 0;
static volatile uint32_t g_size_over = 0;
static uint32_t g_last_debug_ms = 0;
static uint32_t g_last_debug_count = 0;
static uint32_t g_last_publish_ms = 0;

static const char* kTopicName = "spresense/camera/stream/image";
static const uint32_t kMinIntervalMs = 200;
// micro-ROS custom transport MTU(512)*history(4)=2048 bytes; keep payload below this.
static const uint32_t kMaxPayloadBytes = 600;
static const uint32_t kOutWidth = 20;
static const uint32_t kOutHeight = 15;

static void setAllLeds(bool on) {
  const uint8_t val = on ? HIGH : LOW;
  for (size_t i = 0; i < 4; ++i) {
    digitalWrite(kLedPins[i], val);
  }
}

static void waitForAgent() {
  while (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
    digitalWrite(LED0, !digitalRead(LED0));
    delay(100);
  }
  digitalWrite(LED0, LOW);
}

static void setErrorStage(uint8_t stage) {
  g_error_stage = stage;
}

static void error_loop() {
  digitalWrite(LED3, HIGH);
  uint32_t last_report_ms = 0;
  while (1) {
    #if DEBUG_SERIAL
    const uint32_t now = millis();
    if (now - last_report_ms >= 1000) {
      last_report_ms = now;
      Serial.println("ERROR");
      Serial.print("stage=");
      Serial.println((int)g_error_stage);
      if (g_last_cam_err != 0) {
        Serial.print("cam_err=");
        Serial.println((int)g_last_cam_err);
      }
      if (g_last_rcl_err != RCL_RET_OK) {
        Serial.print("rcl_err=");
        Serial.println((int)g_last_rcl_err);
      }
      Serial.flush();
    }
    #endif
    delay(50);
  }
}

static void updateHeaderStamp(std_msgs__msg__Header* header) {
  const uint32_t now_ms = millis();
  header->stamp.sec = (int32_t)(now_ms / 1000);
  header->stamp.nanosec = (uint32_t)((now_ms % 1000) * 1000000UL);
}

static void CamCB(CamImage img) {
  if (!img.isAvailable()) {
    return;
  }

  g_cb_count++;
  // Frame arrived indicator.
  digitalWrite(LED0, !digitalRead(LED0));

  if (g_buffer_locked) {
    return;
  }

  if (img.getPixFormat() != CAM_IMAGE_PIX_FMT_YUV422) {
    g_format_fail++;
    return;
  }

  const int in_w = img.getWidth();
  const int in_h = img.getHeight();
  if (in_w <= 0 || in_h <= 0 || in_w < (int)kOutWidth || in_h < (int)kOutHeight) {
    g_scale_fail++;
    return;
  }

  const int step_x = in_w / (int)kOutWidth;
  const int step_y = in_h / (int)kOutHeight;
  if (step_x <= 0 || step_y <= 0) {
    g_scale_fail++;
    return;
  }

  const uint32_t size = kOutWidth * kOutHeight;
  if (size > sizeof(g_image_buffer)) {
    g_size_over++;
    return;
  }
  if (size > kMaxPayloadBytes) {
    digitalWrite(LED2, !digitalRead(LED2));
    g_size_over++;
    return;
  }

  g_buffer_locked = true;
  const uint8_t* src = img.getImgBuff();
  for (uint32_t y = 0; y < kOutHeight; ++y) {
    const int src_y = (int)(y * step_y);
    const uint8_t* row = src + (size_t)src_y * (size_t)in_w * 2;
    for (uint32_t x = 0; x < kOutWidth; ++x) {
      const int src_x = (int)(x * step_x);
      const uint8_t* px = row + (size_t)(src_x / 2) * 4;
      const uint8_t yval = (src_x % 2 == 0) ? px[1] : px[3];
      g_image_buffer[y * kOutWidth + x] = yval;
    }
  }
  g_frame_size = size;
  g_frame_ready = true;
  g_frame_count++;
  // Frame accepted indicator.
  digitalWrite(LED2, !digitalRead(LED2));
  g_buffer_locked = false;
}

void setup() {
  #if DEBUG_SERIAL
  Serial.begin(115200);
  const uint32_t start_ms = millis();
  while (!Serial && (millis() - start_ms < 2000)) {
    ;
  }
  Serial.println("streaming debug start");
  #endif

  for (size_t i = 0; i < 4; ++i) {
    pinMode(kLedPins[i], OUTPUT);
    digitalWrite(kLedPins[i], LOW);
  }

  // Initialize camera first to secure buffers before micro-ROS allocates memory.
  setErrorStage(1);
  CamErr err = theCamera.begin();
  if (err != CAM_ERR_SUCCESS) {
    g_last_cam_err = err;
    error_loop();
  }

  setErrorStage(2);
  err = theCamera.startStreaming(true, CamCB);
  if (err != CAM_ERR_SUCCESS) {
    g_last_cam_err = err;
    error_loop();
  }

  #if !DEBUG_SERIAL_ONLY
  set_microros_transports();
  delay(2000);
  waitForAgent();

  allocator = rcl_get_default_allocator();
  setErrorStage(3);
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  setErrorStage(4);
  RCCHECK(rclc_node_init_default(&node, "spresense_camera_stream_node", "", &support));
  setErrorStage(5);
  RCCHECK(rclc_publisher_init_best_effort(
      &publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Image),
      kTopicName));
  #endif

  #if !DEBUG_SERIAL_ONLY
  sensor_msgs__msg__Image__init(&msg);
  msg.data.data = g_image_buffer;
  msg.data.capacity = sizeof(g_image_buffer);
  msg.data.size = 0;
  static char kEncoding[] = "mono8";
  static char kFrameId[] = "spresense_camera";
  msg.encoding.data = kEncoding;
  msg.encoding.size = strlen(kEncoding);
  msg.encoding.capacity = sizeof(kEncoding);
  msg.header.frame_id.data = kFrameId;
  msg.header.frame_id.size = strlen(kFrameId);
  msg.header.frame_id.capacity = sizeof(kFrameId);
  msg.height = kOutHeight;
  msg.width = kOutWidth;
  msg.is_bigendian = 0;
  msg.step = kOutWidth;
  #endif

  setAllLeds(true);
}

void loop() {
  #if DEBUG_SERIAL
  const uint32_t now_ms = millis();
  if (now_ms - g_last_debug_ms >= DEBUG_PRINT_EVERY_MS) {
    const uint32_t count = g_frame_count;
    Serial.print("frames=");
    Serial.print(count - g_last_debug_count);
    Serial.print(" size=");
    Serial.print(g_frame_size);
    Serial.println(" bytes");
    Serial.print("cb=");
    Serial.print(g_cb_count);
    Serial.print(" format_fail=");
    Serial.print(g_format_fail);
    Serial.print(" scale_fail=");
    Serial.print(g_scale_fail);
    Serial.print(" size_over=");
    Serial.println(g_size_over);
    g_last_debug_count = count;
    g_last_debug_ms = now_ms;
  }
  #endif

  #if DEBUG_SERIAL_ONLY
  delay(2);
  return;
  #endif

  const uint32_t now = millis();
  if (now - g_last_publish_ms < kMinIntervalMs) {
    delay(2);
    return;
  }

  if (!g_frame_ready || g_buffer_locked) {
    delay(2);
    return;
  }

  g_buffer_locked = true;
  updateHeaderStamp(&msg.header);
  msg.data.data = g_image_buffer;
  msg.data.size = g_frame_size;
  rcl_ret_t pub_rc = rcl_publish(&publisher, &msg, NULL);
  if (pub_rc != RCL_RET_OK) {
    digitalWrite(LED3, !digitalRead(LED3));
  }
  g_frame_ready = false;
  g_last_publish_ms = now;
  g_buffer_locked = false;

  digitalWrite(LED1, !digitalRead(LED1));
  delay(2);
}
