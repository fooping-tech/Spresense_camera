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

#define DEBUG_SERIAL 1
#define DEBUG_SERIAL_ONLY 0
#define DEBUG_PRINT_EVERY_MS 1000

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { g_last_rcl_err = temp_rc; error_loop(); } }
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { } }

static rcl_publisher_t publisher;
static rclc_support_t support;
static rcl_allocator_t allocator;
static rcl_node_t node;

static sensor_msgs__msg__Image msg;

static const char* kTopicName = "spresense/camera/stream/image";
static const uint32_t kMinIntervalMs = 200;
// micro-ROS custom transport MTU(512)*history(4)=2048 bytes; keep payload below this.
static const uint32_t kMaxPayloadBytes = 10000;
static const uint16_t kOutWidthMin = 20;
static const uint16_t kOutHeightMin = 15;
static const uint16_t kOutWidthMax = 80;
static const uint16_t kOutHeightMax = 60;
static const uint16_t kOutWidthStep = 4;
static const uint16_t kOutHeightStep = 3;
static const uint8_t kMaxBytesPerPixel = 2;

enum OutputMode {
  MODE_Y_ONLY = 0,
  MODE_UYVY = 1,
  MODE_GRAY = 2,
  MODE_RGB565 = 3,
  MODE_COUNT = 4
};

struct OutputConfig {
  uint16_t width;
  uint16_t height;
  uint8_t bytes_per_pixel;
  OutputMode mode;
};

// Buffer scales with max output size; keep kOutWidth/Height within SRAM limits.
static const uint32_t kImageBufferBytes =
    (uint32_t)kOutWidthMax * (uint32_t)kOutHeightMax * (uint32_t)kMaxBytesPerPixel;
static uint8_t g_image_buffer[kImageBufferBytes];
static volatile uint8_t g_error_stage = 0;
static volatile int g_last_cam_err = 0;
static volatile rcl_ret_t g_last_rcl_err = RCL_RET_OK;
static volatile bool g_buffer_locked = false;
static volatile bool g_frame_ready = false;
static volatile uint32_t g_frame_size = 0;
static volatile uint32_t g_frame_count = 0;
static volatile uint32_t g_cb_count = 0;
static volatile uint32_t g_format_fail = 0;
static volatile uint32_t g_convert_fail = 0;
static volatile uint32_t g_scale_fail = 0;
static volatile uint32_t g_buffer_over = 0;
static volatile uint32_t g_payload_over = 0;
static volatile uint32_t g_publish_skip = 0;
static volatile bool g_payload_too_large = false;
static volatile OutputConfig g_active_cfg = {kOutWidthMin, kOutHeightMin, 1, MODE_Y_ONLY};
static volatile OutputConfig g_frame_cfg = {kOutWidthMin, kOutHeightMin, 1, MODE_Y_ONLY};
static volatile OutputConfig g_pending_cfg = {kOutWidthMin, kOutHeightMin, 1, MODE_Y_ONLY};
static volatile bool g_pending_valid = false;
static uint32_t g_last_debug_ms = 0;
static uint32_t g_last_debug_count = 0;
static uint32_t g_last_publish_ms = 0;

static char kEncodingMono8[] = "mono8";
static char kEncodingUYVY[] = "yuv422_uyvy";
static char kEncodingRGB565[] = "rgb565";
static char kFrameId[] = "spresense_camera";

static uint8_t bytesPerPixel(OutputMode mode) {
  return (mode == MODE_Y_ONLY || mode == MODE_GRAY) ? 1 : 2;
}

static OutputConfig loadConfig(volatile OutputConfig& src) {
  OutputConfig cfg;
  cfg.width = src.width;
  cfg.height = src.height;
  cfg.bytes_per_pixel = src.bytes_per_pixel;
  cfg.mode = (OutputMode)src.mode;
  return cfg;
}

static void storeConfig(volatile OutputConfig& dst, const OutputConfig& src) {
  dst.width = src.width;
  dst.height = src.height;
  dst.bytes_per_pixel = src.bytes_per_pixel;
  dst.mode = src.mode;
}

static const char* modeName(OutputMode mode) {
  switch (mode) {
    case MODE_Y_ONLY:
      return "Y_ONLY";
    case MODE_UYVY:
      return "UYVY";
    case MODE_GRAY:
      return "GRAY";
    case MODE_RGB565:
      return "RGB565";
    default:
      return "UNKNOWN";
  }
}

static OutputConfig normalizeConfig(OutputConfig cfg) {
  if (cfg.width < kOutWidthMin) {
    cfg.width = kOutWidthMin;
  }
  if (cfg.height < kOutHeightMin) {
    cfg.height = kOutHeightMin;
  }
  if (cfg.width > kOutWidthMax) {
    cfg.width = kOutWidthMax;
  }
  if (cfg.height > kOutHeightMax) {
    cfg.height = kOutHeightMax;
  }
  if (cfg.mode == MODE_UYVY && (cfg.width % 2 != 0)) {
    cfg.width -= 1;
  }
  cfg.bytes_per_pixel = bytesPerPixel(cfg.mode);
  return cfg;
}

static void applyPendingConfig() {
  if (!g_pending_valid) {
    return;
  }
  OutputConfig cfg = loadConfig(g_pending_cfg);
  storeConfig(g_active_cfg, normalizeConfig(cfg));
  g_pending_valid = false;
}

static void scheduleNextConfig(OutputConfig base) {
  OutputConfig next = base;
  next.width = (uint16_t)(base.width + kOutWidthStep);
  next.height = (uint16_t)(base.height + kOutHeightStep);
  if (next.width > kOutWidthMax || next.height > kOutHeightMax) {
    next.width = kOutWidthMin;
    next.height = kOutHeightMin;
    next.mode = (OutputMode)((base.mode + 1) % MODE_COUNT);
  }
  next.bytes_per_pixel = bytesPerPixel(next.mode);
  storeConfig(g_pending_cfg, next);
  g_pending_valid = true;
}

static void applyMsgConfig(OutputConfig cfg) {
  msg.width = cfg.width;
  msg.height = cfg.height;
  msg.is_bigendian = 0;
  msg.step = (uint32_t)cfg.width * (uint32_t)cfg.bytes_per_pixel;
  switch (cfg.mode) {
    case MODE_UYVY:
      msg.encoding.data = kEncodingUYVY;
      msg.encoding.size = strlen(kEncodingUYVY);
      msg.encoding.capacity = sizeof(kEncodingUYVY);
      break;
    case MODE_RGB565:
      msg.encoding.data = kEncodingRGB565;
      msg.encoding.size = strlen(kEncodingRGB565);
      msg.encoding.capacity = sizeof(kEncodingRGB565);
      break;
    case MODE_Y_ONLY:
    case MODE_GRAY:
    default:
      msg.encoding.data = kEncodingMono8;
      msg.encoding.size = strlen(kEncodingMono8);
      msg.encoding.capacity = sizeof(kEncodingMono8);
      break;
  }
}

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

  applyPendingConfig();
  OutputConfig cfg = normalizeConfig(loadConfig(g_active_cfg));

  if (cfg.mode == MODE_GRAY) {
    img.convertPixFormat(CAM_IMAGE_PIX_FMT_GRAY);
    if (img.getPixFormat() != CAM_IMAGE_PIX_FMT_GRAY) {
      g_convert_fail++;
      return;
    }
  } else if (cfg.mode == MODE_RGB565) {
    img.convertPixFormat(CAM_IMAGE_PIX_FMT_RGB565);
    if (img.getPixFormat() != CAM_IMAGE_PIX_FMT_RGB565) {
      g_convert_fail++;
      return;
    }
  } else {
    if (img.getPixFormat() != CAM_IMAGE_PIX_FMT_YUV422) {
      g_format_fail++;
      return;
    }
  }

  const int in_w = img.getWidth();
  const int in_h = img.getHeight();
  if (in_w <= 0 || in_h <= 0 || in_w < (int)cfg.width || in_h < (int)cfg.height) {
    g_scale_fail++;
    return;
  }

  const int step_x = in_w / (int)cfg.width;
  const int step_y = in_h / (int)cfg.height;
  if (step_x <= 0 || step_y <= 0) {
    g_scale_fail++;
    return;
  }

  const uint32_t size =
      (uint32_t)cfg.width * (uint32_t)cfg.height * (uint32_t)cfg.bytes_per_pixel;
  if (size > sizeof(g_image_buffer)) {
    g_buffer_over++;
    return;
  }
  const bool payload_too_large = (size > kMaxPayloadBytes);
  if (payload_too_large) {
    g_payload_over++;
  }

  g_buffer_locked = true;
  const uint8_t* src = img.getImgBuff();
  if (cfg.mode == MODE_Y_ONLY) {
    for (uint32_t y = 0; y < cfg.height; ++y) {
      const int src_y = (int)(y * step_y);
      const uint8_t* row = src + (size_t)src_y * (size_t)in_w * 2;
      for (uint32_t x = 0; x < cfg.width; ++x) {
        const int src_x = (int)(x * step_x);
        const uint8_t* px = row + (size_t)(src_x / 2) * 4;
        const uint8_t yval = (src_x % 2 == 0) ? px[1] : px[3];
        g_image_buffer[y * cfg.width + x] = yval;
      }
    }
  } else if (cfg.mode == MODE_UYVY) {
    const uint32_t out_stride = (uint32_t)cfg.width * 2U;
    for (uint32_t y = 0; y < cfg.height; ++y) {
      const int src_y = (int)(y * step_y);
      const uint8_t* row = src + (size_t)src_y * (size_t)in_w * 2;
      uint8_t* out_row = g_image_buffer + (size_t)y * out_stride;
      for (uint32_t x = 0; x < cfg.width; x += 2) {
        const int src_x0 = (int)(x * step_x);
        const int src_x1 = (int)((x + 1) * step_x);
        const uint8_t* px0 = row + (size_t)(src_x0 / 2) * 4;
        const uint8_t* px1 = row + (size_t)(src_x1 / 2) * 4;
        const uint8_t u = px0[0];
        const uint8_t v = px0[2];
        const uint8_t y0 = (src_x0 % 2 == 0) ? px0[1] : px0[3];
        const uint8_t y1 = (src_x1 % 2 == 0) ? px1[1] : px1[3];
        const size_t out_idx = (size_t)(x / 2) * 4;
        out_row[out_idx + 0] = u;
        out_row[out_idx + 1] = y0;
        out_row[out_idx + 2] = v;
        out_row[out_idx + 3] = y1;
      }
    }
  } else if (cfg.mode == MODE_GRAY) {
    for (uint32_t y = 0; y < cfg.height; ++y) {
      const int src_y = (int)(y * step_y);
      const uint8_t* row = src + (size_t)src_y * (size_t)in_w;
      for (uint32_t x = 0; x < cfg.width; ++x) {
        const int src_x = (int)(x * step_x);
        g_image_buffer[y * cfg.width + x] = row[src_x];
      }
    }
  } else if (cfg.mode == MODE_RGB565) {
    const uint32_t out_stride = (uint32_t)cfg.width * 2U;
    for (uint32_t y = 0; y < cfg.height; ++y) {
      const int src_y = (int)(y * step_y);
      const uint8_t* row = src + (size_t)src_y * (size_t)in_w * 2;
      uint8_t* out_row = g_image_buffer + (size_t)y * out_stride;
      for (uint32_t x = 0; x < cfg.width; ++x) {
        const int src_x = (int)(x * step_x);
        const uint8_t* px = row + (size_t)src_x * 2;
        const size_t out_idx = (size_t)x * 2;
        out_row[out_idx + 0] = px[0];
        out_row[out_idx + 1] = px[1];
      }
    }
  }
  g_frame_size = size;
  g_payload_too_large = payload_too_large;
  storeConfig(g_frame_cfg, cfg);
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
  msg.header.frame_id.data = kFrameId;
  msg.header.frame_id.size = strlen(kFrameId);
  msg.header.frame_id.capacity = sizeof(kFrameId);
  applyMsgConfig(loadConfig(g_active_cfg));
  #endif

  setAllLeds(true);
}

void loop() {
  #if DEBUG_SERIAL
  const uint32_t now_ms = millis();
  if (now_ms - g_last_debug_ms >= DEBUG_PRINT_EVERY_MS) {
    const uint32_t count = g_frame_count;
    OutputConfig cfg = loadConfig(g_frame_cfg);
    Serial.print("frames=");
    Serial.print(count - g_last_debug_count);
    Serial.print(" mode=");
    Serial.print(modeName(cfg.mode));
    Serial.print(" out=");
    Serial.print(cfg.width);
    Serial.print("x");
    Serial.print(cfg.height);
    Serial.print(" bpp=");
    Serial.print(cfg.bytes_per_pixel);
    Serial.print(" size=");
    Serial.print(g_frame_size);
    Serial.println(" bytes");
    Serial.print("cb=");
    Serial.print(g_cb_count);
    Serial.print(" format_fail=");
    Serial.print(g_format_fail);
    Serial.print(" convert_fail=");
    Serial.print(g_convert_fail);
    Serial.print(" scale_fail=");
    Serial.print(g_scale_fail);
    Serial.print(" buffer_over=");
    Serial.print(g_buffer_over);
    Serial.print(" payload_over=");
    Serial.print(g_payload_over);
    Serial.print(" publish_skip=");
    Serial.println(g_publish_skip);
    g_last_debug_count = count;
    g_last_debug_ms = now_ms;
  }
  #endif

  #if DEBUG_SERIAL_ONLY
  if (g_frame_ready && !g_buffer_locked) {
    g_buffer_locked = true;
    OutputConfig cfg = loadConfig(g_frame_cfg);
    g_frame_ready = false;
    g_buffer_locked = false;
    scheduleNextConfig(cfg);
  }
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
  OutputConfig cfg = loadConfig(g_frame_cfg);
  if (g_payload_too_large) {
    g_publish_skip++;
    g_frame_ready = false;
    g_buffer_locked = false;
    scheduleNextConfig(cfg);
    delay(2);
    return;
  }
  applyMsgConfig(cfg);
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
  scheduleNextConfig(cfg);

  digitalWrite(LED1, !digitalRead(LED1));
  delay(2);
}
