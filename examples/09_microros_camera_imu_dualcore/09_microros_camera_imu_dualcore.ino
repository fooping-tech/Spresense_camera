// Main core: camera + IMU micro-ROS publishers (single-core).

#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <rosidl_runtime_c/string_functions.h>

#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>

#include <sensor_msgs/msg/compressed_image.h>
#include <sensor_msgs/msg/imu.h>
#include <math.h>

// rcutils expects getenv() even on bare-metal; provide a stub for Spresense.
extern "C" char *getenv(const char *name) {
  (void)name;
  return NULL;
}

static rcl_allocator_t allocator;
static rclc_support_t support;
static rcl_node_t node;
static rcl_publisher_t image_publisher;
static rcl_publisher_t imu_publisher;
static sensor_msgs__msg__CompressedImage msg_static;
static sensor_msgs__msg__Imu imu_msg;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ printf("soft error\n");}}

#include <Camera.h>
#include "BMI270_Arduino.h"

BMI270Class BMI270;

void error_loop(){
  while(1){
    digitalWrite(LED0, !digitalRead(LED0));
    delay(100);
  }
}

void IMU_print_rslt(int8_t rslt) {
  switch (rslt) {
    case BMI2_OK: return;
    default:
      Serial.println("Error [" + String(rslt) + "] : Unknown error code");
      break;
  }
}

int8_t IMU_configure_sensor() {
  int8_t rslt;
  uint8_t sens_list[2] = { BMI2_ACCEL, BMI2_GYRO };
  struct bmi2_sens_config config[2];

  // Accel
  config[0].type = BMI2_ACCEL;
  config[0].cfg.acc.odr = BMI2_ACC_ODR_200HZ;
  config[0].cfg.acc.range = BMI2_ACC_RANGE_2G;
  config[0].cfg.acc.bwp = BMI2_ACC_NORMAL_AVG4;
  config[0].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;

  // Gyro
  config[1].type = BMI2_GYRO;
  config[1].cfg.gyr.odr = BMI2_GYR_ODR_100HZ;
  config[1].cfg.gyr.range = BMI2_GYR_RANGE_2000;
  config[1].cfg.gyr.bwp = BMI2_GYR_NORMAL_MODE;
  config[1].cfg.gyr.noise_perf = BMI2_POWER_OPT_MODE;
  config[1].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;

  rslt = BMI270.set_sensor_config(config, 2);
  if (rslt != BMI2_OK) return rslt;

  rslt = BMI270.sensor_enable(sens_list, 2);
  return rslt;
}

void setup() {
  Serial.begin(115200);

  set_microros_transports();

  delay(2000);
  
  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));   
  
  // create node 
  RCCHECK(rclc_node_init_default(&node, "my_node_serial", "", &support));
  
  // create publishers
  RCCHECK(rclc_publisher_init_default(
    &image_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, CompressedImage),
    "image/compressed"));
  RCCHECK(rclc_publisher_init_default(
    &imu_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "imu_spresense"));


  static uint8_t img_buff[20000] = {0};
  static char    frame_id_data[30] = {0};
  static char    format_data[6] = {0};
  msg_static.header.frame_id.capacity = 7;
  msg_static.header.frame_id.data=frame_id_data;
  memcpy(msg_static.header.frame_id.data, "myframe", 7);
  msg_static.header.frame_id.size = 7;
  msg_static.data.capacity = 20000;
  msg_static.data.data=img_buff;
  msg_static.data.size = 0;
  msg_static.format.capacity = 4;
  msg_static.format.data=format_data;
  memcpy(msg_static.format.data, "jpeg", 4);
  msg_static.format.size = 4;

  // IMU message init
  imu_msg.header.frame_id.data = (char*)"imu_link";
  imu_msg.header.frame_id.size = strlen("imu_link");
  imu_msg.header.frame_id.capacity = strlen("imu_link");
  imu_msg.orientation_covariance[0] = -1.0f;
  for (int i = 0; i < 9; i++) {
    imu_msg.angular_velocity_covariance[i] = 0.0f;
    imu_msg.linear_acceleration_covariance[i] = 0.0f;
  }
  imu_msg.linear_acceleration.x = 0.0f;
  imu_msg.linear_acceleration.y = 0.0f;
  imu_msg.linear_acceleration.z = 0.0f;
  imu_msg.angular_velocity.x = 0.0f;
  imu_msg.angular_velocity.y = 0.0f;
  imu_msg.angular_velocity.z = 0.0f;

  // Camera setup
  theCamera.begin();
  theCamera.setStillPictureImageFormat(
    CAM_IMGSIZE_QVGA_H, CAM_IMGSIZE_QVGA_V, CAM_IMAGE_PIX_FMT_JPG);

  // IMU setup
  int8_t rslt = BMI270.begin(BMI270_I2C, BMI2_I2C_SEC_ADDR);
  IMU_print_rslt(rslt);
  rslt = IMU_configure_sensor();
  IMU_print_rslt(rslt);
      
  digitalWrite(LED0, HIGH);   
}


void loop() {
  digitalWrite(LED2, LOW);
  digitalWrite(LED3, LOW);
  CamImage img = theCamera.takePicture();
  if (img.isAvailable()) {
    if (img.getImgSize() > msg_static.data.capacity) {
      digitalWrite(LED3, HIGH);
    } else {
      msg_static.data.size = img.getImgSize();
      memcpy(msg_static.data.data, img.getImgBuff(), img.getImgSize());
      RCSOFTCHECK(rcl_publish(&image_publisher, &msg_static, NULL));
    }
  } else {
    digitalWrite(LED2, HIGH);
  }

  static uint32_t last_imu_ms = 0;
  uint32_t now_ms = millis();
  if (now_ms - last_imu_ms >= 100) {
    struct bmi2_sens_float sensor_data;
    int8_t rslt = BMI270.bmi2_get_sensor_float(&sensor_data);
    if (rslt == BMI2_OK) {
      imu_msg.header.stamp.sec = now_ms / 1000;
      imu_msg.header.stamp.nanosec = (now_ms % 1000) * 1000000;

      float ax = -sensor_data.acc.y;
      float ay =  sensor_data.acc.x;
      float az =  sensor_data.acc.z;
      float gx = -sensor_data.gyr.y * M_PI / 180.0f;
      float gy =  sensor_data.gyr.x * M_PI / 180.0f;
      float gz =  sensor_data.gyr.z * M_PI / 180.0f;

      imu_msg.linear_acceleration.x = ax;
      imu_msg.linear_acceleration.y = ay;
      imu_msg.linear_acceleration.z = az;
      imu_msg.angular_velocity.x = gx;
      imu_msg.angular_velocity.y = gy;
      imu_msg.angular_velocity.z = gz;

      RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
    }
    last_imu_ms = now_ms;
  }
}
