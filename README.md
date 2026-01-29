# Spresense_camera

Spresense のカメラモジュールを Arduino IDE から扱うためのサンプル集です。

## Examples

- `examples/01_microros_camera_publisher`
  - JPEG 静止画を `sensor_msgs/CompressedImage` で publish（`image/compressed`）。
- `examples/02_takepicture_timing`
  - `takePicture()` の所要時間と画像サイズを Serial に出力。
- `examples/03_microros_streaming_size_probe`
  - micro-ROS の送信サイズ制約に合わせて画像サイズ/形式を探るストリーミング検証。
- `examples/04_microros_streaming_second_size`
  - 低解像度（mono8）でサイズ固定のストリーミング publish。
- `examples/05_microros_camera_imu_dualcore`
  - カメラ JPEG と IMU（BMI270）を同一スケッチで publish（シングルコア運用）。
