# Spresense Camera Library (Arduino) 仕様まとめ

## 概要
- Arduino向けカメラライブラリは、`CameraClass`(グローバルインスタンス: `theCamera`) と `CamImage` を中心に構成される。
- プレビュー(ストリーミング)取得、静止画撮影、各種パラメータ(露出/ホワイトバランス/ISO/効果)の制御が可能。

## 基本フロー
1. `theCamera.begin(...)` でプレビュー設定を初期化。
2. `startStreaming()` でプレビュー画像をコールバックで受け取る。
3. 静止画は `setStillPictureImageFormat()` を指定して `takePicture()` で取得。

## begin() の主な引数と既定値
```
CamErr begin(
  int      buff_num           = 1,
  int      frame_rate         = CAM_VIDEO_FPS_30,
  int      width              = CAM_IMGSIZE_QVGA_H,
  int      height             = CAM_IMGSIZE_QVGA_V,
  int      format             = CAM_IMAGE_PIX_FMT_YUV422,
  bool     jpegmode           = false,
  int      jpgbufsize_divisor = 7
);
```
- `buff_num`: プレビュー用バッファ数。
- `frame_rate`: プレビューFPS。
- `width/height`: プレビュー解像度。
- `format`: プレビューのピクセルフォーマット。
- `jpegmode` と `jpgbufsize_divisor`: プレビューがJPEGの場合の挙動。

## プレビュー(ストリーミング)
- `startStreaming(bool)` で開始。コールバックは `CamImage` を受け取る。
- プレビューの代表的なピクセルフォーマットは `YUV422` / `RGB565` / `JPEG`。

## 静止画
- `setStillPictureImageFormat(width, height, format)` で静止画フォーマットを指定。
- `takePicture()` で `CamImage` を取得。
- 静止画の代表的なピクセルフォーマットは `YUV422` / `RGB565` / `JPEG`。

## 画像サイズ(一部)
- QVGA: 320x240 (`CAM_IMGSIZE_QVGA_H/V`)
- VGA: 640x480 (`CAM_IMGSIZE_VGA_H/V`)
- HD: 1280x720 (`CAM_IMGSIZE_HD_H/V`)
- QUADVGA: 1280x960 (`CAM_IMGSIZE_QUADVGA_H/V`)
- Full HD: 1920x1080 (`CAM_IMGSIZE_FULLHD_H/V`)
- 4K: 3840x2160 (`CAM_IMGSIZE_4K_H/V`)

## FPS(プレビュー)
- 5 / 6 / 7.5 / 15 / 30 / 60 / 120 (`CAM_VIDEO_FPS_*`)

## ピクセルフォーマット
- `CAM_IMAGE_PIX_FMT_RGB565`
- `CAM_IMAGE_PIX_FMT_YUV422`
- `CAM_IMAGE_PIX_FMT_JPG`
- `CAM_IMAGE_PIX_FMT_GRAY`
- `CAM_IMAGE_PIX_FMT_NONE`

## 代表的な設定API
- 露出: `setAutoExposure(bool)`, `setAbsoluteExposure(int)`
- ISO: `setAutoISOSensitivity(bool)`, `setISOSensitivity(int)`
- ホワイトバランス: `setAutoWhiteBalance(bool)`, `setAutoWhiteBalanceMode(CamWhiteBalance)`
- 色効果: `setColorEffect(CamColorEffect)`
- HDR: `setHDR(CamHDR)`
- JPEG品質: `setJPEGQuality(int)`

## CamImageで使う主なメソッド
- `isAvailable()`
- `getWidth() / getHeight()`
- `getImgBuff() / getImgSize() / getImgBuffSize()`
- `getPixFormat()`
- `convertPixFormat(int)`
- `resizeImageByHW(...)`, `clipAndResizeImageByHW(...)`

## 注意
- 取得可能な解像度やフレームレートはカメラデバイス設定に依存するため、SDK側の対応表も参照する。

## Sources
- https://developer.spresense.sony-semicon.com/development-guides/index?page=arduino_developer_guide&lang=ja#_camera_ライブラリ
- https://developer.spresense.sony-semicon.com/development-guides/sdk_developer_guide_en.html#_camera_library_api_reference
- https://developer.spresense.sony-semicon.com/development-guides/group__CAM__IMGSIZE.html
