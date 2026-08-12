# Orbbec Gemini 305 Wrist RGB + Top Depth Recording

This example records:

- wrist RGB from an Orbbec Gemini 305 exposed as a normal OpenCV/UVC camera
- top depth from an Orbbec camera through the OrbbecSDK v2 C++ bridge

The wrist RGB camera is stored as:

```text
observation.images.wrist
```

The top depth stream is stored as:

```text
observation.depths.top
```

Depth values are stored as `uint16` millimeters by default.

## 1. Find the Wrist RGB Camera

```shell
lerobot-find-cameras opencv
```

Use the detected OpenCV index/path for `wrist.index_or_path`.

## 2. Build the Orbbec C++ Bridge

Build `orbbec_rgbd_bridge` against OrbbecSDK v2:

```shell
cd src/lerobot/cameras/orbbec/cpp
find /home/r/ws/OrbbecSDK_v2 -name ObSensor.hpp
find /home/r/ws/OrbbecSDK_v2 -name Export.h
cmake -S . -B build \
  -DORBBECSDK_ROOT=/home/r/ws/OrbbecSDK_v2 \
  -DORBBECSDK_LIBRARY=/home/r/ws/OrbbecSDK_v2/build/linux_arm64/lib/libOrbbecSDK.so \
  -DORBBECSDK_INCLUDE_DIR=/home/r/ws/OrbbecSDK_v2/include \
  -DORBBECSDK_GENERATED_INCLUDE_DIR=/home/r/ws/OrbbecSDK_v2/build/src/generated
cmake --build build --config Release
```

`ORBBECSDK_INCLUDE_DIR` should be the directory that contains
`libobsensor/ObSensor.hpp`. For example, if `find` prints:

```text
/home/r/ws/OrbbecSDK_v2/include/libobsensor/ObSensor.hpp
```

then pass:

```text
-DORBBECSDK_INCLUDE_DIR=/home/r/ws/OrbbecSDK_v2/include
```

Some source builds also need generated headers. If `find` prints:

```text
/home/r/ws/OrbbecSDK_v2/build/src/generated/Export.h
```

then pass:

```text
-DORBBECSDK_GENERATED_INCLUDE_DIR=/home/r/ws/OrbbecSDK_v2/build/src/generated
```

If your OrbbecSDK installation provides `OrbbecSDKConfig.cmake`, you can use
`-DOrbbecSDK_DIR=/path/to/directory/containing/OrbbecSDKConfig.cmake` instead.

Use the resulting executable path as `top.bridge_binary`.

## 3. Record

```shell
lerobot-record \
  --robot.type=so100_follower \
  --robot.port=/dev/ttyACM0 \
  --robot.id=rebot_arm \
  --robot.cameras='{
    wrist: {
      type: opencv,
      index_or_path: 0,
      width: 640,
      height: 480,
      fps: 30
    },
    top: {
      type: orbbec,
      bridge_binary: "/absolute/path/to/orbbec_rgbd_bridge",
      width: 640,
      height: 480,
      fps: 30,
      record_color: false,
      use_depth: true,
      record_depth: true,
      depth_key: "depths.top",
      depth_dtype: "uint16",
      align_depth_to_color: true
    }
  }' \
  --teleop.type=so100_leader \
  --teleop.port=/dev/ttyACM1 \
  --teleop.id=rebot_leader \
  --dataset.repo_id="${HF_USER}/rebot_orbbec_rgbd" \
  --dataset.num_episodes=10 \
  --dataset.single_task="Put the object into the target area"
```

To enable the Orbbec LingBot EnhancedDepthFilter, add:

```text
use_enhanced_depth_filter: true,
enhanced_depth_filter_name: "EnhancedDepthFilter",
enhanced_depth_model_path: "/absolute/path/to/model.sm4",
enhanced_depth_confidence_key: "confidence_threshold",
enhanced_depth_confidence_threshold: 51,
enhanced_depth_license_check_command: [
  "/absolute/path/to/LicenseTool",
  "check",
  "{serial_number}"
]
```

The bridge creates the OrbbecSDK private filter and applies it to the
synchronized color+depth `FrameSet` before Python receives the frame:

```text
FilterFactory::createPrivateFilter("EnhancedDepthFilter", model.sm4)
```

Use the `model.sm4`, extension libraries, and license files from the same
OrbbecSDK release. If the SDK, extension, model, TensorRT/CUDA runtime, or
license is not valid, recording stops instead of falling back to raw depth. If
your SDK release uses a different private filter name or config schema key,
override `enhanced_depth_filter_name` or `enhanced_depth_confidence_key`.
