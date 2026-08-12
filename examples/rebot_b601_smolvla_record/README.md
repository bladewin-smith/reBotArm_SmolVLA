# reBot B601-DM SmolVLA RGB-D Recording

This example records teleoperated demonstrations for:

```text
Arrange the banana model and the transparent plastic cola bottle back to their assigned places on the desktop.
```

The setup is:

- follower: reBot Arm B601-DM
- leader: reBot Arm B601-DM in low-stiffness impedance mode
- top camera: Orbbec Gemini 335L RGB + LingBot EnhancedDepthFilter depth + depth visualization image
- wrist camera: Orbbec Gemini 305 RGB only

For SmolVLA, the useful visual streams are:

```text
observation.images.top
observation.images.top_depth
observation.images.wrist
```

The raw depth stream is also stored for later model variants. When
`use_enhanced_depth_filter=true`, both streams are generated from the filtered
depth output:

```text
observation.depths.top
```

## Build Orbbec Bridge

```shell
cd src/lerobot/cameras/orbbec/cpp
rm -rf build
cmake -S . -B build \
  -DORBBECSDK_ROOT=/home/r/ws/OrbbecSDK_v2 \
  -DORBBECSDK_LIBRARY=/home/r/ws/OrbbecSDK_v2/build/linux_arm64/lib/libOrbbecSDK.so \
  -DORBBECSDK_INCLUDE_DIR=/home/r/ws/OrbbecSDK_v2/include \
  -DORBBECSDK_GENERATED_INCLUDE_DIR=/home/r/ws/OrbbecSDK_v2/build/src/generated
cmake --build build --config Release
```

## Calibrate

Use the same neutral zero pose for the leader and follower.

```shell
lerobot-calibrate \
  --robot.type=rebot_b601_follower \
  --robot.port=/dev/ttyACM0 \
  --robot.transport=motorbridge \
  --robot.id=b601_follower

lerobot-calibrate \
  --teleop.type=rebot_b601_leader \
  --teleop.port=/dev/ttyACM0 \
  --teleop.transport=motorbridge \
  --teleop.id=b601_leader
```

## Debug Leader Impedance

Before full teleoperation, tune the leader arm by itself. This does not connect
the follower arm or any cameras.

```shell
python examples/rebot_b601_smolvla_record/debug_leader_impedance.py \
  --port /dev/ttyACM0 \
  --transport motorbridge \
  --id b601_leader \
  --mode impedance \
  --hz 30 \
  --print-hz 2
```

Start with very soft gains, then raise them only if the leader feels too limp:

```shell
python examples/rebot_b601_smolvla_record/debug_leader_impedance.py \
  --port /dev/ttyACM0 \
  --transport motorbridge \
  --id b601_leader \
  --mode impedance \
  --kp 2.0 \
  --kd 0.15 \
  --hz 30
```

You can also tune per joint with comma-separated values in motor order:

```shell
python examples/rebot_b601_smolvla_record/debug_leader_impedance.py \
  --port /dev/ttyACM0 \
  --transport motorbridge \
  --id b601_leader \
  --mode impedance \
  --kp 4,4,3.5,1.5,1,1,0.6 \
  --kd 0.45,0.45,0.35,0.18,0.12,0.12,0.08
```

Press `Ctrl+C` to stop; the script disconnects and disables torque.

## Debug Leader Gravity Compensation

After the impedance test can read all motors normally, test gravity
compensation on the leader alone. The script reuses Seeed's
`reBotArm_control_py` dynamics model and motorbridge serial transport. Install
or keep the Seeed SDK beside this repository, for example:

```shell
/home/r/ws/rebot_grasp/sdk/reBotArm_control_py
```

First run without enabling torque. This only prints the current joint angles
and gravity vector so you can confirm that model loading and feedback are
working:

```shell
python examples/rebot_b601_smolvla_record/debug_leader_gravity_comp.py \
  --port /dev/ttyACM0 \
  --id b601_leader \
  --no-calibrate \
  --dry-run \
  --duration-s 5
```

Then enable only `joint_2` with a small feedforward scale. Keep one hand
supporting the arm and be ready to press `Ctrl+C`:

```shell
python examples/rebot_b601_smolvla_record/debug_leader_gravity_comp.py \
  --port /dev/ttyACM0 \
  --id b601_leader \
  --no-calibrate \
  --enabled-joints joint_2 \
  --torque-scale 0.15 \
  --torque-limit 1.0 \
  --kp 0.3 \
  --kd 0.2 \
  --hz 100
```

If the arm pushes in the wrong direction or feels more difficult to support,
stop immediately and do not raise `--torque-scale`. If the direction feels
correct, increase gently, for example `--torque-scale 0.25`, then test more
joints:

```shell
python examples/rebot_b601_smolvla_record/debug_leader_gravity_comp.py \
  --port /dev/ttyACM0 \
  --id b601_leader \
  --no-calibrate \
  --enabled-joints joint_2,joint_3 \
  --torque-scale 0.25 \
  --torque-limit 2.0 \
  --kp 0.5 \
  --kd 0.2
```

Only after that feels stable, try the arm joints together:

```shell
python examples/rebot_b601_smolvla_record/debug_leader_gravity_comp.py \
  --port /dev/ttyACM0 \
  --id b601_leader \
  --no-calibrate \
  --enabled-joints joint_2,joint_3,joint_4,joint_5,joint_6 \
  --torque-scale 0.35 \
  --torque-limit 3.0 \
  --kp 0.8 \
  --kd 0.3
```

`joint_1` is usually less important for gravity compensation because it rotates
around the vertical axis. Add it only after the other joints are stable.

If the debug script reports packet drops for every motor, probe the raw CAN bus
before tuning gains. This is only for direct SocketCAN wiring; the Seeed
reBotArm_control_py stack normally uses motorbridge DM serial on `/dev/ttyACM0`:

```shell
python examples/rebot_b601_smolvla_record/scan_damiao_can.py \
  --interface can0 \
  --ids 0x01-0x20
```

If that finds nothing, try classic CAN:

```shell
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
python examples/rebot_b601_smolvla_record/scan_damiao_can.py \
  --interface can0 \
  --ids 0x01-0x20 \
  --no-fd
```

## Record

```shell
lerobot-record \
  --robot.type=rebot_b601_follower \
  --robot.port=/dev/ttyACM0 \
  --robot.transport=motorbridge \
  --robot.id=b601_follower \
  --robot.max_relative_target=8.0 \
  --robot.cameras='{
    top: {
      type: orbbec,
      serial_number: "GEMINI335L_SERIAL",
      bridge_binary: "/home/r/ws/rebot_lerobot/lerobot/src/lerobot/cameras/orbbec/cpp/build/orbbec_rgbd_bridge",
      width: 640,
      height: 480,
      fps: 30,
      record_color: true,
      use_depth: true,
      record_depth: true,
      depth_key: "depths.top",
      record_depth_viz: true,
      depth_viz_key: "top_depth",
      depth_viz_min_mm: 250,
      depth_viz_max_mm: 1800,
      align_depth_to_color: true,
      use_enhanced_depth_filter: true,
      enhanced_depth_filter_name: "EnhancedDepthFilter",
      enhanced_depth_model_path: "/home/r/ws/OrbbecSDK_v2/extensions/LingBot-Depth/model.sm4",
      enhanced_depth_confidence_key: "confidence_threshold",
      enhanced_depth_confidence_threshold: 51,
      enhanced_depth_license_check_command: [
        "/home/r/ws/OrbbecSDK_v2/tools/LicenseTool",
        "check",
        "{serial_number}"
      ]
    },
    wrist: {
      type: opencv,
      index_or_path: 0,
      width: 640,
      height: 480,
      fps: 30
    }
  }' \
  --teleop.type=rebot_b601_leader \
  --teleop.port=/dev/ttyACM0 \
  --teleop.transport=motorbridge \
  --teleop.id=b601_leader \
  --teleop.manual_control_mode=impedance \
  --dataset.repo_id="${HF_USER}/rebot_b601_banana_bottle_rgbd" \
  --dataset.fps=30 \
  --dataset.num_episodes=50 \
  --dataset.episode_time_s=45 \
  --dataset.reset_time_s=20 \
  --dataset.single_task="Arrange the banana model and the transparent plastic cola bottle back to their assigned places on the desktop" \
  --display_data=true
```

If the wrist Gemini 305 must also be read through OrbbecSDK instead of OpenCV,
replace the `wrist` camera block with another `type: orbbec` block and set
`record_depth: false` and `record_depth_viz: false`.

## Train SmolVLA

```shell
lerobot-train \
  --policy.path=lerobot/smolvla_base \
  --dataset.repo_id="${HF_USER}/rebot_b601_banana_bottle_rgbd" \
  --batch_size=32 \
  --steps=80000
```

The raw depth tensor is stored in the dataset but is not consumed by the stock
SmolVLA visual encoder. Use `observation.images.top_depth` for depth-aware
training without modifying SmolVLA internals.

## LingBot EnhancedDepthFilter Notes

The Orbbec LingBot EnhancedDepthFilter must run inside the Orbbec C++ bridge,
before depth bytes are sent to Python. This keeps data collection, training,
and edge inference on the same visual distribution.

Use the `model.sm4`, extension libraries, and license files from the same
OrbbecSDK release. The bridge creates the SDK private filter with:

```text
FilterFactory::createPrivateFilter("EnhancedDepthFilter", model.sm4)
```

and applies it to the synchronized color+depth `FrameSet` before Python sees
the frame. If the SDK, extension, model, TensorRT/CUDA runtime, or license is
not valid, recording stops instead of falling back to raw depth.

Configure `enhanced_depth_license_check_command` with the exact check/verify
command from Orbbec LicenseTool. The placeholders `{serial_number}` and
`{model_path}` are expanded before the command runs. If LicenseTool exits with
a non-zero code, recording stops before the camera bridge starts.

If your SDK release uses a different private filter name or config schema key,
override `enhanced_depth_filter_name` or `enhanced_depth_confidence_key` in the
camera config without changing the bridge source.
