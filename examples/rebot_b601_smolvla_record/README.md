# reBot B601-DM SmolVLA RGB-D Recording

This example records teleoperated demonstrations for:

```text
Arrange the banana model and the transparent plastic cola bottle back to their assigned places on the desktop.
```

The setup is:

- follower: reBot Arm B601-DM
- leader: reBot Arm B601-DM in gravity compensation mode
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

## Seeed SDK Dependency

The B601-DM motorbridge transport and leader gravity compensation depend on
Seeed's official reBot DevArm SDK. Clone it beside `rebot_lerobot` before
running the B601 examples:

```shell
cd /home/r/ws
git clone https://github.com/Seeed-Projects/reBot-DevArm-Grasp.git rebot_grasp
```

The expected SDK path is:

```text
/home/r/ws/rebot_grasp/sdk/reBotArm_control_py
```

If your checkout is elsewhere, pass the path explicitly when using gravity
compensation:

```shell
--teleop.gravity_comp_sdk_root=/path/to/rebot_grasp/sdk/reBotArm_control_py
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
  --enabled-joints joint_1,joint_2,joint_3,joint_4,joint_5,joint_6 \
  --torque-scale 0.95 \
  --torque-limit 8.0 \
  --kp 1.9 \
  --kd 0.75 \
  --hz 100
```

These are the tuned values used by the recording command below. Keep the
gripper out of gravity compensation unless you have separately tuned it.

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

## Teleoperate Without Recording

Before recording, run a short leader-to-follower teleoperation test without
cameras or dataset writing. The tuned B601 defaults include:

- follower gripper endpoint mapping: leader `5 -> -310` maps to follower `10 -> -320`
- follower gripper command range: `-330` to `10`
- follower gripper gains: `kp=35.0`, `kd=0.8`
- leader gripper force assist: `kp=0`, `kd=0`, `open_bias_torque=0.09`, `torque_limit=0.15`

```shell
lerobot-teleoperate \
  --robot.type=rebot_b601_follower \
  --robot.port=/dev/ttyACM1 \
  --robot.transport=motorbridge \
  --robot.id=b601_follower \
  --teleop.type=rebot_b601_leader \
  --teleop.port=/dev/ttyACM0 \
  --teleop.transport=motorbridge \
  --teleop.id=b601_leader \
  --teleop.manual_control_mode=gravity_comp \
  --fps=30 \
  --teleop_time_s=60
```

If `Relative goal position magnitude had to be clamped` appears repeatedly,
the leader and follower are starting from noticeably different poses. Put both
arms in a similar neutral pose before starting the test. For controlled bench
testing you may temporarily raise `--robot.max_relative_target`, but keep it
enabled during normal teleoperation and recording.

To temporarily enable the leader gripper motor without applying gravity
feedforward torque to it, include `gripper` in `gravity_comp_enabled_joints`
but keep `gravity_comp_torque_joints` limited to the arm joints:

```shell
--teleop.gravity_comp_enabled_joints='["joint_1","joint_2","joint_3","joint_4","joint_5","joint_6","gripper"]' \
--teleop.gravity_comp_torque_joints='["joint_1","joint_2","joint_3","joint_4","joint_5","joint_6"]'
```

If the leader gripper is still difficult to open, enable the optional gripper
assist in its safer low-stiffness mode. This does not actively drive the
leader gripper open; it only lowers the gripper stiffness and damping:

```shell
--teleop.gravity_comp_gripper_mode=low_stiffness \
--teleop.gravity_comp_gripper_assist=true \
--teleop.gravity_comp_gripper_kp=0.15 \
--teleop.gravity_comp_gripper_kd=0.05
```

If the leader gripper becomes harder to backdrive when powered than when power
is disconnected, use zero-torque MIT mode instead. This keeps the gripper motor
enabled for a free-drive command with `kp=0`, `kd=0`, and `tau=0`:

```shell
--teleop.gravity_comp_gripper_mode=zero_torque
```

The current default uses `force_assist` because it gave the best tuned gripper
feel on the test B601 pair. If the leader gripper starts to open by itself,
reduce these two values:

```shell
--teleop.gravity_comp_gripper_open_bias_torque=0.05 \
--teleop.gravity_comp_gripper_torque_limit=0.10
```

If the follower gripper still does not open far enough, scale only the follower
gripper target instead of forcing the leader gripper farther:

```shell
--robot.gripper_action_scale=1.5
```

If the follower gripper still lags behind, keep the arm safety limit but give
the gripper its own relative limit and gains:

```shell
--robot.gripper_action_scale=2.0 \
--robot.gripper_max_relative_target=30.0 \
--robot.gripper_position_kp=35.0 \
--robot.gripper_position_kd=0.8
```

For the best leader-to-follower gripper feel, calibrate a linear endpoint
mapping instead of using only `gripper_action_scale`. First measure:

- leader closed position
- leader open position
- follower closed command/position
- follower open command/position

Then pass those four endpoints:

```shell
--robot.gripper_leader_close_pos=LEADER_CLOSED_DEG \
--robot.gripper_leader_open_pos=LEADER_OPEN_DEG \
--robot.gripper_follower_close_pos=FOLLOWER_CLOSED_DEG \
--robot.gripper_follower_open_pos=FOLLOWER_OPEN_DEG
```

When these endpoint parameters are set, `gripper_action_scale` and
`gripper_action_offset` are ignored for the gripper. The leader closed endpoint
maps to the follower closed endpoint, the leader open endpoint maps to the
follower open endpoint, and the middle range is interpolated linearly.

To measure the real leader/follower gripper ranges before tuning scale and
offset, run:

```shell
python examples/rebot_b601_smolvla_record/debug_gripper_mapping.py \
  --leader-port /dev/ttyACM0 \
  --follower-port /dev/ttyACM1
```

If the follower gripper appears capped by its absolute command range, command
the follower gripper directly before changing teleoperation parameters:

```shell
python examples/rebot_b601_smolvla_record/debug_follower_gripper_command.py \
  --port /dev/ttyACM1 \
  --target -80 \
  --min-pos -90 \
  --max-pos 5

python examples/rebot_b601_smolvla_record/debug_follower_gripper_command.py \
  --port /dev/ttyACM1 \
  --target 0 \
  --min-pos -90 \
  --max-pos 5
```

If the direct command reaches the desired physical open/close range, expose the
same range during teleoperation:

```shell
--robot.gripper_min_pos=-90 \
--robot.gripper_max_pos=5
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
  --teleop.manual_control_mode=gravity_comp \
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
