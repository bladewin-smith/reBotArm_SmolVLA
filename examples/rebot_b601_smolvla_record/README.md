# reBot B601-DM SmolVLA RGB-D Recording

This example records teleoperated demonstrations for:

```text
Arrange the banana model and the transparent plastic cola bottle back to their assigned places on the desktop.
```

The setup is:

- follower: reBot Arm B601-DM
- leader: reBot Arm B601-DM in gravity compensation mode
- top camera: Orbbec Gemini 335L RGB + LingBot EnhancedDepthFilter depth + depth visualization image
- wrist camera: Orbbec Gemini 305 RGB only through the OrbbecSDK bridge

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
  --robot.port=/dev/ttyACM1 \
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

## Follower Safety During Recording

The follower MotorBridge backend runs a dedicated MIT command stream by
default. The dataset loop only updates the target while the stream repeats the
latest complete seven-motor command at 100 Hz. This is important because
Orbbec frame waits, EnhancedDepthFilter processing, episode encoding, and
dataset saving can all pause the main recording loop. Without an independent
stream, a long pause can trigger a DM communication watchdog or protection
state even when the leader and follower pose difference is small.

The recording defaults are:

```text
FOLLOWER_COMMAND_STREAM_ENABLED=true
FOLLOWER_COMMAND_STREAM_HZ=100
FOLLOWER_COMMAND_STREAM_MAX_FAILURES=5
FOLLOWER_COMMAND_STREAM_MAX_GAP_S=0.25
FOLLOWER_ABORT_ON_MOTOR_FAULT_STATUS=true
FOLLOWER_MOTOR_FEEDBACK_MAX_MISSES=3
FOLLOWER_RUNTIME_ERROR_HOLD_S=15
LEADER_COMMAND_STREAM_ENABLED=true
LEADER_COMMAND_STREAM_HZ=100
LEADER_COMMAND_STREAM_MAX_FAILURES=5
LEADER_COMMAND_STREAM_MAX_GAP_S=0.25
LEADER_ABORT_ON_MOTOR_FAULT_STATUS=true
LEADER_MOTOR_FEEDBACK_MAX_MISSES=3
```

The stream fault is latched after five consecutive serial-send failures, and
recording then exits instead of silently resuming. A completed MIT refresh that
arrives just after the 0.25 s deadline is reported as a recovered-gap warning;
one recovered scheduling spike is not treated as five serial failures. Five
consecutive completed-gap violations still latch a fault, and a health check
made while the stream is currently older than 0.25 s fails immediately. This
keeps sustained command loss fatal without aborting a new episode because of a
single already-recovered video-encoding or scheduler spike. A single recovered
gap at least twice the configured limit (0.5 s with the defaults) also latches
immediately because it is too long to treat as harmless scheduler jitter.

MotorBridge follows the DM
status definitions: `0x0` is `DISABLED`, `0x1` is `ENABLED`, and `0x8` through
`0xE` are drive faults. Both normal states provide valid feedback. Before
torque enable the handshake accepts `DISABLED`; while actively controlling the
arm every commanded motor must report `ENABLED`. A drive fault preserves the
last valid joint feedback and aborts collection. An unexpected transition to
`DISABLED` during active control also aborts. The software deliberately does
not automatically re-enable either condition. Three consecutive missing
feedback samples from the same motor also abort collection instead of reusing
an old joint angle indefinitely.

The leader uses the same independent 100 Hz refresh for gravity compensation.
Its gravity vector and gripper assist are updated from each new leader feedback
sample, while the most recent complete command is repeated during camera and
dataset stalls. This preserves the 100 Hz command cadence used by the standalone
gravity-compensation test even when recording RGB-D at about 10 FPS.
During shutdown, follower cameras are disconnected before the follower motor
bus, so the command stream remains active through slow Orbbec SDK teardown.
If a camera, encoder, or dataset operation raises while motor control is still
healthy, the follower holds its last pose for 15 seconds before teardown. This
is only time for the operator to support the arm or use the E-stop; it is not
automatic fault recovery. The delay is skipped when the command stream or a DM
status is already unhealthy.

Before reconnecting the two Orbbec cameras, test the updated follower command
stream with camera-free teleoperation while physically supporting the arm:

First test the follower alone. This script deliberately sleeps its main thread
for five seconds at a time; the follower should keep holding the safe starting
pose through the independent stream. It disables torque on exit, so support the
arm throughout the test:

```shell
python examples/rebot_b601_smolvla_record/debug_follower_command_stream.py \
  --port /dev/ttyACM0 \
  --duration-s 20 \
  --stall-s 5 \
  --stream-hz 100
```

Then test camera-free leader-to-follower teleoperation:

```shell
lerobot-teleoperate \
  --robot.type=rebot_b601_follower \
  --robot.port=/dev/ttyACM0 \
  --robot.transport=motorbridge \
  --robot.id=b601_follower \
  --robot.command_stream_enabled=true \
  --robot.command_stream_hz=100 \
  --robot.command_stream_max_gap_s=0.25 \
  --teleop.type=rebot_b601_leader \
  --teleop.port=/dev/ttyACM1 \
  --teleop.transport=motorbridge \
  --teleop.id=b601_leader \
  --teleop.manual_control_mode=gravity_comp \
  --teleop.command_stream_enabled=true \
  --teleop.command_stream_hz=100 \
  --teleop.command_stream_max_gap_s=0.25 \
  --fps=30 \
  --teleop_time_s=60
```

Verify `/dev/ttyACM0` and `/dev/ttyACM1` on the actual Jetson before running;
USB enumeration can swap them after reconnecting devices. Then run one short
RGB-D episode before a full collection:

```shell
bash examples/rebot_b601_smolvla_record/record_b601_smolvla_rgbd.sh \
  --num-episodes 1 \
  --episode-time-s 20 \
  --reset-time-s 10
```

If the arm still loses torque with the command stream active, do not raise Kp,
Kd, torque limits, or relative-target thresholds. Preserve the complete log and
look for `status_code`, `MIT command stream failure`, or `MIT command stream
gap`. Also measure the follower 24 V rail during motion and inspect the E-stop,
power supply current limit, serial/CAN bridge, motor power connectors, and
joint temperature. A simultaneous whole-arm drop with clean pose tracking is
more consistent with common power/communication loss than calibration error.

`recovered after a 0.252s gap, exceeding 0.250s (completed-gap violation 1/5)`
means the stream has already sent a fresh command and remains usable. It should
not terminate recording by itself. Repeated warnings indicate Jetson CPU or I/O
contention; keep RGB-D collection at 10 FPS, use H.264, close Rerun when it is
not needed, and avoid running other CPU-heavy jobs during collection. The B601
RGB-D script also defaults `PARALLEL_VIDEO_ENCODING=false`, so the three camera
videos are encoded one at a time between episodes instead of starting three
competing encoder processes. This can make the reset interval longer, but does
not change the dataset schema, frames, or SmolVLA compatibility. A
`command stream gap is ...` error means the stream is still late at the time of
the health check and remains a hard stop. A latched serial-send, repeated-gap,
DM status, or missing-feedback fault also remains a hard stop.

If collection stops before a new episode receives its first frame, all episodes
that were saved previously remain valid. Keep the same `DATASET_ROOT` and
`DATASET_REPO_ID`, calculate `remaining = target_total - saved_total`, and
resume with:

```shell
target_total=30
saved_total=7
remaining=$((target_total - saved_total))
bash examples/rebot_b601_smolvla_record/record_b601_smolvla_rgbd.sh \
  --num-episodes "${remaining}" \
  --extra --resume=true
```

Read `meta/info.json` under the dataset root to confirm `total_episodes` before
resuming. Here `--num-episodes` is the number of additional episodes for this
process, not the final total stored in the dataset. Alternatively set
`NUM_EPISODES` to the remaining count and `EXTRA_ARGS=(--resume=true)` in the
script's user settings block.

### Known Intermittent Jetson Command-Stream Stall

An intermittent failure has been observed on Jetson Orin NX during this RGB-D
collection workload. At an episode boundary, both MotorBridge command streams
can be delayed long enough to trip the hard safety limit. One captured failure
reported approximately 0.62-0.64 s gaps on the follower and leader streams,
exceeding the 0.50 s hard recovered-gap limit. Stopping the failed process and
restarting it with the same dataset root and `--resume=true` successfully
continued from the missing episode and completed the remaining collection.

The current working hypothesis is transient Jetson system load, I/O contention,
USB power/bus pressure, thermal throttling, or platform power/current throttling
while running two Orbbec streams, EnhancedDepthFilter, image writing, H.264
encoding, and two 100 Hz MotorBridge streams. This is not yet a confirmed root
cause. In the captured failure there was no DM `0xA (OVER_CURRENT)`, voltage,
temperature, or communication fault status. Simultaneous gaps on both
`/dev/ttyACM0` and `/dev/ttyACM1` point more strongly to a shared host-side
stall than to one arm motor entering over-current protection. Jetson platform
over-current and arm-drive over-current are separate conditions and must be
diagnosed from their respective logs and power rails.

Use the following operational workaround:

1. Keep both arms supported and the E-stop ready. Allow the safety exception to
   stop the process; do not disable the command-stream checks or increase the
   hard limit merely to hide the interruption.
2. Confirm how many episodes were committed in `meta/info.json`. The episode
   named immediately before the exception might not have been saved, so use
   metadata rather than the last console message as the source of truth.
3. Close unnecessary applications and keep `DISPLAY_DATA=false`, H.264, and
   `PARALLEL_VIDEO_ENCODING=false`. If recording was running at 15 FPS, retry at
   10 FPS to leave more CPU, USB, and EnhancedDepthFilter headroom.
4. Check Jetson cooling and input power. If USB power is suspected, test the
   cameras on a suitable externally powered USB 3 hub while preserving enough
   bandwidth, and separately inspect the 24 V arm supply and MotorBridge links.
5. Restart with the same root/repository id, `--resume=true`, and only the
   remaining number of episodes. For example:

```shell
bash examples/rebot_b601_smolvla_record/record_b601_smolvla_rgbd.sh \
  --num-episodes 3 \
  --fps 10 \
  --parallel-video-encoding false \
  --extra --resume=true
```

For root-cause investigation, collect synchronized system and application logs.
Run these in separate terminals before starting collection:

```shell
mkdir -p ~/rebot_diag
sudo tegrastats --interval 1000 2>&1 | tee ~/rebot_diag/tegrastats.log
```

```shell
sudo journalctl -kf 2>&1 | tee ~/rebot_diag/kernel.log
```

Capture the recorder output as well:

```shell
set -o pipefail
bash examples/rebot_b601_smolvla_record/record_b601_smolvla_rgbd.sh \
  2>&1 | tee ~/rebot_diag/record.log
```

Correlate the timestamp of each `completed-send gap` with CPU load, clocks,
temperature, throttling or over-current indications, USB/xHCI resets,
`ttyACM` disconnects, and DM status codes. Also compare camera-free
teleoperation, RGB-only recording, RGB-D without EnhancedDepthFilter, and the
full workload to isolate which added load triggers the stall.

This restart-and-resume procedure is a practical recovery path, not the final
fix. Reproduction reports with synchronized logs, root-cause analyses, and
patches that solve the underlying stall are very welcome so future users can
run the complete collection without intermittent restarts.

Do not continue recording if the follower arm repeatedly reports relative goal
clamping on several arm joints. That means the leader command has moved far
away from the follower's current pose, often because the follower is near a
singular or mechanically weak posture. The B601 follower now enters a software
safety hold when multiple arm joints are heavily clamped: it commands the
current follower pose instead of chasing the unreachable target.

This is a joint-command tracking safety mechanism. It detects large leader to
follower position gaps through repeated relative-target clamping. It is not a
full kinematic singularity detector. MotorBridge exposes a raw DM status code.
This integration decodes the normal `DISABLED`/`ENABLED` states and the
documented voltage, current, temperature, communication, and overload faults.
A drive that has already disabled itself may be unable to execute automatic
recovery; an unexpected `DISABLED` state or a real drive-fault status aborts
recording without trying to re-enable it.

At the start of every episode, the recorder stores the follower and leader arm
joint positions. If an arm safety hold occurs during recording or environment
reset, the workflow is:

1. Stop the active loop before adding the fault-triggering frame to the dataset.
2. Move only follower `joint_1` through `joint_6` back toward the stored start
   pose using small bounded position steps. The gripper is not moved by recovery.
   The leader action is still refreshed in this loop so gravity compensation
   continues to follow the leader's current posture.
3. Wait for the operator to manually return the gravity-compensated leader arm
   near its stored start pose.
4. Clear the incomplete episode buffer and record the same episode index again.

For non-faulting B601 frames, the dataset stores the follower command returned
by `send_action()`, including any ordinary per-frame safety limiting, rather
than the unbounded leader target. This keeps SmolVLA action labels consistent
with the commands that the follower actually received.

If follower recovery or leader return times out, recording stops instead of
continuing from an unknown pose. During automatic recovery, press Right Arrow
or `Esc` to cancel. Always use the hardware E-stop if motion is unsafe.

For collection, the recording script uses conservative follower defaults:

```text
FOLLOWER_MAX_RELATIVE_TARGET=12.0
FOLLOWER_GRIPPER_MAX_RELATIVE_TARGET=30.0
FOLLOWER_DISABLE_TORQUE_ON_DISCONNECT=false
FOLLOWER_COMMAND_STREAM_ENABLED=true
FOLLOWER_COMMAND_STREAM_HZ=100
FOLLOWER_COMMAND_STREAM_MAX_FAILURES=5
FOLLOWER_COMMAND_STREAM_MAX_GAP_S=0.25
FOLLOWER_ABORT_ON_MOTOR_FAULT_STATUS=true
FOLLOWER_MOTOR_FEEDBACK_MAX_MISSES=3
FOLLOWER_RUNTIME_ERROR_HOLD_S=15
LEADER_COMMAND_STREAM_ENABLED=true
LEADER_COMMAND_STREAM_HZ=100
LEADER_COMMAND_STREAM_MAX_FAILURES=5
LEADER_COMMAND_STREAM_MAX_GAP_S=0.25
LEADER_ABORT_ON_MOTOR_FAULT_STATUS=true
LEADER_MOTOR_FEEDBACK_MAX_MISSES=3
FOLLOWER_SAFETY_HOLD_ON_RELATIVE_CLAMP=true
FOLLOWER_SAFETY_ABORT_EPISODE_ON_HOLD=true
FOLLOWER_SAFETY_AUTO_RECOVER_TO_EPISODE_START=true
FOLLOWER_SAFETY_RECOVERY_STEP_DEG=1.5
FOLLOWER_SAFETY_RECOVERY_HZ=20
FOLLOWER_SAFETY_RECOVERY_TIMEOUT_S=25
FOLLOWER_SAFETY_RECOVERY_TOLERANCE_DEG=3.0
FOLLOWER_SAFETY_RECOVERY_POSITION_KP_SCALE=0.5
FOLLOWER_SAFETY_WAIT_FOR_LEADER_START=true
FOLLOWER_SAFETY_LEADER_START_TOLERANCE_DEG=8.0
FOLLOWER_SAFETY_LEADER_START_TIMEOUT_S=45
```

The episode start pose must itself be a safe, supported, non-singular pose.
For the first hardware test, reduce recovery speed and record one short episode:

```shell
bash examples/rebot_b601_smolvla_record/record_b601_smolvla_rgbd.sh \
  --num-episodes 1 \
  --episode-time-s 20 \
  --reset-time-s 10 \
  --follower-max-relative-target 4.0 \
  --follower-safety-recovery-step-deg 1.0 \
  --follower-safety-recovery-hz 15 \
  --follower-safety-recovery-timeout-s 30
```

Do not deliberately drive the arm into a singularity to test this feature.
Instead, while supporting the follower and staying near the neutral pose, move
the leader far enough ahead to trigger `safety hold active`. Confirm that the
current episode is discarded, the follower returns slowly, and recording only
restarts after the leader is also near its start pose.

`FOLLOWER_DISABLE_TORQUE_ON_DISCONNECT=false` prevents a Python exception or
Escape stop from immediately disabling all follower motor torque. This avoids a
software-triggered drop, but it also means the follower may remain powered after
the script exits. Keep one hand near the power/E-stop, support the arm before
disconnecting power, and do not leave the powered arm unattended.

Before each episode, place the leader and follower in similar non-singular
poses: avoid fully stretched elbow poses, wrist-flip extremes, and postures
where the load hangs far from the shoulder. Keep the banana and bottle task
workspace inside the comfortable middle area of the arm instead of at the edge
of reach.

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

The tuned defaults already use endpoint mapping for the gripper, so the
following scale/offset options are only useful if you deliberately clear the
four endpoint parameters and go back to direct scaling.

If the follower gripper still does not open far enough in direct-scaling mode,
scale only the follower gripper target instead of forcing the leader gripper
farther:

```shell
--robot.gripper_action_scale=1.5
```

If the follower gripper still lags behind in direct-scaling mode, keep the arm
safety limit but give the gripper its own relative limit and gains:

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

Recommended script:

```shell
bash examples/rebot_b601_smolvla_record/record_b601_smolvla_rgbd.sh
```

Before running it, edit the `User settings` block at the top of
`record_b601_smolvla_rgbd.sh`, especially `DATASET_ROOT`, `TOP_SERIAL`,
`WRIST_SERIAL`, `ORBBEC_BRIDGE`, and `LINGBOT_MODEL`. The script writes the
dataset locally by default and does not push to the Hub. Run with `--help` only
when you want temporary command line overrides.

With the Gemini 335L RGB + depth + EnhancedDepthFilter stream and the Gemini
305 wrist RGB stream running together at 640x480, the Jetson Orin NX test setup
has shown an effective visual rate of about 10 Hz. Keep the dataset FPS aligned
with that measured throughput unless you lower the camera resolution, disable
EnhancedDepthFilter, or otherwise verify a faster stable rate.

`TOP_SERIAL` must be the OrbbecSDK serial number of the Gemini 335L, not a
`/dev/video*` path. If the bridge reports available serials such as
`CV2TC5100075, CP3L44P0001N`, test each serial once and keep the one belonging
to the top Gemini 335L. `WRIST_SERIAL` should be the Gemini 305 serial. Avoid
using OpenCV `/dev/video*` nodes for Orbbec RGB-D cameras unless you have
verified the node is a normal decoded color stream; some nodes expose raw
depth/IR/metadata and appear as green speckle images when interpreted as RGB.

Equivalent expanded command:

```shell
lerobot-record \
  --robot.type=rebot_b601_follower \
  --robot.port=/dev/ttyACM1 \
  --robot.transport=motorbridge \
  --robot.id=b601_follower \
  --robot.cameras='{
    wrist: {
      type: orbbec,
      serial_number: "GEMINI305_SERIAL",
      bridge_binary: "/home/r/ws/rebot_lerobot/lerobot/src/lerobot/cameras/orbbec/cpp/build/orbbec_rgbd_bridge",
      width: 640,
      height: 480,
      fps: 10,
      warmup_s: 15,
      timeout_ms: 15000,
      record_color: true,
      use_depth: false,
      align_depth_to_color: false,
      record_depth: false,
      record_depth_viz: false
    },
    top: {
      type: orbbec,
      serial_number: "GEMINI335L_SERIAL",
      bridge_binary: "/home/r/ws/rebot_lerobot/lerobot/src/lerobot/cameras/orbbec/cpp/build/orbbec_rgbd_bridge",
      width: 640,
      height: 480,
      fps: 10,
      warmup_s: 25,
      timeout_ms: 25000,
      record_color: true,
      use_depth: true,
      record_depth: true,
      depth_key: "depths.top",
      record_depth_viz: true,
      depth_viz_key: "top_depth",
      depth_viz_min_mm: 250,
      depth_viz_max_mm: 1800,
      align_depth_to_color: true,
      align_depth_to_color_mode: "sw",
      use_enhanced_depth_filter: true,
      enhanced_depth_filter_name: "EnhancedDepthFilter",
      enhanced_depth_model_path: "/home/r/ws/OrbbecSDK_v2/extensions/LingBot-Depth/model.sm4",
      enhanced_depth_confidence_key: "confidence_threshold",
      enhanced_depth_confidence_threshold: 51
    }
  }' \
  --teleop.type=rebot_b601_leader \
  --teleop.port=/dev/ttyACM0 \
  --teleop.transport=motorbridge \
  --teleop.id=b601_leader \
  --teleop.manual_control_mode=gravity_comp \
  --dataset.repo_id="${HF_USER}/rebot_b601_banana_bottle_rgbd" \
  --dataset.fps=10 \
  --dataset.num_episodes=50 \
  --dataset.episode_time_s=45 \
  --dataset.reset_time_s=20 \
  --dataset.single_task="Arrange the banana model and the transparent plastic cola bottle back to their assigned places on the desktop" \
  --display_data=false
```

The wrist Gemini 305 is read through OrbbecSDK instead of OpenCV because the
camera exposes multiple `/dev/video*` nodes. Some of those nodes are not normal
decoded RGB streams and can look like green speckle images if OpenCV interprets
the raw data as BGR.

## Merge Batches

You can record multiple batches into separate local directories, review/delete
bad episodes in each batch, and then merge the accepted batches into a new
dataset directory for training.

For each batch, change these values:

```shell
DATASET_ROOT="/home/r/ws/rebot_lerobot/datasets/rebot_b601_banana_bottle_rgbd_batch01"
DATASET_REPO_ID="local/rebot_b601_banana_bottle_rgbd_batch01"
```

Keep the schema-defining values identical across all batches:

```shell
WIDTH=640
HEIGHT=480
FPS=10
TOP_SERIAL="CP3L44P0001N"
WRIST_SERIAL="CV2TC5100075"
TOP_DEPTH_ALIGN_MODE="sw"
ENHANCED_DEPTH_FILTER_NAME="EnhancedDepthFilter"
ENHANCED_DEPTH_CONFIDENCE_KEY="confidence_threshold"
```

Also keep the camera keys and depth keys unchanged: `wrist`, `top`,
`top_depth`, and `depths.top`. The source directories may differ, but all
datasets must have the same feature names, shapes, fps, robot type, and
EnhancedDepthFilter/depth settings.

Recommended merge script:

```shell
bash examples/rebot_b601_smolvla_record/merge_b601_datasets.sh
```

Before running it, edit `SOURCE_ROOTS`, `SOURCE_REPO_IDS`, `OUTPUT_ROOT`, and
`OUTPUT_REPO_ID` in the script. Use the merged `OUTPUT_ROOT` and
`OUTPUT_REPO_ID` when training.

## Train SmolVLA

Recommended script:

```shell
bash examples/rebot_b601_smolvla_record/train_smolvla_local.sh
```

Before running it, edit the `User settings` block at the top of
`train_smolvla_local.sh`, especially `DATASET_ROOT`, `DATASET_REPO_ID`, and
`OUTPUT_DIR`.

For RTX 4090 24GB, start with `--batch-size 32 --use-amp true`. If CUDA memory
is tight, lower `--batch-size` to 16.

Equivalent expanded command:

```shell
lerobot-train \
  --policy.path=lerobot/smolvla_base \
  --dataset.repo_id="${HF_USER}/rebot_b601_banana_bottle_rgbd" \
  --dataset.root=/home/r/datasets/rebot_b601_banana_bottle_rgbd \
  --batch_size=32 \
  --steps=80000
```

The raw depth tensor is stored in the dataset but is not consumed by the stock
SmolVLA visual encoder. Use `observation.images.top_depth` for depth-aware
training without modifying SmolVLA internals. The SmolVLA preprocessor removes
`observation.depths.*` before device transfer, so the preserved uint16 tensor
does not consume GPU memory during stock SmolVLA training or edge inference.
When loading `lerobot/smolvla_base`, the policy factory also rebinds its generic
`camera1`/`camera2`/`camera3` and six-dimensional embodiment features to this
dataset's `wrist`/`top`/`top_depth`, 21-dimensional state, and seven-dimensional
action features while retaining the pretrained weights.

Before a full cloud run, start a short training smoke test:

```shell
bash examples/rebot_b601_smolvla_record/train_smolvla_local.sh \
  --dataset-root /path/to/rebot_b601_rgbd_dataset \
  --repo-id local/rebot_b601_rgbd_dataset \
  --output-dir /tmp/rebot_b601_smolvla_smoke \
  --batch-size 2 \
  --steps 2 \
  --save-freq 2 \
  --num-workers 0
```

The startup log must bind `observation.state` with shape `(21,)`, the visual
keys `observation.images.wrist`, `observation.images.top`, and
`observation.images.top_depth`, plus `action` with shape `(7,)`. Stop if the
base checkpoint's generic `camera1`/`camera2`/`camera3` keys remain.

## LingBot EnhancedDepthFilter Notes

The Orbbec LingBot EnhancedDepthFilter must run inside the Orbbec C++ bridge,
before depth bytes are sent to Python. This keeps data collection, training,
and edge inference on the same visual distribution.

Use the `model.sm4`, extension libraries, and device-stored license from the
same OrbbecSDK release. The bridge creates the SDK private filter with:

```text
FilterFactory::createPrivateFilter("EnhancedDepthFilter", model.sm4)
```

and applies it to the synchronized color+depth `FrameSet` before Python sees
the frame. If the SDK, extension, model, TensorRT/CUDA runtime, or license is
not valid, recording stops instead of falling back to raw depth.

If the LingBot-Depth license has already been written into the camera, leave
`enhanced_depth_license_check_command` unset. OrbbecSDK reads and validates the
device license when the C++ bridge creates and runs `EnhancedDepthFilter`.

If your SDK release also provides an external LicenseTool and you want an
extra preflight check before starting the bridge, configure
`enhanced_depth_license_check_command` with the exact check/verify command. The
placeholders `{serial_number}` and `{model_path}` are expanded before the
command runs. If LicenseTool exits with a non-zero code, recording stops before
the camera bridge starts.

If your SDK release uses a different private filter name or config schema key,
override `enhanced_depth_filter_name` or `enhanced_depth_confidence_key` in the
camera config without changing the bridge source.
