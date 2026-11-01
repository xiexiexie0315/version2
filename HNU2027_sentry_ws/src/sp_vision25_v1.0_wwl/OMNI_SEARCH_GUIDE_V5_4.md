# SP Vision v5.4 Omni Search Guide

This version keeps the original self-aim entry unchanged:

```bash
./build-serial/standard_mpc_serial configs/standard3.yaml
```

The new optional entry is:

```bash
./build-serial/standard_mpc_serial_omni_search configs/standard3.yaml
```

It uses the HikRobot main camera for normal aiming. Two USB side cameras only
search for armor and turn the gimbal toward the target. They do not shoot and
do not replace the main camera aiming pipeline.

## Build

```bash
cd ~/sp_vision25_serial_omni_search_v5_4
rm -rf build-serial

cmake -S . -B build-serial \
  -DCMAKE_BUILD_TYPE=Release \
  -DSP_VISION_DISABLE_ROS2=ON

cmake --build build-serial \
  --target nav_serial_wire_test standard_mpc_serial standard_mpc_serial_omni_search -j2

./build-serial/nav_serial_wire_test
```

## Find USB Camera Devices

List video devices:

```bash
ls /dev/video*
```

Preview one camera at a time:

```bash
ffplay -f v4l2 -video_size 640x480 /dev/video0
ffplay -f v4l2 -video_size 640x480 /dev/video2
ffplay -f v4l2 -video_size 640x480 /dev/video4
```

If `ffplay` is not installed, use:

```bash
sudo apt install ffmpeg
```

After confirming which camera is left/right, edit:

```yaml
omni_search_cameras:
  - {name: "left", device: "video0", yaw_center_deg: -60}
  - {name: "right", device: "video2", yaw_center_deg: 60}
```

Use `video0`, not `/dev/video0`; both formats are accepted, but the short form
is simpler.

## Run

Run only one serial owner. Do not run the navigation serial node at the same
time.

```bash
./build-serial/standard_mpc_serial_omni_search configs/standard3.yaml
```

Expected behavior:

- Main HikRobot camera sees armor: normal self-aim controls the gimbal.
- Main camera loses armor, side USB camera sees armor: gimbal turns roughly
  toward that side target.
- No camera sees armor: tracking is cleared and the gimbal receives safe zero
  commands.

## First Test

Keep firing hardware disabled.

Set the side camera devices correctly first. If the left camera makes the
gimbal turn right, swap the sign:

```yaml
  - {name: "left", device: "video0", yaw_center_deg: 60}
```

or swap the left/right camera entries.

If turning is too aggressive, reduce:

```yaml
omni_search_yaw_limit_deg: 45
omni_search_pitch_limit_deg: 10
```

If side cameras do not detect reliably, tune:

```yaml
usb_exposure: 80
usb_gain: 20
image_width: 640
image_height: 480
```

The current FOV values are rough:

```yaml
omni_search_fov_h_deg: 60
omni_search_fov_v_deg: 45
```

They are enough for first-stage search. Later calibration can replace these
with real values.
