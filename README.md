# AirTag Locator

A ROS 2 (Python) control stack for a 3-axis gimbal robot that aims a directional
Yagi antenna to find a hidden Bluetooth tracker, such as an Apple AirTag, placed
on a vehicle. The robot sweeps the antenna, measures the tracker's Bluetooth
signal strength (RSSI) at each angle, and points at the strongest direction.
A laser on the barrel marks where to look.

Built as a vehicle anti-stalking / security research project.

## How it works

```
BLE sniffers + servos + IMUs (ESP32)
        │  JSON over USB serial (115200 baud)
        ▼
   bridge ──────────────► ROS 2 topics (servo positions, IMU data, BLE advertisements)
        ▲                          │
        │ bridge_send              ▼
        │                  strongestMAC ──► candidate trackers, ranked by RSSI
        │                          │
        │                          ▼  operator picks a target in the GUI
        ├── horizontal_scan_node   (base / yaw axis)
        ├── vertical_scan_node     (tilt axis)
        └── robot_control_gui      (PyQt5 control panel)
```

1. **Detect.** `bridge` reads JSON packets from the ESP32 and publishes BLE
   advertisements, servo positions, and IMU orientation/calibration as ROS 2 topics.
2. **Pick a target.** `strongestMAC` tracks the RSSI of every nearby device,
   prunes stale readings, lets the operator blacklist known devices (a phone,
   the car's own Bluetooth), and publishes the chosen tracker. If the target is
   not seen for 60 s it is reported as lost.
3. **Sweep horizontally.** `horizontal_scan_node` narrows the yaw range with a
   ternary-style search: it scores two probe angles, discards the weaker third,
   and repeats until the window is about 3°, then checks each remaining degree.
4. **Sweep vertically.** `vertical_scan_node` repeats the search on the tilt axis.
5. **Point.** The robot holds the best angle and the operator can switch the laser on.

Each angle is scored from 15 advertisements from the target (mean + standard
deviation of RSSI). Single Bluetooth readings are noisy, so a larger sample
gives a steadier estimate. If the operator switches targets mid-scan, the
samples are flushed and collection restarts.

## Nodes

| Executable | Role |
|---|---|
| `bridge` | Serial ⇄ ROS 2 bridge. A dedicated RX thread reads the port; commands arrive on `bridge_send` and are written to the ESP32. |
| `strongestMAC` | Candidate tracking, blacklist, target selection, lost-target detection. |
| `horizontal_scan_node` | Yaw search on the base servo (clamped to ±90°). |
| `vertical_scan_node` | Search on the tilt servo (clamped to ±90°). |
| `robot_control_gui` | PyQt5 panel: live servo and IMU readouts, IMU calibration status, manual jog buttons, target list, scan start/stop, laser on/off. |
| `bridge_send` | Terminal tool for sending raw JSON commands to the ESP32 (debugging). |

Key topics: `/strongest_mac/candidates`, `/strongest_mac/selected`,
`/strongest_mac/lost`, `/scanning_commands`, `/scanning_status`,
`/scan_bounds_update`, `/{base,tilt,barrel}/motor/position`,
`/{tilt,barrel}/imu/euler/{x,y,z}`, `/{tilt,barrel}/imu/calibration_status/*`.

## Run

Requires ROS 2, Python 3, `pyserial`, and `PyQt5`. The ESP32
firmware is not included in this repository.

```bash
# inside a colcon workspace: src/stacy_py
colcon build --packages-select stacy_py
source install/setup.bash
ros2 launch stacy_py start_full_system.launch.py
```

The launch file starts `bridge` first, then starts the other nodes 2 seconds
later so the serial link is up before they subscribe. The serial port defaults
to `/dev/ttyUSB0` (see `PORT` in `stacy_py/bridge.py`).

## License

Proprietary. All rights reserved. See [LICENSE](LICENSE).
