# 03 - Interface reference

What your controllers can see and do. The starter controllers print every device of their robot, so you can always check the exact names on your own.

## Controllers

| Controller | Robot(s) | You write it in |
|---|---|---|
| `student_mir` | every MiR100 | assignment 1 onwards (from assignment 2 with `libs/for_sensors.py`) |
| `student_arm` | all six UR3e arms (one program, one copy per arm; `robot.getName()` tells which arm: `ur3e_in_0..2`, `ur3e_out_0..2`) | assignment 3 onwards |

Language: Python. Standard library, numpy, scipy, OpenCV and other pip packages are allowed. The grading machine must be
able to run your code: list every non-standard package in your report.
The robots are plain `Robot` nodes: you have **no** supervisor access (no ground-truth positions, no moving objects).

## MiR100

Devices (printed by the starter controller):

| Device | Type | Use |
|---|---|---|
| `middle_left_wheel_joint`, `middle_right_wheel_joint` | RotationalMotor | the two driven wheels (differential drive) |
| `middle_left_wheel_joint_sensor`, `middle_right_wheel_joint_sensor` | PositionSensor | wheel encoders for odometry |
| `front_*/back_*_wheel_joint_sensor`, `*_caster_joint_sensor` | PositionSensor | passive wheels and casters, not needed for driving |
| `imu_data accelerometer`, `imu_data gyro`, `imu_data compass` | Accelerometer, Gyro, Compass | inertial data |
| `ds0` ... `ds3` | DistanceSensor | short-range distance sensors |
| `depth_camera` | RangeFinder | depth image |
| `front_lidar`, `back_lidar` | Lidar (SICK S300-like) | 2D laser scanners on opposite corners of the robot: `front_lidar` at the front left, `back_lidar` at the back right, so together they see all around (used from assignment 2). They are not at the robot center: account for their offset when you build a map. Read the field of view, resolution and range from the device: `getFov()`, `getHorizontalResolution()`, `getMaxRange()` |

* Drive with velocity control: `motor.setPosition(float('inf'))`, then `motor.setVelocity(rad_per_s)` on both middle wheels.
* Nominal geometry (from the official Webots MiR100 example controller): wheel radius **0.0625 m**, distance from the robot
  center to each driven wheel **0.2226 m** (wheel separation 0.4452 m). These are the manufacturer-style values of the model,
  not a guarantee about how the simulated robot behaves: test your odometry against the real motion.
* Typical speeds in the official example: up to 0.95 m/s forward, 0.3 rad/s turning in its keyboard demo; the motors accept
  more, but the battery drains with distance, so faster is not free.
* Heading 0 means the robot's front points along world +x (east). Positive wheel velocities on both wheels drive it forward.
* The MiR100 has no GPS: absolute position is something you estimate (assignment 1: odometry, assignment 2: localization).

### Sensor layer, lidar and pose report (assignment 2 onwards)

From assignment 2 the sensors are **realistic**: you read them through `libs/for_sensors.py`, which returns what a real
robot would measure (the raw simulated devices are ideal). Do not read the raw devices of the gyro, wheel sensors or lidars
around it: the grader re-runs your controller and checks for this.

```python
import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "libs"))
from for_sensors import Sensors, PoseLog
sensors = Sensors(robot)          # after Robot(); enables the devices
pose_log = PoseLog(robot)
while robot.step(timestep) != -1:
    sensors.update()              # once per step, first
    sensors.left_angle            # left / right middle wheel angle [rad], accumulated, quantized, slightly wrong scale
    sensors.right_angle
    sensors.gyro_z                # yaw rate [rad/s]: noise and a slowly drifting bias
    sensors.scans()               # {"front_lidar": [(x, y), ...], "back_lidar": [...]}  robot frame, scan order
    pose_log.write(x, y, theta)   # your estimated pose (world frame): compared with the true pose by the grader
```

| Sensor | Error model (typical datasheet values, preset `a2` -> `sensors`) |
|---|---|
| wheel encoders | 4096 counts per revolution; each wheel's radius is wrong by a fixed random factor (about 0.6 %, 1 sigma) |
| gyro (z) | white noise 3e-3 rad/s per sample; a bias of about 1.5e-4 rad/s (random sign, differs from run to run) that also wanders slowly |
| lidar | range noise 1.2 cm + 0.4 % of the range (1 sigma); 1 % of the rays return nothing |

* **Scans.** `sensors.scans()` returns points in the **robot frame** (x forward, y left, origin at the robot center); the offset
  and rotation of each scanner are already applied. Points of one scanner are in scan order, which is what line extraction needs.
  Rays with no return are left out. The scanners are two SICK S300-like scanners at diagonal corners; each has a 270 degree view, 540 rays and a range of 10 m.
* **Known map.** `config/line_map.json` lists straight faces in the world frame (outer walls, cabinet fronts, charger unit, dispenser
  fronts) as segments `[x1, y1, x2, y2]`; `libs/line_map.py` loads them (`load_map()`) and can score extracted lines against them
  (`evaluate_lines(lines, pose)`).
* **Pose report.** `PoseLog.write` stores your estimate at 5 Hz. After the run, `summary.json` -> `robots[].localization` holds
  the mean, final and maximum position error and the heading error, and `results/latest/localization_<robot>.csv` has both
  poses over time. If you never call `write`, localization is not scored.
* The status JSON in `customData` has two more fields in assignment 2: `sensors` (the error parameters above plus a `seed`)
  and `lidar_mounts` (scanner poses in the robot frame, already used by `Sensors`).

### Battery and mission status (customData)

The simulation writes a JSON status into the robot's `customData` once per second. Read it with `robot.getCustomData()`:

```json
{"type": "battery", "robot": "MIR100_1", "level": 73.4, "charging": false, "dead": false, "low": false,
 "mission": {"route": ["charger", "input_0", "output_0"]}}
```

* `level` percent, `charging` true while on the charger pad, `dead` true at 0 %, `low` true below 20 %.
* `mission` only exists in assignments that define one (assignment 1).

```python
import json
status = json.loads(robot.getCustomData() or "{}")     # {} until the first message (first second of the run)
level = status.get("level")
```

The same message is also broadcast on **channel 8** for controllers that have a Receiver, but `customData` is the
simplest way and the one assignment 1 uses.

## UR3e arms

* Joints: `shoulder_pan_joint`, `shoulder_lift_joint`, `elbow_joint`, `wrist_1_joint`, `wrist_2_joint`, `wrist_3_joint`
  (rotational motors with position sensors named `<joint>_sensor`).
* Gripper: Robotiq 2F-85 on every arm. Motors `ROBOTIQ 2F-85 Gripper::left finger joint` and
  `ROBOTIQ 2F-85 Gripper::right finger joint`, sensors `ROBOTIQ 2F-85 Gripper left finger joint sensor` and
  `ROBOTIQ 2F-85 Gripper right finger sensor`. Opening/closing range: check `motor.getMinPosition()` / `getMaxPosition()`.
* The arm base is at the coordinates in `layout.json -> arms`; the pick and place points are in `layout.json -> points`.
* Ready pose used by the starter code: `[0, -1.57, 1.57, -1.57, -1.57, 0]`.

## Orders

The list of **open orders** (orders whose tote is on an output belt and not yet complete) is written once per second into
the `customData` of **every MiR and every arm**, no Receiver needed:

* arms: `{"type": "orders", "time": 62.0, "open": [ <order>, ... ]}`
* MiR: the status JSON from above with an extra `"orders": [ <order>, ... ]`

```json
{"id": 1, "lane": 0, "items": {"blue": 3, "red": 2}, "time_limit_s": 300, "started": 5.0}
```

`lane` is the output lane (0, 1, 2) whose tote was created for this order; `started` is the simulation time at which the
tote appeared. An order disappears from the list when it is complete. Completed totes are counted by the manager; you
only see that an order is gone.

The same events are also broadcast on **channel 7** for controllers that add a Receiver (`new_order` and `order_complete`,
JSON with the fields above plus `duration_s`, `late`, `extra_items` on completion). Using `customData` is the simpler way.

## Layout file

`config/layout.json` is safe to read from a controller at run time (it never changes between runs):
`columns_x`, `stations` (`[x, y, heading]`), `mir_start`, `arms.input/output[i].base`, `points.input_pick`,
`points.output_place`, `points.basket_drop`, `tote`, `charging_station`, `input_stop_y`, `block_size`.
Do **not** read `config/world_config.json` for run-time decisions: the grader replaces it.

## Things that are not allowed in controllers

Accessing `Supervisor` functions, reading or writing anything in `results/` (except through `PoseLog`), `controllers/world_manager/` or other
controllers' folders, starting processes, or using the network. See `04_rules_and_grading.md`.
