"""
Assignment 1 - MiR100 Differential-Drive Odometry & Go-to-Pose Controller
File: controllers/student_mir/student_mir.py

Kinematics & Sensor Fusion:
- Linear displacement ds is computed from middle wheel encoders (r = 0.0625 m).
- Angular heading theta fuses calibrated differential-drive kinematics
  (L_eff = 0.4662 m, measured experimentally) with the onboard Z-axis gyroscope
  ('imu_data gyro') to eliminate caster-wheel slip drift during turns.
"""

import json
import math
import os
from controller import Robot


# ---------------------------------------------------------------------------
# Robot Geometry & Tuning Constants
# ---------------------------------------------------------------------------
WHEEL_RADIUS = 0.0625       # [m] nominal driven wheel radius (docs/03)
WHEEL_SEPARATION = 0.4662   # [m] experimentally calibrated effective track width (nominal 0.4452 m)

MAX_LINEAR_SPEED = 0.65     # [m/s] cruising speed in open aisle
APPROACH_SPEED = 0.22     # [m/s] final straight-line docking speed
MIN_LINEAR_SPEED = 0.05     # [m/s] minimum crawl speed near target
MAX_ANGULAR_SPEED = 0.65    # [rad/s] controlled rotation speed to prevent caster slip
MIN_ANGULAR_SPEED = 0.10    # [rad/s] minimum turn speed to overcome static friction

KP_LINEAR = 1.4             # proportional gain for distance
KP_ANGULAR_DRIVE = 2.6      # heading correction gain while driving forward
KP_ANGULAR_TURN = 2.2       # angular gain while turning in place

HEADING_ALIGN_TOL = 0.04    # [rad] angle threshold to transition from turn -> drive
WAYPOINT_POS_TOL = 0.035    # [m] position tolerance for intermediate waypoints
DOCK_POS_TOL = 0.012        # [m] tight position tolerance for final station dock
DOCK_YAW_TOL = 0.012        # [rad] tight heading tolerance for final station dock

PRE_DOCK_DIST = 0.55        # [m] straight-line approach distance along station heading
DOCK_HOLD_TIME = 1.30       # [s] must exceed 1.0 s below 0.03 m/s to register dock
TARGET_BATTERY_PCT = 96.0   # [%] wait at charger until battery > 95 %


def normalize_angle(angle: float) -> float:
    """Wrap an angle in radians to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def load_layout() -> dict:
    """Load station and start coordinates from config/layout.json."""
    ctrl_dir = os.path.dirname(os.path.abspath(__file__))
    layout_path = os.path.normpath(os.path.join(ctrl_dir, "..", "..", "config", "layout.json"))
    with open(layout_path, "r", encoding="utf-8") as f:
        return json.load(f)


def parse_pose(pose_data, default_heading: float = 0.0, robot_index: int = 0) -> tuple:
    """Safely extract (x, y, heading) from list, nested-list, or dict formats in layout.json."""
    if isinstance(pose_data, dict):
        return (
            float(pose_data["x"]),
            float(pose_data["y"]),
            float(pose_data.get("heading", pose_data.get("yaw", default_heading))),
        )
    if isinstance(pose_data, (list, tuple)):
        if len(pose_data) > 0 and isinstance(pose_data[0], (list, tuple, dict)):
            idx = robot_index if robot_index < len(pose_data) else 0
            return parse_pose(pose_data[idx], default_heading=default_heading)
        if len(pose_data) >= 3:
            return float(pose_data[0]), float(pose_data[1]), float(pose_data[2])
        if len(pose_data) == 2:
            return float(pose_data[0]), float(pose_data[1]), default_heading
    raise ValueError(f"Unsupported pose format in layout.json: {pose_data!r}")


class DifferentialDriveOdometry:
    """
    Estimates 2D robot pose (x, y, theta) from left/right middle wheel encoders,
    fusing Z-axis gyroscope yaw rate to eliminate lateral caster-wheel slip drift.
    """

    def __init__(self, x0: float, y0: float, theta0: float, radius: float, axle_len: float):
        self.x = x0
        self.y = y0
        self.theta = normalize_angle(theta0)
        self.pure_wheel_theta = normalize_angle(theta0)
        self.r = radius
        self.L = axle_len
        self.prev_left = None
        self.prev_right = None

    def update(self, left_angle: float, right_angle: float, gyro_wz: float, dt: float) -> tuple:
        """Update pose using midpoint integration of encoder displacement and gyro heading."""
        if math.isnan(left_angle) or math.isnan(right_angle):
            return self.x, self.y, self.theta

        if self.prev_left is None or self.prev_right is None:
            self.prev_left = left_angle
            self.prev_right = right_angle
            return self.x, self.y, self.theta

        d_left = left_angle - self.prev_left
        d_right = right_angle - self.prev_right
        self.prev_left = left_angle
        self.prev_right = right_angle

        # Arc lengths traveled by each driven wheel [m]
        ds_left = self.r * d_left
        ds_right = self.r * d_right

        # Forward center displacement [m] and wheel-only rotation [rad]
        ds = 0.5 * (ds_right + ds_left)
        d_theta_wheels = (ds_right - ds_left) / self.L
        self.pure_wheel_theta = normalize_angle(self.pure_wheel_theta + d_theta_wheels)

        # Use gyro yaw rate when valid; fallback to calibrated wheel odometry otherwise
        if gyro_wz is not None and not math.isnan(gyro_wz):
            d_theta = gyro_wz * dt
        else:
            d_theta = d_theta_wheels

        # Midpoint integration reduces curved-path discretization error
        theta_mid = self.theta + 0.5 * d_theta
        self.x += ds * math.cos(theta_mid)
        self.y += ds * math.sin(theta_mid)
        self.theta = normalize_angle(self.theta + d_theta)

        return self.x, self.y, self.theta


def build_station_waypoints(curr_x: float, curr_y: float, station_pose: tuple) -> list:
    """
    Generate obstacle-free waypoints through the central aisle (y ~ 0) ending with
    a straight 0.55 m approach aligned with the target station's docking heading.
    """
    sx, sy, stheta = station_pose
    waypoints = []

    def add_wp(wx: float, wy: float, heading=None, is_dock: bool = False):
        if waypoints:
            last = waypoints[-1]
            if math.hypot(wx - last["x"], wy - last["y"]) < 0.08 and not is_dock:
                return
        elif math.hypot(wx - curr_x, wy - curr_y) < 0.08 and not is_dock:
            return
        waypoints.append({"x": wx, "y": wy, "heading": heading, "is_dock": is_dock})

    # 1. If near the west charger wall, pull east into the main aisle first
    if curr_x < -4.8:
        add_wp(-4.8, curr_y)

    # 2. If near a north input dock or south output dock, exit to the central aisle (y = 0)
    if abs(curr_y) > 0.65:
        exit_x = max(curr_x, -4.8)
        add_wp(exit_x, 0.0)

    # 3. Compute pre-dock point 0.55 m behind the station along its target heading
    pre_x = sx - PRE_DOCK_DIST * math.cos(stheta)
    pre_y = sy - PRE_DOCK_DIST * math.sin(stheta)

    # 4. Travel along the central aisle (y = 0) to align with the target's approach lane
    if abs(pre_y) > 0.25:
        add_wp(pre_x, 0.0)

    # 5. Move to the pre-dock staging point
    add_wp(pre_x, pre_y)

    # 6. Final straight-line approach into the station dock pose
    add_wp(sx, sy, heading=stheta, is_dock=True)
    return waypoints


def set_wheel_velocities(left_motor, right_motor, v: float, omega: float, max_wheel_rad_s: float):
    """Convert desired linear velocity v [m/s] and angular velocity omega [rad/s] to wheel speeds."""
    w_left = (v - 0.5 * omega * WHEEL_SEPARATION) / WHEEL_RADIUS
    w_right = (v + 0.5 * omega * WHEEL_SEPARATION) / WHEEL_RADIUS

    peak = max(abs(w_left), abs(w_right))
    if peak > max_wheel_rad_s:
        scale = max_wheel_rad_s / peak
        w_left *= scale
        w_right *= scale

    left_motor.setVelocity(w_left)
    right_motor.setVelocity(w_right)


def main():
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())

    # Print all robot devices (Assignment 1 Part 1)
    print(f"[student_mir] Devices on {robot.getName()}:")
    for i in range(robot.getNumberOfDevices()):
        dev = robot.getDeviceByIndex(i)
        print(f"  - {dev.getName()} ({dev.getNodeType()})")

    # Initialize driven wheel motors
    left_motor = robot.getDevice("middle_left_wheel_joint")
    right_motor = robot.getDevice("middle_right_wheel_joint")
    left_motor.setPosition(float("inf"))
    right_motor.setPosition(float("inf"))
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)
    max_wheel_speed = min(left_motor.getMaxVelocity(), right_motor.getMaxVelocity())

    # Initialize middle wheel encoders for odometry
    left_sensor = robot.getDevice("middle_left_wheel_joint_sensor")
    right_sensor = robot.getDevice("middle_right_wheel_joint_sensor")
    left_sensor.enable(timestep)
    right_sensor.enable(timestep)

    # Initialize onboard gyroscope to prevent turn-slip heading drift
    gyro = robot.getDevice("imu_data gyro")
    if gyro is not None:
        gyro.enable(timestep)

    # Load static warehouse geometry
    layout = load_layout()
    start_x, start_y, start_theta = parse_pose(layout.get("mir_start", [-1.5, 0.0, 0.0]))
    stations = {
        name: parse_pose(coords)
        for name, coords in layout.get("stations", {}).items()
    }

    odom = DifferentialDriveOdometry(
        x0=start_x,
        y0=start_y,
        theta0=start_theta,
        radius=WHEEL_RADIUS,
        axle_len=WHEEL_SEPARATION,
    )

    route = []
    route_idx = 0
    waypoints = []
    wp_idx = 0
    extra_charge_trip = False

    state = "WAIT_MISSION"
    hold_elapsed = 0.0
    last_battery_print_t = -10.0

    while robot.step(timestep) != -1:
        sim_time = robot.getTime()
        dt = timestep / 1000.0

        # 1. Read sensors and update pose estimate
        gyro_wz = gyro.getValues()[2] if gyro is not None else None
        x, y, theta = odom.update(left_sensor.getValue(), right_sensor.getValue(), gyro_wz, dt)

        # 2. Parse battery and mission route from customData
        raw_data = robot.getCustomData()
        status = json.loads(raw_data) if raw_data else {}
        battery_level = status.get("level")

        if battery_level is not None and (sim_time - last_battery_print_t) >= 10.0 - 1e-6:
            print(
                f"[student_mir] t={sim_time:6.1f}s | Battery: {battery_level:5.1f}% | "
                f"Pose=({x:.2f}, {y:.2f}, {theta:.2f})"
            )
            last_battery_print_t = sim_time

        if not route and "mission" in status and "route" in status["mission"]:
            route = list(status["mission"]["route"])
            print(f"[student_mir] Received mission route: {route}")
            if route:
                waypoints = build_station_waypoints(x, y, stations[route[0]])
                wp_idx = 0
                state = "TURN_TO_WP"

        # 3. State machine
        if state == "WAIT_MISSION":
            set_wheel_velocities(left_motor, right_motor, 0.0, 0.0, max_wheel_speed)

        elif state == "TURN_TO_WP":
            wp = waypoints[wp_idx]
            dx = wp["x"] - x
            dy = wp["y"] - y
            dist = math.hypot(dx, dy)

            if dist < (DOCK_POS_TOL if wp["is_dock"] else WAYPOINT_POS_TOL):
                state = "FINAL_ALIGN" if wp["is_dock"] else "NEXT_WP"
                continue

            target_heading = math.atan2(dy, dx)
            alpha = normalize_angle(target_heading - theta)

            if abs(alpha) <= HEADING_ALIGN_TOL:
                state = "DRIVE_TO_WP"
            else:
                omega = KP_ANGULAR_TURN * alpha
                omega = math.copysign(max(MIN_ANGULAR_SPEED, min(MAX_ANGULAR_SPEED, abs(omega))), omega)
                set_wheel_velocities(left_motor, right_motor, 0.0, omega, max_wheel_speed)

        elif state == "DRIVE_TO_WP":
            wp = waypoints[wp_idx]
            dx = wp["x"] - x
            dy = wp["y"] - y
            dist = math.hypot(dx, dy)
            pos_tol = DOCK_POS_TOL if wp["is_dock"] else WAYPOINT_POS_TOL

            if dist <= pos_tol:
                set_wheel_velocities(left_motor, right_motor, 0.0, 0.0, max_wheel_speed)
                state = "FINAL_ALIGN" if wp["is_dock"] else "NEXT_WP"
                continue

            target_heading = math.atan2(dy, dx)
            alpha = normalize_angle(target_heading - theta)

            if abs(alpha) > 0.30:
                state = "TURN_TO_WP"
                continue

            v_cap = APPROACH_SPEED if wp["is_dock"] else MAX_LINEAR_SPEED
            v = min(v_cap, max(MIN_LINEAR_SPEED, KP_LINEAR * dist))
            v *= max(0.3, math.cos(alpha))

            omega = KP_ANGULAR_DRIVE * alpha
            omega = max(-MAX_ANGULAR_SPEED, min(MAX_ANGULAR_SPEED, omega))
            set_wheel_velocities(left_motor, right_motor, v, omega, max_wheel_speed)

        elif state == "FINAL_ALIGN":
            wp = waypoints[wp_idx]
            yaw_err = normalize_angle(wp["heading"] - theta)

            if abs(yaw_err) <= DOCK_YAW_TOL:
                set_wheel_velocities(left_motor, right_motor, 0.0, 0.0, max_wheel_speed)
                hold_elapsed = 0.0
                state = "DOCK_HOLD"
            else:
                omega = KP_ANGULAR_TURN * yaw_err
                omega = math.copysign(max(MIN_ANGULAR_SPEED, min(MAX_ANGULAR_SPEED, abs(omega))), omega)
                set_wheel_velocities(left_motor, right_motor, 0.0, omega, max_wheel_speed)

        elif state == "DOCK_HOLD":
            set_wheel_velocities(left_motor, right_motor, 0.0, 0.0, max_wheel_speed)
            hold_elapsed += dt

            if hold_elapsed >= DOCK_HOLD_TIME:
                if not extra_charge_trip:
                    station_name = route[route_idx]
                    print(
                        f"[student_mir] t={sim_time:6.1f}s | DOCKED at '{station_name}' "
                        f"({route_idx + 1}/{len(route)}) | Est Pose=({x:.3f}, {y:.3f}, {theta:.3f})"
                    )
                    route_idx += 1

                    if route_idx < len(route):
                        next_station = route[route_idx]
                        waypoints = build_station_waypoints(x, y, stations[next_station])
                        wp_idx = 0
                        state = "TURN_TO_WP"
                    else:
                        if station_name != "charger" and (battery_level is None or battery_level < TARGET_BATTERY_PCT):
                            print(f"[student_mir] t={sim_time:6.1f}s | Route complete; heading to charger...")
                            extra_charge_trip = True
                            waypoints = build_station_waypoints(x, y, stations["charger"])
                            wp_idx = 0
                            state = "TURN_TO_WP"
                        else:
                            print(f"[student_mir] t={sim_time:6.1f}s | All stations visited. Waiting to charge > 95%...")
                            state = "CHARGING_WAIT"
                else:
                    print(f"[student_mir] t={sim_time:6.1f}s | Arrived at charger. Waiting to charge > 95%...")
                    state = "CHARGING_WAIT"

        elif state == "NEXT_WP":
            wp_idx += 1
            if wp_idx < len(waypoints):
                state = "TURN_TO_WP"
            else:
                state = "FINAL_ALIGN"

        elif state == "CHARGING_WAIT":
            set_wheel_velocities(left_motor, right_motor, 0.0, 0.0, max_wheel_speed)
            if battery_level is not None and battery_level >= TARGET_BATTERY_PCT:
                print(f"[student_mir] t={sim_time:6.1f}s | Battery charged to {battery_level:.1f}%. Mission complete!")
                state = "DONE"

        elif state == "DONE":
            set_wheel_velocities(left_motor, right_motor, 0.0, 0.0, max_wheel_speed)


if __name__ == "__main__":
    main()