# Assignment 1 - Setup, the world, and your first MiR100 controller

**Deadline:** 29 Aban (20 November) | **Preset:** `a1` | **Run length:** 360 s | **Group work:** up to 3 students

## Goals

* Install and run the simulation environment, and learn your way around the Webots scene tree and console.
* Understand how a mobile robot is controlled: devices, time step, the control loop.
* Implement **differential-drive odometry** and a **go-to-pose controller** and use them to park a MiR100 precisely at
  the warehouse stations, in a mission given at run time.
* Read the robot's battery over a message channel and charge when the mission says so.

Read `docs/01` to `docs/04` first.

## Part 0 - Setup (no points, but needed for everything else)

Follow `docs/01_setup_guide.md`. Run the unmodified project until the RUN FINISHED report appears. Put a screenshot of
the Webots window and of the report in your PDF report.

## Part 1 - Understand the world (report, 3 short answers)

Open the scene tree (left panel) and answer in two or three sentences each:

1. What is the `basicTimeStep` of this world, and what does it mean for your controller's `robot.step(...)` loop?
2. What is the difference between a `Robot` and a `Supervisor`? Which node in this world is a supervisor and what does
   it do for you? Why is your MiR not one?
3. List the devices of your MiR100 (copy the console output of the starter controller) and say which ones you will use
   for this assignment and why.

## Part 2 - The controller (`controllers/student_mir/`)

Preset `a1`: no blocks, no orders, no arm work. The MiR starts at (-1.5, 0) heading 0 with a battery at **60 %**.

### Mission

Dock at the stations of `mission.route` **in the given order**. The default route (public config) is

```
charger -> input_0 -> output_0 -> input_1 -> output_1 -> input_2 -> output_2 -> charger
```

The graded run uses a **different order** of the same seven stations. Your controller receives the route from the
simulation (the `mission` field of the status JSON in the robot's `customData`, see `docs/03`) and must work for any order. Station
coordinates are in `config/layout.json -> stations`.

A dock counts when you stay within **0.15 m / 0.20 rad** of the station pose and below **0.03 m/s** for **1 s**
(`docs/02`). Aim well inside the tolerance: precision is scored.

After the last stop at the charger, **stay there** until the battery is above 95 %.

### Requirements

1. **Odometry - implement this yourself.** Estimate the pose (x, y, heading) of the robot from the wheel position sensors.
   Derive the differential-drive equations from the wheel radius and axle length, and explain them in the report. Reason:
   this is the core of the lecture on kinematics, and every later assignment builds on it. You may not use ground-truth
   positions (the robot has no supervisor access anyway).
2. **Go-to-pose - implement this yourself.** A controller that drives from the current estimated pose to a goal pose and
   stops with the required heading (for example turn-to-goal, drive with a proportional heading correction, final
   alignment; or a polar-coordinates controller). Include speed limits and a sensible stopping rule. Use no motion-planning
   library.
3. **Mission handling.** Read the route from `customData`, follow it, and print a log line for every station reached.
4. **Battery.** Read the battery status from `customData`, print `level` every 10 s of simulated time, and wait at the charger as described.
5. **Odometry inputs.** Use the two middle wheel encoders (`middle_left/right_wheel_joint_sensor`). You may use the compass or
   gyro to check or correct your heading, but say so in the report. Start from the nominal wheel radius and separation in
   `docs/03` and show how you checked them by experiment.

### Hints

* Check your odometry first: drive a known distance and compare with the robot's pose in the simulation, then do a full
  turn on the spot.
* Approach each station so that your last half metre is a straight line along the station heading; alignment errors are
  much easier to remove far from the target.
* Drive fast on long legs and slow down near the goal, but remember the battery: distance costs 0.5 % per metre.
* The aisle is free of obstacles, but arms, belts and tote areas are not. Plan routes that stay in the aisle between the
  docks (y between about -1.1 and 0.8).
* Time and heading errors accumulate; test with several stations in a row, not only one.

## Part 3 - Report (PDF, at most 3 pages)

* Part 1 answers, screenshots from Part 0.
* Your odometry derivation and the go-to-pose controller (equations or pseudocode, plus the main parameters you chose).
* One plot or table showing your results (for example `results/latest/report.png` and the dock errors from
  `summary.json`).
* **Errors and noise.** Which sources of error did you find in your odometry (what you measured, with numbers: how large,
  in which situations, how you noticed)? How did you tackle each one, and what is still left? Support it with at least one
  experiment of your own (for example a straight-line test and a spin test, estimated against real pose).
* **Differential-drive kinematics by hand (handwritten, a scan or photo in the PDF).** The MiR100 is a differential-drive robot
  with wheel radius r = 0.0625 m and distance L = 0.4452 m between its two driven wheels. At one instant, seen from above, its
  left wheel turns at 10 rad/s forward and its right wheel at 6 rad/s backward. Assume a flat floor and no wheel slip. The robot's
  pose in the global frame is (x, y) = (5 cm, 10 cm) with heading 30 degrees. Calculate, with the full solution and units:
  * the linear velocity of each wheel relative to the ground;
  * the velocity of the robot's center in its own local frame;
  * the angular velocity of the robot;
  * the new position and heading in the global frame after a very short time step of 0.1 s (this is the update your odometry performs every step).
* **Swedish wheels.** Suppose the MiR100 were replaced by a platform with three Swedish (omnidirectional) wheels at 120 degrees.
  Sketch it and explain why it can drive sideways while your robot cannot. Which part of your controller (the go-to-pose
  controller and the docking) would become simpler, and which part of your odometry would become harder or less accurate? Would you choose it for this warehouse?
* The list of non-standard Python packages you used, and which AI tools you used and for what.

## Automatic score (100 points, run on the hidden configuration)

| Points | Criterion | Full points | Zero points |
|---|---|---|---|
| 45 | Stations docked in the required order | all 8 stops | none |
| 15 | Mean parking position error | <= 0.05 m | >= 0.15 m |
| 5 | Mean parking heading error | <= 0.05 rad | >= 0.20 rad |
| 15 | Time to finish the route (last dock) | <= 200 s | >= 340 s |
| 10 | Battery at the end of the run | >= 95 % | <= 60 % |
| 10 | Time with an empty battery | 0 s | >= 30 s |

Scores between the limits are linear.
The assignment's final grade combines this score with the TA meeting and the report as stated in `docs/04_rules_and_grading.md`.

## Submission

Zip `controllers/student_mir/` and `controllers/student_arm/` (all files) together with `report.pdf`. Name the zip with
your group's student numbers. Upload it on Courseware before the deadline.

## TA meeting

Be ready to: explain your odometry equations; change a controller gain or speed limit live and predict the effect; show how
your controller behaves when the route order changes; explain how you used any AI assistance.
