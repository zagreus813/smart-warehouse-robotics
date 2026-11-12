# 04 - Rules and grading

## Working rules

* **Use of AI assistants is allowed.** You are responsible for every line you submit. In the meeting with the teaching
  assistant (TA) you must be able to explain your code, change it live, and answer questions about the underlying
  theory. A solution you cannot explain gets no credit for the parts you cannot explain.
* You may use libraries when they do the job properly (numpy, OpenCV, ...). Where an assignment says **"implement this
  yourself"** it names the part and gives the reason; for that part use no library that solves it for you.
* Edit only `controllers/student_*/` (and `config/world_config.json` for your own tests). Do not change the world, protos,
  manager, presets or layout. Do not try to influence the simulation from a controller.
* Groups: up to 3 students (working alone or in pairs is fine). Every member must be able to explain all of the code.
* Questions: use the course Telegram group; do not post your solution code.

## What you submit

For each assignment: a zip with your `controllers/student_mir/` and `controllers/student_arm/` folders (every file in
them), plus a short PDF report. The assignment text lists what the report must contain. The deadline is in each assignment text. Upload on Courseware.

## How your work is checked

1. **Re-run.** The TA runs your controllers in a clean copy of the official project. The world, manager and presets are
   the official ones; only your `controllers/student_*` are used. The run uses a **hidden configuration**: same task, same
   rules, different seed and different orders (and mission details where the assignment says so).
2. **Integrity check.** Modified project files, scene-tree access, file or network access from a controller and similar
   things are flagged and reviewed by hand.
3. **Automatic score** from the metrics below, with the thresholds printed in each assignment.
4. **Meeting** with a TA: you explain and modify your code; this decides the remaining points.

Split of each assignment's points: automatic score 50 %, meeting and code understanding 35 %, report 15 %.
When an assignment has written exercises, they are part of the report.

## Metrics

All are computed by the world manager and written to `results/latest/summary.json`. You see exactly what the grader sees
for your own run; only the configuration of the graded run differs.

**Orders**

| Metric | Meaning |
|---|---|
| `orders.completed`, `shipped`, `released`, `late` | counts |
| `orders.completed_per_min` | completed orders divided by run time (the main throughput measure) |
| `orders.avg_time_s`, `median_time_s`, `p90_time_s` | time from tote appearance to completion |
| `orders.on_time_rate` | share of completed orders that were not late |
| `orders.avg_queue_delay_s` | how long orders waited for a free lane |

**Blocks**

| Metric | Meaning |
|---|---|
| `blocks.picked`, `picked_per_min` | blocks lifted off an input belt |
| `blocks.delivered_in_orders` | blocks counted in completed orders |
| `blocks.extra_in_totes` | blocks in a tote beyond (or not in) the order at completion |
| `blocks.dropped`, `blocks.rejected` | blocks that fell to the floor / ended in the reject bin |
| `blocks.efficiency` | delivered / picked: how much of what you pick is useful |
| `blocks.avg_pick_wait_s` | how long a block waits at the belt end before being picked |

**Robots (per MiR100)**

| Metric | Meaning |
|---|---|
| `distance_m`, `moving_pct` | distance driven, share of time moving |
| `battery_end`, `battery_min`, `charge_sessions`, `charge_time_s` | battery use |
| `dead_time_s`, `rescues` | time with an empty battery, tow events |
| `docks`, `docks_per_station`, `dock_mean_pos_err_m`, `dock_mean_yaw_err_rad` | parking events and precision |
| `localization.mean_pos_err_m`, `final_pos_err_m`, `max_pos_err_m`, `mean_yaw_err_rad` | (assignment 2) how far the pose you report with `PoseLog` is from the true pose |

Rates and counts are compared against thresholds from a reference solution. A robust solution that finishes slightly
fewer orders scores better than a fast one that drops blocks or lets a battery die.

## Reproducibility

The simulation is deterministic for a given seed and configuration. The graded run uses a seed you have not seen, so
tune against several of your own seeds (`"seed"` in `world_config.json`) before submitting.
