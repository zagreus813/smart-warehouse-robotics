# 01 - Setup guide

You will work on one Webots project for the whole semester. This page gets it running on your computer.

## 1. Install the tools

| Tool | Version | Notes |
|---|---|---|
| Webots | **R2025a** (use exactly this release) | Windows or Linux. Download from <https://cyberbotics.com>. Other releases may not open the world. |
| Python | 3.10 - 3.12 | Webots runs your controllers with it. In Webots: *Tools -> Preferences -> General -> Python command*, enter `python` (Windows) or `python3` (Linux), or the full path of your interpreter. |
| matplotlib (optional) | any recent | Only for the end-of-run plots. Install it into **the Python that Webots uses** (the one set in *Preferences -> Python command*; a virtual environment works if you point the command at its `python.exe`). Without it you still get the printed report and the CSV files; you can also draw the plots later with `python tools/report.py results/latest` from any environment that has matplotlib. |
| An editor | any | Webots has a built-in text editor; VS Code or PyCharm work well. If you use an external editor, set *Preferences -> General -> Text editor* so Webots reloads the file. |

You need an internet connection the first time you open the world: Webots downloads the MiR100 and UR3e robot models
(and the gripper) from GitHub and caches them. Later runs work offline.

## 2. Unpack the project

Unzip `FoR_Project_aN.zip` somewhere with a short path that contains no spaces or non-ASCII characters, for example
`C:\FoR\` or `~/FoR/`. You get a folder `FoR_Warehouse`:

```
FoR_Warehouse/
  worlds/warehouse.wbt        the world (do not edit)
  protos/                     conveyor, tote, block, basket models (do not edit)
  controllers/
    student_mir/              YOUR controller for the MiR100 robot(s)        <- you edit this
    student_arm/              YOUR controller for the six UR3e arms          <- you edit this (from assignment 3)
    world_manager/            the supervisor that runs the task (do not edit)
    conveyor_belt/            belt motor driver (do not edit)
  config/
    world_config.json         which preset to use + your overrides           <- you may edit this
    presets/                  one preset per assignment (do not edit)
    layout.json               coordinates of stations, belts, arms (read-only reference)
  tools/report.py             draws the end-of-run plots
  results/latest/             written by every run (summary, events, timeseries, report.png)
  docs/                       this documentation
```

**Only edit** `controllers/student_*/` and (for your own experiments) `config/world_config.json`. Everything else is
checked by the grader; the official copy is used when your work is re-run (see `04_rules_and_grading.md`).

## 3. Open the world and check it

1. Start Webots, *File -> Open World...*, choose `FoR_Warehouse/worlds/warehouse.wbt`.
2. Wait for the model download to finish (the console at the bottom shows progress).
3. Press **Play**.

You should see:

* a warehouse with three belts on the north side (input) and three on the south side (output), a charging pad on the
  west side, a reject bin, six arms with grippers, and one MiR100 robot near the center;
* the console prints `[world_manager] orders: ...` and, every 15 simulated seconds, a status line;
* each `student_*` controller prints the list of devices of its robot (the starter code does this);
* after the preset's run time (for assignment 1: 360 s) the simulation pauses and the console prints a **RUN FINISHED**
  report; the plots are saved in `results/latest/report.png`.

If something in this list does not happen, ask in the course group and include the first red/orange lines of the Webots
console (text, not a photo).

## 4. Everyday workflow

* Edit the controller, then press **Reset** (the circular arrow) so Webots restarts it, then Play. Saving the file is
  not enough.
* *Fast* mode (>>) runs faster than real time; turn rendering off with the *Disable rendering* toolbar button when you
  do not need to look.
* Changing `world_config.json` also needs a Reset.
* Do not save the world (Ctrl+S) after dragging objects around. Your changes to `warehouse.wbt` are ignored by the grader
  and may break your own setup.
* `results/latest/` is overwritten by every run. Copy it elsewhere if you want to keep a run for your report.
