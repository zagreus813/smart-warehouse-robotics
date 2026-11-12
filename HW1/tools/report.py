"""Draw plots for a finished run.   python tools/report.py [results/latest]

Reads summary.json, events.csv and timeseries.csv written by the world_manager and writes
report.png (overview) into the same folder. Needs matplotlib (pip install matplotlib)."""
import csv
import json
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

folder = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).resolve().parents[1] / "results" / "latest"
summary = json.loads((folder / "summary.json").read_text(encoding="utf-8"))


def read_csv(name):
    p = folder / name
    if not p.exists():
        return []
    with open(p, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


ts = read_csv("timeseries.csv")
ev = read_csv("events.csv")
col = lambda k: [float(r[k]) for r in ts if r.get(k) not in (None, "")]
t = col("t")

fig, ax = plt.subplots(2, 2, figsize=(13, 8))
fig.suptitle(f"Run report - {summary['run']['sim_time_s']:.0f} s, preset {summary['run']['preset']}, seed {summary['run']['seed']}")

# 1 orders over time
a = ax[0][0]
if t:
    a.step(t, col("orders_completed"), where="post", label="completed")
    a.step(t, col("orders_shipped"), where="post", label="shipped")
    a.step(t, col("orders_waiting"), where="post", label="waiting for a lane", alpha=0.7)
a.set_title("Orders"); a.set_xlabel("time [s]"); a.legend(); a.grid(alpha=0.3)

# 2 order durations
a = ax[0][1]
done = summary["order_list"]
if done:
    a.bar([str(o["id"]) for o in done], [o["duration_s"] for o in done],
          color=["#d95f02" if o["late"] else "#1b9e77" for o in done])
a.set_title("Time per order [s] (orange = late)"); a.set_xlabel("order id"); a.grid(alpha=0.3, axis="y")

# 3 battery
a = ax[1][0]
for k in sorted({c[:-8] for c in (ts[0] if ts else {}) if c.endswith("_battery")}):
    a.plot(t, col(k + "_battery"), label=k)
for r in ev:
    if r["type"] in ("charge_start",):
        a.axvline(float(r["t"]), color="grey", ls=":", alpha=0.6)
    if r["type"] == "battery_empty":
        a.axvline(float(r["t"]), color="red", ls="--", alpha=0.8)
a.set_ylim(0, 105); a.set_title("Battery [%] (dotted = charging started, red = empty)")
a.set_xlabel("time [s]"); a.legend(); a.grid(alpha=0.3)

# 4 robot paths
a = ax[1][1]
for k in sorted({c[:-2] for c in (ts[0] if ts else {}) if c.startswith("mir") and c.endswith("_x")}):
    a.plot(col(k + "_x"), col(k + "_y"), lw=1, label=k)
try:
    lay = json.loads((folder.parents[1] / "config" / "layout.json").read_text(encoding="utf-8"))
    for name, (sx, sy, _h) in {**{f"in{i}": p for i, p in enumerate(lay["mir_dock_poses"]["input"])},
                               **{f"out{i}": p for i, p in enumerate(lay["mir_dock_poses"]["output"])},
                               "chg": lay["charging_station"]["pose"]}.items():
        a.plot(sx, sy, "kx"); a.annotate(name, (sx, sy), xytext=(4, 4), textcoords="offset points", fontsize=8)
    for x in lay["columns_x"]:
        a.plot([x, x], [lay["input_y"][1], lay["input_y"][0]], color="#bbb", lw=6, solid_capstyle="butt")
        a.plot([x, x], [lay["output_y"][1], lay["output_y"][0]], color="#bbb", lw=6, solid_capstyle="butt")
    a.set_xlim(-7, 7); a.set_ylim(-6.5, 6.5)
except Exception:
    pass
a.set_aspect("equal"); a.set_title("MiR paths [m]"); a.set_xlabel("x"); a.set_ylabel("y"); a.legend(); a.grid(alpha=0.3)

fig.tight_layout()
out = folder / "report.png"
fig.savefig(out, dpi=110)
print(f"saved {out}")
