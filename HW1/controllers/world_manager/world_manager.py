"""world_manager - supervisor that runs the order-preparation task environment.

Reads config/world_config.json (what to spawn, orders, battery model) and config/layout.json
(geometry written by tools/generate_world.py). It:

  * spawns blocks on the input belts (input belt i always carries color shipment.input_colors[i]),
  * stops an input belt while a block is waiting at its arm (and resumes when the block is taken),
  * releases orders: each order gets a free output lane, an empty order tote is spawned on that
    output belt under the output arm, and the order is broadcast on Emitter channel 7,
  * detects when a tote holds the ordered blocks, then ships it down the output belt (the tote and
    its blocks are removed at the belt end) and frees the lane for the next order,
  * simulates a battery for every MiR100 (drains with time and movement, charges at the
    charging station) and publishes it on Emitter channel 8 and in each robot's customData,
  * records metrics (results/latest/events.csv, timeseries.csv, summary.json) and, when
    run.duration_s is set, ends the run, prints a report and draws plots (tools/report.py).

Config: config/world_config.json. It may name a preset ("preset": "a1") which is loaded from
config/presets/<name>.json and then overridden by the keys in world_config.json. The environment
variable FOR_CONFIG (path to a JSON file) replaces world_config.json - used by the teacher side
for hidden test configs.
Students normally only edit the JSON config.
"""
import bisect
import csv
import json
import math
import os
import random
import subprocess
import sys
from pathlib import Path

from controller import Supervisor

ROOT = Path(__file__).resolve().parents[2]
CONFIG_PATH = Path(os.environ.get("FOR_CONFIG") or ROOT / "config" / "world_config.json")
LAYOUT_PATH = ROOT / "config" / "layout.json"
RESULTS_DIR = Path(os.environ.get("FOR_RESULTS") or ROOT / "results" / "latest")
RESULTS_PATH = RESULTS_DIR / "summary.json"

EVAL_PERIOD = 0.2          # s between zone / order / battery evaluations
SPAWN_CLEARANCE = 0.12     # do not spawn a block if another one is closer than this to the spawn point
LANE_CLEARANCE = 0.8       # a shipped tote must be this far from the arm before the lane is reused
PUBLISH_PERIOD = 1.0       # s between battery broadcasts


def draw(value, rng):
    """value is a number or [min, max]."""
    if isinstance(value, (list, tuple)):
        return rng.uniform(value[0], value[1])
    return float(value)


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def deep_merge(base, over):
    out = dict(base)
    for k, v in over.items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = deep_merge(out[k], v)
        else:
            out[k] = v
    return out


def load_config():
    cfg = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
    name = cfg.get("preset")
    if name:
        base = json.loads((ROOT / "config" / "presets" / f"{name}.json").read_text(encoding="utf-8"))
        cfg = deep_merge(base, {k: v for k, v in cfg.items() if k != "preset"})
        cfg["preset"] = name
    return cfg


def percentile(vals, q):
    if not vals:
        return None
    s = sorted(vals)
    i = min(len(s) - 1, max(0, int(math.ceil(q * len(s))) - 1))
    return s[i]


class Block:
    def __init__(self, bid, color, node, conveyor, t):
        self.id = bid
        self.color = color
        self.node = node
        self.conveyor = conveyor
        self.t_spawn = t
        self.state = "alive"      # alive | carried | rejected | dropped | gone
        self.remove_at = None
        self.pos = None
        self.field_t = None
        self.t_arrive = None      # first time the block sat at the pick position
        self.t_pick = None        # time it was lifted off the belt


class Tote:
    def __init__(self, tid, lane, node, order, pos):
        self.id = tid
        self.lane = lane
        self.node = node
        self.order = order
        self.pos = pos
        self.state = "filling"    # filling | shipping | gone
        self.carried = []
        self.field_t = None


class Battery:
    def __init__(self, name, node, level, field_cd):
        self.name = name
        self.node = node
        self.level = level
        self.field_cd = field_cd
        self.prev = None
        self.prev_yaw = 0.0
        self.distance = 0.0
        self.charging = False
        self.dead = False
        self.dead_since = None
        self.low_warned = False
        self.min_level = level
        self.dead_time = 0.0
        self.charge_time = 0.0
        self.rescues = 0
        self.moving_time = 0.0
        self.charges = 0              # number of separate charging sessions
        self.speed = 0.0
        self.yaw = 0.0
        self.parked = {}              # station name -> time the robot has been inside tolerance
        self.armed = {}               # station name -> True when a new dock event may be logged
        self.docks = []               # [(t, station, pos_err, yaw_err)]

    def message(self, low):
        return {"type": "battery", "robot": self.name, "level": round(self.level, 1),
                "charging": self.charging, "dead": self.dead, "low": self.level < low}


class WorldManager:
    def __init__(self):
        self.robot = Supervisor()
        self.ts = int(self.robot.getBasicTimeStep())
        self.cfg = load_config()
        self.lay = json.loads(LAYOUT_PATH.read_text(encoding="utf-8"))
        self.run_cfg = self.cfg.get("run", {})
        self.events, self.series = [], []
        self.truth = {}                # robot name -> [(t, x, y, yaw)] every evaluation step
        self.mounts = {}               # robot name -> {lidar name: [x, y, yaw] in the robot frame}
        self.finished = False
        self.wait_samples, self.queue_delays, self.max_queue = [], [], 0
        self.next_sample = 0.0
        self.stations = self.build_stations()
        self.sh = self.cfg["shipment"]
        self.rng = random.Random(self.cfg.get("seed", 0))
        self.colors = self.cfg["colors"]
        self.n = len(self.lay["columns_x"])
        self.children = self.robot.getRoot().getField("children")
        self.order_emitter = self.robot.getDevice("order_emitter")
        self.battery_emitter = self.robot.getDevice("battery_emitter")

        self.blocks, self.totes, self.rules, self.batteries = [], [], [], []
        self.next_block_id = self.next_tote_id = 0
        self.stats = {"blocks_spawned": 0, "orders_released": 0, "orders_completed": 0, "orders_shipped": 0,
                      "orders_late": 0, "items_delivered": 0, "extra_items": 0,
                      "blocks_dropped": 0, "blocks_rejected": 0, "blocks_picked": 0}
        self.order_log = []
        self.last_lane = None
        self.order_queue, self.explicit_orders = [], []
        self.next_order_id = 0
        self.random_left = 0
        self.next_random_at = 0.0

        # belts: nominal speeds, current speeds, node handles, texture handles
        sp = self.cfg.get("conveyors", {})
        default = self.lay["belt"]["default_speed"]
        self.nominal = {"in": sp.get("input_speed", [default] * self.n), "out": sp.get("output_speed", [default] * self.n)}
        self.cur = {"in": [None] * self.n, "out": [None] * self.n}
        self.conv = {}

        self.next_eval = 0.0
        self.next_report = self.cfg.get("report_every_s", 15)
        self.next_publish = 0.0
        self.last_batt_t = 0.0

    # ------------------------------------------------------------------ setup
    def setup(self):
        for kind, prefix in (("in", "IN_CONV_"), ("out", "OUT_CONV_")):
            for k in range(self.n):
                node = self.robot.getFromDef(f"{prefix}{k}")
                self.conv[(kind, k)] = node
        for kind in ("in", "out"):
            for k in range(self.n):
                self.set_speed(kind, k, 0.0)

        self.input_colors = self.sh["input_colors"]
        for i, color in enumerate(self.input_colors):
            r, g, b = self.rgb(color)
            pos = self.lay["label_positions"]["input"][i]
            self.import_node(f'ColorLabel {{ translation {pos[0]} {pos[1]} {pos[2]} '
                             f'name "label_input_{i}" color {r} {g} {b} }}')
            s = dict(self.sh["spawn"])
            self.rules.append({
                "conveyor": i, "colors": [color], "interval": s.get("interval", 5), "count": s.get("count", -1),
                "max_on_belt": s.get("max_on_belt", 8), "next_t": s.get("start_delay", 1) + 0.5 * i, "spawned": 0})

        self.explicit_orders = sorted((dict(o) for o in self.sh.get("orders", [])), key=lambda o: o["at"])
        ro = self.sh.get("random_orders", {})
        self.random_left = int(ro.get("count", 0))
        self.next_random_at = float(ro.get("first_at", 60))
        for o in self.explicit_orders:
            self.check_items(o["items"])

        self.setup_batteries()
        self.setup_mounts()
        self.publish_batteries()
        print(f"[world_manager] orders: {len(self.explicit_orders)} listed, random={self.random_left}; "
              f"MiR batteries tracked: {len(self.batteries)}")

    def build_stations(self):
        st = self.lay.get("stations")
        if st:
            return st
        out = {}
        for i, p in enumerate(self.lay.get("mir_dock_poses", {}).get("input", [])):
            out[f"input_{i}"] = list(p)
        for i, p in enumerate(self.lay.get("mir_dock_poses", {}).get("output", [])):
            out[f"output_{i}"] = list(p)
        if "charging_station" in self.lay:
            out["charger"] = list(self.lay["charging_station"]["pose"])
        return out

    # ------------------------------------------------------------------ metrics log
    def log_event(self, t, kind, ident="", **detail):
        self.events.append((round(t, 2), kind, ident, json.dumps(detail, separators=(",", ":")) if detail else ""))

    def sample(self, t):
        row = {"t": round(t, 1), "orders_completed": self.stats["orders_completed"],
               "orders_shipped": self.stats["orders_shipped"], "orders_waiting": len(self.order_queue),
               "totes_open": sum(1 for x in self.totes if x.state == "filling"),
               "blocks_on_input": sum(1 for b in self.blocks if b.state == "alive" and b.pos and b.pos[2] > 0.3
                                      and b.pos[1] > 1.9),
               "dropped": self.stats["blocks_dropped"], "rejected": self.stats["blocks_rejected"],
               "extra_items": self.stats["extra_items"]}
        for i, bat in enumerate(self.batteries):
            p = bat.prev or [0, 0, 0]
            row[f"mir{i + 1}_x"] = round(p[0], 3)
            row[f"mir{i + 1}_y"] = round(p[1], 3)
            row[f"mir{i + 1}_yaw"] = round(bat.yaw, 3)
            row[f"mir{i + 1}_battery"] = round(bat.level, 2)
            row[f"mir{i + 1}_speed"] = round(bat.speed, 3)
        self.series.append(row)

    def check_parking(self, t, bat, p, yaw, dt):
        pc = self.cfg.get("parking", {})
        pos_tol, yaw_tol = pc.get("pos_tol_m", 0.15), pc.get("yaw_tol_rad", 0.2)
        hold, vmax = pc.get("hold_s", 1.0), pc.get("max_speed", 0.03)
        for name, (sx, sy, sh) in self.stations.items():
            d = math.hypot(p[0] - sx, p[1] - sy)
            ye = abs(wrap(yaw - sh))
            if d > 0.6:
                bat.armed[name] = True
            if d < pos_tol and ye < yaw_tol and bat.speed < vmax:
                bat.parked[name] = bat.parked.get(name, 0.0) + dt
                if bat.parked[name] >= hold and bat.armed.get(name, True):
                    bat.armed[name] = False
                    bat.docks.append((t, name, d, ye))
                    self.log_event(t, "dock", bat.name, station=name, pos_err_m=round(d, 3), yaw_err_rad=round(ye, 3))
            else:
                bat.parked[name] = 0.0

    def check_items(self, items):
        for c in items:
            if c not in self.input_colors:
                raise ValueError(f"order asks for '{c}' but no input conveyor provides it ({self.input_colors})")

    def rgb(self, name):
        if name not in self.colors:
            raise KeyError(f"color '{name}' is not defined in config 'colors'")
        return self.colors[name]

    def import_node(self, text):
        self.children.importMFNodeFromString(-1, text)
        return self.children.getMFNode(self.children.getCount() - 1)

    # ------------------------------------------------------------------ belts
    def set_speed(self, kind, k, v):
        if self.cur[kind][k] == v:
            return
        self.cur[kind][k] = v
        node = self.conv.get((kind, k))
        if node is not None:
            node.getField("customData").setSFString(str(v))

    def block_at_arm(self, k):
        """True while an input block waits at the stop position of belt k (within reach of the arm)."""
        x = self.lay["columns_x"][k]
        stop_y = self.lay["input_stop_y"]
        for b in self.blocks:
            if b.state == "alive" and b.pos and abs(b.pos[0] - x) < 0.12 and stop_y - 0.15 < b.pos[1] <= stop_y + 0.003 \
                    and 0.33 < b.pos[2] < 0.41:
                return True
        return False

    def control_belts(self):
        for k in range(self.n):
            self.set_speed("in", k, 0.0 if self.block_at_arm(k) else self.nominal["in"][k])
            shipping = any(t.state == "shipping" and t.lane == k for t in self.totes)
            self.set_speed("out", k, self.nominal["out"][k] if shipping else 0.0)

    def belt_velocity(self, p):
        """Current speed of the input belt under position p (None if p is not resting on one)."""
        lay = self.lay
        top = lay["belt"]["height"]
        if not (top - 0.01 < p[2] < top + 0.06):
            return None
        for k, x in enumerate(lay["columns_x"]):
            if abs(p[0] - x) > lay["belt"]["input_width"] / 2:
                continue
            if lay["input_stop_y"] < p[1] < lay["input_y"][0]:
                return self.cur["in"][k]
        return None

    def assist_belts(self, dt):
        """Kinematic carrier for blocks on the input belts (the Track node does not move them).
        A block moves at belt speed, stops behind the block ahead and at the end stop."""
        alive = [b for b in self.blocks if b.state == "alive"]
        for b in alive:
            p = b.pos
            v = self.belt_velocity(p)
            if not v:
                continue
            if b.node.getVelocity()[1] < -v * 0.9:
                continue                                   # the Track is carrying it
            ny = p[1] - v * dt
            if any(o is not b and abs(o.pos[0] - p[0]) < 0.07 and 0 < p[1] - o.pos[1] < 0.055 for o in alive):
                continue
            self.move_block(b, [p[0], ny, p[2]])

    def move_block(self, b, pos):
        if b.field_t is None:
            b.field_t = b.node.getField("translation")
        b.field_t.setSFVec3f(list(pos))
        b.pos = list(pos)

    # ------------------------------------------------------------------ blocks
    def spawn_block(self, conveyor, color, t):
        lay = self.lay
        x = lay["columns_x"][conveyor] + self.rng.uniform(-1, 1) * self.cfg.get("lateral_jitter", 0.0)
        y = lay["input_y"][0] - lay["spawn"]["y_offset_from_start"]
        z = lay["spawn"]["z"]
        for b in self.blocks:
            if b.state == "alive" and b.pos and math.hypot(b.pos[0] - x, b.pos[1] - y) < SPAWN_CLEARANCE:
                return False
        yaw = self.rng.uniform(0, math.pi) if self.cfg.get("random_yaw", True) else 0.0
        r, g, bl = self.rgb(color)
        self.next_block_id += 1
        bid = self.next_block_id
        node = self.import_node(
            f'ColoredBlock {{ translation {x:.4f} {y:.4f} {z:.4f} rotation 0 0 1 {yaw:.4f} '
            f'name "block_{bid}" color {r} {g} {bl} recognitionColors [{r} {g} {bl}] }}')
        blk = Block(bid, color, node, conveyor, t)
        blk.pos = [x, y, z]
        self.blocks.append(blk)
        self.stats["blocks_spawned"] += 1
        return True

    def belt_load(self, conveyor):
        return sum(1 for b in self.blocks if b.state == "alive" and b.pos and self.on_input_belt(conveyor, b.pos))

    def on_input_belt(self, i, p):
        x = self.lay["columns_x"][i]
        y0, y1 = self.lay["input_y"]
        return abs(p[0] - x) < 0.25 and y1 - 0.2 < p[1] < y0 and p[2] > 0.3

    def run_spawners(self, t):
        for r in self.rules:
            if t < r["next_t"] or (r["count"] >= 0 and r["spawned"] >= r["count"]):
                continue
            if self.belt_load(r["conveyor"]) >= r["max_on_belt"]:
                r["next_t"] = t + 0.5
                continue
            if self.spawn_block(r["conveyor"], self.rng.choice(r["colors"]), t):
                r["spawned"] += 1
                r["next_t"] = t + draw(r["interval"], self.rng)
            else:
                r["next_t"] = t + 0.5

    # ------------------------------------------------------------------ orders and totes
    def emit_order(self, msg):
        self.order_emitter.send(json.dumps(msg).encode("utf-8"))

    def new_order(self, spec, t):
        self.next_order_id += 1
        return {"id": self.next_order_id, "items": dict(spec["items"]), "lane": spec.get("lane"),
                "time_limit_s": spec.get("time_limit_s", self.sh.get("time_limit_s")), "t_release": t}

    def random_order(self):
        ro = self.sh["random_orders"]
        n = self.rng.randint(ro.get("min_items", 2), ro.get("max_items", 6))
        items = {}
        for _ in range(n):
            c = self.rng.choice(self.input_colors)
            items[c] = items.get(c, 0) + 1
        return {"items": items}

    def lane_free(self, k):
        place_y = self.lay["points"]["output_place"][k][1]
        for t in self.totes:
            if t.lane != k:
                continue
            if t.state == "filling" or (t.state == "shipping" and t.pos[1] > place_y - LANE_CLEARANCE):
                return False
        return True

    def pick_lane(self, order):
        lanes = self.sh.get("lanes", list(range(self.n)))
        if order["lane"] is not None:
            return order["lane"] if self.lane_free(order["lane"]) else None
        start = (self.last_lane + 1) % self.n if self.last_lane is not None else 0
        for k in sorted(lanes, key=lambda l: (l - start) % self.n):       # round-robin over free lanes
            if self.lane_free(k):
                return k
        return None

    def release_orders(self, t):
        while self.explicit_orders and t >= self.explicit_orders[0]["at"]:
            self.order_queue.append(self.new_order(self.explicit_orders.pop(0), t))
        if self.random_left != 0 and t >= self.next_random_at:
            self.order_queue.append(self.new_order(self.random_order(), t))
            if self.random_left > 0:
                self.random_left -= 1
            self.next_random_at = t + draw(self.sh["random_orders"].get("interval", 60), self.rng)
        for order in list(self.order_queue):
            lane = self.pick_lane(order)
            if lane is None:
                continue
            self.order_queue.remove(order)
            self.assign_order(order, lane, t)

    def assign_order(self, order, lane, t):
        order["lane"] = lane
        self.last_lane = lane
        order["t_assigned"] = t
        pos = self.lay["points"]["output_place"][lane]
        ts = self.lay["tote"]
        x, y, z = pos[0], pos[1], ts["spawn_z"]
        self.next_tote_id += 1
        node = self.import_node(
            f'OrderTote {{ translation {x:.4f} {y:.4f} {z:.4f} name "tote_order_{order["id"]}" '
            f'size {ts["size"][0]} {ts["size"][1]} {ts["size"][2]} wall {ts["wall"]} }}')
        order["t_start"] = t
        self.totes.append(Tote(self.next_tote_id, lane, node, order, [x, y, z]))
        self.stats["orders_released"] += 1
        self.queue_delays.append(t - order["t_release"])
        self.log_event(t, "order_start", order["id"], lane=lane, items=order["items"],
                       queue_delay_s=round(t - order["t_release"], 1))
        self.emit_order({"type": "new_order", "id": order["id"], "lane": lane, "items": order["items"],
                         "time_limit_s": order["time_limit_s"], "time": round(t, 2)})
        print(f"[world_manager] t={t:.1f}s order {order['id']} -> output lane {lane}: {order['items']}")

    def tote_contents(self, tote):
        ts = self.lay["tote"]
        ix = ts["size"][0] / 2 - ts["wall"]
        iy = ts["size"][1] / 2 - ts["wall"]
        z0, z1 = tote.pos[2], tote.pos[2] + ts["size"][2]
        return [b for b in self.blocks if b.state == "alive" and b.pos
                and abs(b.pos[0] - tote.pos[0]) < ix and abs(b.pos[1] - tote.pos[1]) < iy and z0 < b.pos[2] < z1]

    def check_totes(self, t):
        for tote in self.totes:
            if tote.state != "filling":
                continue
            inside = self.tote_contents(tote)
            have = {}
            for b in inside:
                have[b.color] = have.get(b.color, 0) + 1
            need = tote.order["items"]
            if not all(have.get(c, 0) >= n for c, n in need.items()):
                continue
            order = tote.order
            dur = t - order["t_assigned"]
            limit = order["time_limit_s"]
            late = limit is not None and dur > limit
            extra = {c: n - need.get(c, 0) for c, n in have.items() if n > need.get(c, 0)}
            n_extra = sum(extra.values())
            rec = {"id": order["id"], "lane": tote.lane, "items": need, "duration_s": round(dur, 1),
                   "late": late, "extra_items": extra, "t_complete": round(t, 1)}
            self.order_log.append(rec)
            self.stats["orders_completed"] += 1
            self.stats["items_delivered"] += sum(need.values())
            self.stats["extra_items"] += n_extra
            self.stats["orders_late"] += 1 if late else 0
            self.log_event(t, "order_complete", order["id"], lane=tote.lane, duration_s=round(dur, 1),
                           late=late, extra=extra)
            self.emit_order({"type": "order_complete", **rec, "time": round(t, 2)})
            print(f"[world_manager] t={t:.1f}s order {order['id']} COMPLETE in {dur:.1f}s "
                  f"{'(LATE) ' if late else ''}extra={extra}")
            for b in inside:
                b.state = "carried"
            tote.carried = inside
            tote.state = "shipping"

    def step_totes(self, dt):
        end_y = self.lay["tote"]["ship_end_y"]
        for tote in self.totes:
            if tote.state != "shipping":
                continue
            dy = self.nominal["out"][tote.lane] * dt
            tote.pos = [tote.pos[0], tote.pos[1] - dy, tote.pos[2]]
            if tote.field_t is None:
                tote.field_t = tote.node.getField("translation")
            tote.field_t.setSFVec3f(tote.pos)
            for b in tote.carried:
                p = b.node.getPosition()
                self.move_block(b, [p[0], p[1] - dy, p[2]])
            if tote.pos[1] <= end_y:
                for b in tote.carried:
                    b.node.remove()
                    b.state = "gone"
                tote.node.remove()
                tote.state = "gone"
                self.stats["orders_shipped"] += 1
                self.log_event(self.robot.getTime(), "order_shipped", tote.order["id"], lane=tote.lane)
                print(f"[world_manager] order {tote.order['id']} shipped from lane {tote.lane}")
        self.totes = [t for t in self.totes if t.state != "gone"]
        self.blocks = [b for b in self.blocks if b.state != "gone"]

    # ------------------------------------------------------------------ evaluation
    def in_reject_bin(self, p):
        bk = self.lay["basket"]
        return (abs(p[0] - bk["center"][0]) < bk["size"][0] / 2 - bk["wall"]
                and abs(p[1] - bk["center"][1]) < bk["size"][1] / 2 - bk["wall"]
                and bk["floor_z"] - 0.02 < p[2] < bk["floor_z"] + bk["size"][2] + 0.15)

    def track_pick(self, b, t):
        """Detect a block arriving at the pick position and later being lifted off the belt."""
        if b.t_pick is not None:
            return
        x = self.lay["columns_x"][b.conveyor]
        stop_y = self.lay["input_stop_y"]
        at_stop = (abs(b.pos[0] - x) < 0.12 and stop_y - 0.06 < b.pos[1] <= stop_y + 0.01 and 0.33 < b.pos[2] < 0.41)
        if b.t_arrive is None:
            if at_stop:
                b.t_arrive = t
                self.log_event(t, "block_at_pick", b.id, belt=b.conveyor, color=b.color)
        elif not at_stop:
            b.t_pick = t
            self.stats["blocks_picked"] += 1
            self.wait_samples.append(t - b.t_arrive)
            self.log_event(t, "block_picked", b.id, belt=b.conveyor, color=b.color, wait_s=round(t - b.t_arrive, 1))

    def evaluate(self, t):
        rm = self.cfg.get("cleanup", {})
        for b in self.blocks:
            if b.state in ("gone", "carried"):
                continue
            if b.remove_at is not None:
                if t >= b.remove_at:
                    b.node.remove()
                    b.state = "gone"
                continue
            if b.state != "alive":
                continue
            self.track_pick(b, t)
            if self.in_reject_bin(b.pos):
                b.state = "rejected"
                self.stats["blocks_rejected"] += 1
                self.log_event(t, "block_rejected", b.id, color=b.color)
                if rm.get("remove_rejected_after_s") is not None:
                    b.remove_at = t + rm["remove_rejected_after_s"]
            elif b.pos[2] < 0.15 and t - b.t_spawn > 1.0:
                b.state = "dropped"
                self.stats["blocks_dropped"] += 1
                self.log_event(t, "block_dropped", b.id, color=b.color, x=round(b.pos[0], 2), y=round(b.pos[1], 2))
                if rm.get("remove_dropped_after_s") is not None:
                    b.remove_at = t + rm["remove_dropped_after_s"]
        self.blocks = [b for b in self.blocks if b.state != "gone"]
        self.check_totes(t)

    # ------------------------------------------------------------------ batteries
    def setup_batteries(self):
        bc = self.cfg.get("battery", {})
        if not bc.get("enabled", True):
            return
        for name in self.lay.get("mir_defs", []):
            node = self.robot.getFromDef(name)
            if node is None:
                print(f"[world_manager] battery: robot DEF '{name}' not found, skipped")
                continue
            try:
                fcd = node.getField("customData")
            except Exception:
                fcd = None
            self.batteries.append(Battery(name, node, float(bc.get("start_level", 100)), fcd))
        if self.batteries and self.batteries[0].field_cd is None:
            print("[world_manager] MiR customData not available: use the battery Receiver (channel 8)")

    def setup_mounts(self):
        """Lidar mounting poses in the robot frame (read from the scene tree at the start pose)."""
        for bat in self.batteries:
            node = bat.node
            p, R = node.getPosition(), node.getOrientation()
            mounts = {}
            for slot, name in (("frontLidarSlot", "front_lidar"), ("backLidarSlot", "back_lidar")):
                try:
                    ln = node.getField(slot).getMFNode(0)
                except Exception:
                    ln = None
                if ln is None:
                    continue
                q, Rl = ln.getPosition(), ln.getOrientation()
                d = [q[i] - p[i] for i in range(3)]
                x = R[0] * d[0] + R[3] * d[1] + R[6] * d[2]          # R^T d
                y = R[1] * d[0] + R[4] * d[1] + R[7] * d[2]
                m00 = R[0] * Rl[0] + R[3] * Rl[3] + R[6] * Rl[6]     # (R^T Rl)[0][0]
                m10 = R[1] * Rl[0] + R[4] * Rl[3] + R[7] * Rl[6]     # (R^T Rl)[1][0]
                mounts[name] = [round(x, 4), round(y, 4), round(math.atan2(m10, m00), 4)]
            self.mounts[bat.name] = mounts
            if mounts:
                print(f"[world_manager] {bat.name} lidar mounts (x, y, yaw in robot frame): {mounts}")

    def update_batteries(self, t, dt):
        bc = self.cfg.get("battery", {})
        if not self.batteries or dt <= 0:
            return
        cx, cy, heading = self.lay["charging_station"]["pose"]
        radius = self.lay["charging_station"]["radius"]
        empty = bc.get("when_empty", {})
        for bat in self.batteries:
            p = bat.node.getPosition()
            R = bat.node.getOrientation()
            yaw = math.atan2(R[3], R[0])
            if bat.prev is None:
                bat.prev, bat.prev_yaw = p, yaw
                continue
            moved = math.hypot(p[0] - bat.prev[0], p[1] - bat.prev[1])
            if moved > 1.0:
                moved = 0.0                                   # teleport / reset, not driving
            dyaw = abs(wrap(yaw - bat.prev_yaw))
            bat.prev, bat.prev_yaw = p, yaw
            self.truth.setdefault(bat.name, []).append((t, p[0], p[1], yaw))
            bat.distance += moved
            speed = moved / dt
            bat.speed, bat.yaw = speed, yaw
            if speed > 0.05:
                bat.moving_time += dt
            was_charging = bat.charging
            bat.charging = math.hypot(p[0] - cx, p[1] - cy) < radius and speed < 0.05
            if bat.charging and not was_charging:
                bat.charges += 1
                self.log_event(t, "charge_start", bat.name, level=round(bat.level, 1))
            elif was_charging and not bat.charging:
                self.log_event(t, "charge_end", bat.name, level=round(bat.level, 1))
            self.check_parking(t, bat, p, yaw, dt)
            if bat.charging:
                bat.level = min(100.0, bat.level + bc.get("charge_per_s", 1.0) * dt)
                bat.charge_time += dt
            else:
                bat.level = max(0.0, bat.level - bc.get("idle_drain_per_s", 0.04) * dt
                                - bc.get("move_drain_per_m", 0.5) * moved
                                - bc.get("turn_drain_per_rad", 0.05) * dyaw)
            bat.min_level = min(bat.min_level, bat.level)
            low = bc.get("low_warning", 20)
            if bat.level < low and not bat.low_warned:
                bat.low_warned = True
                print(f"[world_manager] t={t:.0f}s {bat.name} battery LOW ({bat.level:.0f}%)")
            if bat.level > low + 5:
                bat.low_warned = False
            was_dead = bat.dead
            bat.dead = bat.level <= 0.0
            if bat.dead:
                bat.dead_time += dt
                if not was_dead:
                    bat.dead_since = t
                    self.log_event(t, "battery_empty", bat.name)
                    print(f"[world_manager] t={t:.0f}s {bat.name} battery EMPTY - robot immobilized")
                after = empty.get("rescue_after_s")
                if after is not None and t - bat.dead_since >= after:
                    self.rescue(bat, cx, cy, heading, empty.get("rescue_level", 10), p[2])
            elif was_dead:
                print(f"[world_manager] t={t:.0f}s {bat.name} battery recovered")

    def rescue(self, bat, cx, cy, heading, level, z):
        bat.node.getField("translation").setSFVec3f([cx, cy, z])
        bat.node.getField("rotation").setSFRotation([0, 0, 1, heading])
        bat.node.resetPhysics()
        bat.level = float(level)
        bat.dead = False
        bat.rescues += 1
        bat.prev = None
        bat.armed["charger"] = False
        self.log_event(self.robot.getTime(), "rescue", bat.name)
        print(f"[world_manager] {bat.name} towed to the charging station (rescue #{bat.rescues})")

    def enforce_dead(self):
        for bat in self.batteries:
            if bat.dead:
                bat.node.setVelocity([0.0] * 6)

    def open_orders(self):
        return [{"id": x.order["id"], "lane": x.lane, "items": x.order["items"],
                 "time_limit_s": x.order["time_limit_s"], "started": round(x.order.get("t_start", 0.0), 1)}
                for x in self.totes if x.state == "filling"]

    def arm_data_fields(self):
        if not hasattr(self, "_arm_cd"):
            self._arm_cd = []
            for side in ("IN", "OUT"):
                for i in range(len(self.lay["columns_x"])):
                    node = self.robot.getFromDef(f"UR3E_{side}_{i}")
                    f = node.getField("customData") if node else None
                    if f is not None:
                        self._arm_cd.append(f)
        return self._arm_cd

    def publish_batteries(self):
        low = self.cfg.get("battery", {}).get("low_warning", 20)
        orders = self.open_orders()
        for bat in self.batteries:
            msg = bat.message(low)
            if self.cfg.get("mission"):
                msg["mission"] = {k: v for k, v in self.cfg["mission"].items() if not k.startswith("_")}
            msg["orders"] = orders
            sensors = self.cfg.get("sensors")
            if sensors:
                msg["sensors"] = dict(sensors, seed=self.cfg.get("seed", 0))
                msg["lidar_mounts"] = self.mounts.get(bat.name, {})
            self.battery_emitter.send(json.dumps(msg).encode("utf-8"))
            if bat.field_cd is not None:
                bat.field_cd.setSFString(json.dumps(msg))
        arm_msg = json.dumps({"type": "orders", "time": round(self.robot.getTime(), 1), "open": orders})
        for f in self.arm_data_fields():
            f.setSFString(arm_msg)

    # ------------------------------------------------------------------ summary
    def localization(self, name):
        """Compare the pose the controller reported (results/estimate_<robot>.csv) with the true pose."""
        path = RESULTS_DIR / f"estimate_{name}.csv"
        truth = self.truth.get(name) or []
        if not path.exists() or not truth:
            return None, []
        times = [r[0] for r in truth]
        rows = []
        try:
            with open(path, newline="", encoding="utf-8") as f:
                for r in csv.DictReader(f):
                    te, xe, ye, the = float(r["t"]), float(r["x"]), float(r["y"]), float(r["theta"])
                    i = min(range(max(0, bisect.bisect_left(times, te) - 1), min(len(times), bisect.bisect_left(times, te) + 1)),
                            key=lambda k: abs(times[k] - te))
                    _, xt, yt, tht = truth[i]
                    rows.append((te, xt, yt, tht, xe, ye, the, math.hypot(xe - xt, ye - yt), abs(wrap(the - tht))))
        except (OSError, ValueError, KeyError):
            return None, []
        if not rows:
            return None, []
        pe = [r[7] for r in rows]
        ye = [r[8] for r in rows]
        loc = {"samples": len(rows), "mean_pos_err_m": round(sum(pe) / len(pe), 3), "max_pos_err_m": round(max(pe), 3),
               "final_pos_err_m": round(pe[-1], 3), "mean_yaw_err_rad": round(sum(ye) / len(ye), 3),
               "final_yaw_err_rad": round(ye[-1], 3)}
        return loc, rows

    def summary(self, t):
        done = self.order_log
        mins = max(t, 1e-6) / 60.0
        durs = [o["duration_s"] for o in done]
        st = self.stats
        late = st["orders_late"]
        orders = {
            "released": st["orders_released"], "completed": st["orders_completed"], "shipped": st["orders_shipped"],
            "late": late, "on_time_rate": round(1 - late / len(done), 3) if done else None,
            "completed_per_min": round(st["orders_completed"] / mins, 3),
            "avg_time_s": round(sum(durs) / len(durs), 1) if durs else None,
            "median_time_s": round(percentile(durs, 0.5), 1) if durs else None,
            "p90_time_s": round(percentile(durs, 0.9), 1) if durs else None,
            "avg_queue_delay_s": round(sum(self.queue_delays) / len(self.queue_delays), 1) if self.queue_delays else None,
            "waiting_now": len(self.order_queue), "open_now": sum(1 for x in self.totes if x.state == "filling"),
        }
        picked = st["blocks_picked"]
        blocks = {
            "spawned": st["blocks_spawned"], "picked": picked, "delivered_in_orders": st["items_delivered"],
            "extra_in_totes": st["extra_items"], "dropped": st["blocks_dropped"], "rejected": st["blocks_rejected"],
            "picked_per_min": round(picked / mins, 2),
            "avg_pick_wait_s": round(sum(self.wait_samples) / len(self.wait_samples), 2) if self.wait_samples else None,
            "efficiency": round(st["items_delivered"] / picked, 3) if picked else None,
        }
        robots = []
        for b in self.batteries:
            errs = [d[2] for d in b.docks]
            yerr = [d[3] for d in b.docks]
            per_station = {}
            for d in b.docks:
                per_station[d[1]] = per_station.get(d[1], 0) + 1
            robots.append({
                "robot": b.name, "distance_m": round(b.distance, 1),
                "moving_pct": round(100.0 * b.moving_time / max(t, 1e-6), 1),
                "battery_end": round(b.level, 1), "battery_min": round(b.min_level, 1),
                "dead_time_s": round(b.dead_time, 1), "rescues": b.rescues,
                "charge_sessions": b.charges, "charge_time_s": round(b.charge_time, 1),
                "docks": len(b.docks), "docks_per_station": per_station,
                "dock_mean_pos_err_m": round(sum(errs) / len(errs), 3) if errs else None,
                "dock_mean_yaw_err_rad": round(sum(yerr) / len(yerr), 3) if yerr else None,
                "localization": self.localization(b.name)[0]})
        run = {"sim_time_s": round(t, 1), "duration_s": self.run_cfg.get("duration_s"), "finished": self.finished,
               "seed": self.cfg.get("seed"), "preset": self.cfg.get("preset")}
        return {"run": run, "orders": orders, "blocks": blocks, "robots": robots, "order_list": done}

    def write_results(self, t):
        d = self.summary(t)
        try:
            RESULTS_DIR.mkdir(parents=True, exist_ok=True)
            RESULTS_PATH.write_text(json.dumps(d, indent=2), encoding="utf-8")
            with open(RESULTS_DIR / "events.csv", "w", newline="", encoding="utf-8") as f:
                w = csv.writer(f)
                w.writerow(["t", "type", "id", "detail_json"])
                w.writerows(self.events)
            for b in self.batteries:
                loc, rows = self.localization(b.name)
                if rows:
                    with open(RESULTS_DIR / f"localization_{b.name}.csv", "w", newline="", encoding="utf-8") as f:
                        w = csv.writer(f)
                        w.writerow(["t", "true_x", "true_y", "true_theta", "est_x", "est_y", "est_theta", "pos_err_m", "yaw_err_rad"])
                        w.writerows([[round(v, 4) for v in r] for r in rows])
            if self.series:
                keys = list(self.series[-1].keys())
                with open(RESULTS_DIR / "timeseries.csv", "w", newline="", encoding="utf-8") as f:
                    w = csv.DictWriter(f, fieldnames=keys, extrasaction="ignore")
                    w.writeheader()
                    w.writerows(self.series)
        except OSError as e:
            print(f"[world_manager] could not write results: {e}")
        return d

    def report(self, t):
        d = self.write_results(t)
        o, bl = d["orders"], d["blocks"]
        print(f"[world_manager] t={t:.0f}s orders done/shipped={o['completed']}/{o['shipped']} "
              f"({o['completed_per_min']}/min) late={o['late']} extra={bl['extra_in_totes']} "
              f"dropped={bl['dropped']} rejected={bl['rejected']} | " +
              ", ".join(f"{b['robot']} {b['battery_end']}%" for b in d["robots"]))

    def final_report(self, t):
        self.finished = True
        d = self.write_results(t)
        o, bl, run = d["orders"], d["blocks"], d["run"]
        line = "=" * 64
        print("\n" + line)
        print(f" RUN FINISHED  t={t:.0f}s  preset={run['preset']}  seed={run['seed']}")
        print(line)
        print(f" Orders   released {o['released']}  completed {o['completed']}  shipped {o['shipped']}  "
              f"late {o['late']}  still open {o['open_now'] + o['waiting_now']}")
        print(f"          throughput {o['completed_per_min']} orders/min   avg time {o['avg_time_s']} s   "
              f"p90 {o['p90_time_s']} s   queue delay {o['avg_queue_delay_s']} s")
        print(f" Blocks   picked {bl['picked']}  delivered {bl['delivered_in_orders']}  extra {bl['extra_in_totes']}  "
              f"dropped {bl['dropped']}  rejected {bl['rejected']}  efficiency {bl['efficiency']}")
        print(f"          picked/min {bl['picked_per_min']}   avg wait at the arm {bl['avg_pick_wait_s']} s")
        for r in d["robots"]:
            print(f" {r['robot']:<9}distance {r['distance_m']} m  moving {r['moving_pct']}%  battery end/min "
                  f"{r['battery_end']}/{r['battery_min']}%  charges {r['charge_sessions']}  dead {r['dead_time_s']} s  "
                  f"rescues {r['rescues']}  docks {r['docks']}")
        for r in d["robots"]:
            loc = r.get("localization")
            if loc:
                print(f" {r['robot']:<9}localization: mean error {loc['mean_pos_err_m']} m, final {loc['final_pos_err_m']} m, "
                      f"max {loc['max_pos_err_m']} m; heading mean {loc['mean_yaw_err_rad']} rad ({loc['samples']} samples)")
        print(f" Files    {RESULTS_DIR}")
        print(line + "\n")
        if self.run_cfg.get("plots", True):
            script = ROOT / "tools" / "report.py"
            if script.exists():
                try:
                    r = subprocess.run([sys.executable, str(script), str(RESULTS_DIR)], capture_output=True,
                                       text=True, timeout=120)
                    if r.returncode == 0:
                        print(f"[world_manager] plots: {(r.stdout or '').strip()}")
                    elif "matplotlib" in (r.stderr or ""):
                        print("[world_manager] plots skipped: matplotlib is not installed for this Python "
                              f"({sys.executable}). Install it ('pip install matplotlib') or set Webots "
                              "Preferences > Python command to an environment that has it. The CSV/JSON results were saved.")
                    else:
                        print(f"[world_manager] plots failed: {(r.stderr or '').strip()[-300:]}")
                except Exception as e:  # matplotlib missing etc.
                    print(f"[world_manager] plots skipped ({e}); run: python tools/report.py results/latest")

    # ------------------------------------------------------------------ main loop
    def run(self):
        self.setup()
        assist = self.cfg.get("conveyor_assist", True)
        dt = self.ts / 1000.0
        duration = self.run_cfg.get("duration_s")
        while self.robot.step(self.ts) != -1:
            t = self.robot.getTime()
            for b in self.blocks:
                if b.state == "alive":
                    b.pos = b.node.getPosition()
            if assist:
                self.assist_belts(dt)
            self.control_belts()
            self.run_spawners(t)
            self.release_orders(t)
            self.step_totes(dt)
            self.enforce_dead()
            if t >= self.next_eval:
                self.evaluate(t)
                self.update_batteries(t, t - self.last_batt_t)
                self.last_batt_t = t
                self.next_eval = t + EVAL_PERIOD
                self.max_queue = max(self.max_queue, len(self.order_queue))
            if t >= self.next_sample:
                self.sample(t)
                self.next_sample = t + 1.0
            if t >= self.next_publish:
                self.publish_batteries()
                self.next_publish = t + PUBLISH_PERIOD
            if t >= self.next_report:
                self.report(t)
                self.next_report = t + self.cfg.get("report_every_s", 15)
            if duration is not None and t >= duration:
                self.final_report(t)
                action = self.run_cfg.get("on_finish", "pause")
                if action == "quit":
                    self.robot.simulationQuit(0)
                    return
                self.robot.simulationSetMode(Supervisor.SIMULATION_MODE_PAUSE)
                # keep stepping so the supervisor stays alive after Play is pressed again
                duration = None


if __name__ == "__main__":
    WorldManager().run()
