# bridge.py
import json
import pprint
import signal
import sys
import time
from dataclasses import dataclass, asdict
from typing import Dict, List, Optional

# --- External deps: carla, traci (SUMO) ---
import carla
import traci


# -------------------- Config --------------------

@dataclass
class SumoConfig:
    config_file: str
    step_length: float

@dataclass
class CarlaWorldConfig:
    name: str
    host: str
    port: int

@dataclass
class ZoneConfig:
    axis: str   # "x" or "y" (первая версия: "x")
    start: float
    end: float
    z_offset: float

class Config:
    def __init__(self, path: str):
        with open(path, "r") as f:
            data = json.load(f)

        self.sumo = SumoConfig(
            config_file=data["sumo"]["config_file"],
            step_length=float(data["sumo"].get("step_length", 0.05)),
        )

        self.carla_worlds: List[CarlaWorldConfig] = [
            CarlaWorldConfig(name=w["name"], host=w["host"], port=int(w["port"]))
            for w in data["carla"]["worlds"]
        ]
        if len(self.carla_worlds) != 2:
            raise ValueError("Config must define exactly 2 CARLA worlds")

        z = data["zone"]
        if z["axis"] not in ("x", "y"):
            raise ValueError("zone.axis must be 'x' or 'y'")
        self.zone = ZoneConfig(
            axis=z["axis"],
            start=float(z["start"]),
            end=float(z["end"]),
            z_offset=float(z.get("z_offset", 0.1)),
        )

    def to_dict(self):
        return {
            "sumo": asdict(self.sumo),
            "carla": [asdict(w) for w in self.carla_worlds],
            "zone": asdict(self.zone),
        }

    def pretty_print(self):
        pprint.pprint(self.to_dict())


# -------------------- Bridge --------------------

# -------------------- Bridge --------------------

class Bridge:
    def __init__(self, cfg: Config):
        self.cfg = cfg

        # Connect CARLA worlds
        self.client_a = carla.Client(self.cfg.carla_worlds[0].host, self.cfg.carla_worlds[0].port)
        self.client_b = carla.Client(self.cfg.carla_worlds[1].host, self.cfg.carla_worlds[1].port)
        self.client_a.set_timeout(5.0)
        self.client_b.set_timeout(5.0)

        self.world_a = self.client_a.get_world()
        self.world_b = self.client_b.get_world()

        self._orig_settings_a = self.world_a.get_settings()
        self._orig_settings_b = self.world_b.get_settings()

        self._apply_sync(self.world_a, self.cfg.sumo.step_length)
        self._apply_sync(self.world_b, self.cfg.sumo.step_length)

        self.bps_a = self.world_a.get_blueprint_library().filter("vehicle.*")
        self.bps_b = self.world_b.get_blueprint_library().filter("vehicle.*")
        if not self.bps_a or not self.bps_b:
            raise RuntimeError("No vehicle blueprints found in CARLA")

        self.actors: Dict[str, Dict[str, Optional[carla.Actor]]] = {}
        self._last_ids: set[str] = set()

        # offset между сетями SUMO и CARLA (инициализируем позже)
        self.offset = (0.0, 0.0)

    def start_sumo(self):
        traci.start([
            "sumo-gui",
            "-c", self.cfg.sumo.config_file,
            "--step-length", str(self.cfg.sumo.step_length),
            "--quit-on-end"
        ])
        # получаем offset сети SUMO (смещение net.xml)
        try:
            net_offset = traci.simulation.getNetOffset()  # (x,y)
            self.offset = (net_offset[0], net_offset[1])
            print(f"[bridge] SUMO net offset: {self.offset}")
        except Exception:
            print("[bridge] Warning: could not get net offset from SUMO, using (0,0)")
            self.offset = (0.0, 0.0)

    # ---- новая функция ----
    def sumo_to_carla_transform(self, xs, ys, angle, z):
        # yaw: SUMO (0=east, CCW) -> CARLA (0=+X, CW)
        yaw = -angle + 90

        # учёт offset
        x_off = xs - self.offset[0]
        y_off = ys - self.offset[1]

        # инверсия Y для CARLA
        x_carla = x_off
        y_carla = -y_off

        return carla.Transform(
            carla.Location(x=x_carla, y=y_carla, z=z),
            carla.Rotation(yaw=yaw)
        )

    def run(self):
        print("[bridge] Starting SUMO-GUI…")
        self.start_sumo()
        print("[bridge] Connected to CARLA A/B and SUMO. Entering loop.")

        while True:
            traci.simulationStep()
            ids = set(traci.vehicle.getIDList())

            # Remove vehicles gone
            for gone in list(self._last_ids - ids):
                self._destroy_vehicle_everywhere(gone)

            for vid in ids:
                xs, ys = traci.vehicle.getPosition(vid)
                angle = traci.vehicle.getAngle(vid)
                z = self.cfg.zone.z_offset

                tf = self.sumo_to_carla_transform(xs, ys, angle, z)
                worlds_needed = self._assign_worlds(tf.location.x, tf.location.y)

                state = self.actors.get(vid, {"A": None, "B": None})

                # spawn if missing
                if "A" in worlds_needed and state["A"] is None:
                    state["A"] = self._spawn(self.world_a, self.bps_a, vid, tf)
                if "B" in worlds_needed and state["B"] is None:
                    state["B"] = self._spawn(self.world_b, self.bps_b, vid, tf)

                # remove if not needed
                if "A" not in worlds_needed and state["A"]:
                    self._safe_destroy(state["A"]); state["A"] = None
                if "B" not in worlds_needed and state["B"]:
                    self._safe_destroy(state["B"]); state["B"] = None

                # update transforms
                if state["A"]: state["A"].set_transform(tf)
                if state["B"]: state["B"].set_transform(tf)

                self.actors[vid] = state

            self.world_a.tick()
            self.world_b.tick()
            self._last_ids = ids

    def _spawn(self, world: carla.World, bps, vid: str, tf: carla.Transform) -> Optional[carla.Actor]:
        idx = abs(hash(vid)) % len(bps)
        bp = bps[idx]
        if bp.has_attribute("role_name"):
            bp.set_attribute("role_name", vid)
        try:
            actor = world.try_spawn_actor(bp, tf)
            if actor is None:
                tf.location.z += 0.5
                actor = world.try_spawn_actor(bp, tf)
            return actor
        except Exception as e:
            print(f"[spawn] failed for {vid}: {e}")
            return None

# -------------------- entrypoint --------------------

def main():
    if len(sys.argv) != 2:
        print("Usage: python bridge.py path/to/config.json")
        sys.exit(1)

    cfg = Config(sys.argv[1])
    print("[config]")
    cfg.pretty_print()

    bridge = Bridge(cfg)

    def _graceful_exit(sig, frame):
        print("\n[bridge] Shutting down…")
        bridge.close()
        sys.exit(0)

    signal.signal(signal.SIGINT, _graceful_exit)
    signal.signal(signal.SIGTERM, _graceful_exit)

    try:
        bridge.run()
    except Exception as e:
        print(f"[bridge] Error: {e}")
        bridge.close()
        sys.exit(2)

if __name__ == "__main__":
    main()
