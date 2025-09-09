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

        # Remember original settings to restore on exit
        self._orig_settings_a = self.world_a.get_settings()
        self._orig_settings_b = self.world_b.get_settings()

        # Enable synchronous mode w/ fixed delta
        self._apply_sync(self.world_a, self.cfg.sumo.step_length)
        self._apply_sync(self.world_b, self.cfg.sumo.step_length)

        # Blueprints (cache)
        self.bps_a = self.world_a.get_blueprint_library().filter("vehicle.*")
        self.bps_b = self.world_b.get_blueprint_library().filter("vehicle.*")
        if not self.bps_a or not self.bps_b:
            raise RuntimeError("No vehicle blueprints found in CARLA")

        # veh_id -> {"A": actor, "B": actor}
        self.actors: Dict[str, Dict[str, Optional[carla.Actor]]] = {}

        # Track presence to remove disappeared vehicles
        self._last_ids: set[str] = set()

    # ----- lifecycle -----

    def start_sumo(self):
        # Start SUMO-GUI using sumocfg; we still push step via TraCI
        traci.start(["sumo-gui", "-c", self.cfg.sumo.config_file, "--quit-on-end"])
        # Optional: enforce step length at runtime (often already in .sumocfg)
        traci.simulation.setStepLength(self.cfg.sumo.step_length)

    def close(self):
        # Destroy remaining actors
        for vid, slots in list(self.actors.items()):
            for key in ("A", "B"):
                if slots.get(key):
                    try:
                        slots[key].destroy()
                    except Exception:
                        pass
            self.actors.pop(vid, None)
        # Restore CARLA settings
        try:
            self.world_a.apply_settings(self._orig_settings_a)
        except Exception:
            pass
        try:
            self.world_b.apply_settings(self._orig_settings_b)
        except Exception:
            pass
        # Close SUMO
        try:
            traci.close(False)
        except Exception:
            pass

    # ----- core loop -----

    def run(self):
        print("[bridge] Starting SUMO-GUI…")
        self.start_sumo()
        print("[bridge] Connected to CARLA A/B and SUMO. Entering loop.")

        while True:
            traci.simulationStep()
            ids = set(traci.vehicle.getIDList())

            # Remove vehicles that disappeared from SUMO
            for gone in list(self._last_ids - ids):
                self._destroy_vehicle_everywhere(gone)

            # Update/Spawn for existing
            for vid in ids:
                x, y = traci.vehicle.getPosition(vid)     # SUMO coords
                angle = traci.vehicle.getAngle(vid)        # degrees
                yaw = float(angle)                         # assuming aligned; adjust if needed
                z = self.cfg.zone.z_offset

                worlds_needed = self._assign_worlds(x, y)

                # ensure registry entry
                state = self.actors.get(vid, {"A": None, "B": None})

                # Spawn where missing
                if "A" in worlds_needed and state["A"] is None:
                    state["A"] = self._spawn(self.world_a, self.bps_a, vid, x, y, z, yaw)
                if "B" in worlds_needed and state["B"] is None:
                    state["B"] = self._spawn(self.world_b, self.bps_b, vid, x, y, z, yaw)

                # Remove where not needed
                if "A" not in worlds_needed and state["A"]:
                    self._safe_destroy(state["A"]); state["A"] = None
                if "B" not in worlds_needed and state["B"]:
                    self._safe_destroy(state["B"]); state["B"] = None

                # Update transforms
                tf = carla.Transform(carla.Location(x=x, y=y, z=z), carla.Rotation(yaw=yaw))
                if state["A"]: state["A"].set_transform(tf)
                if state["B"]: state["B"].set_transform(tf)

                self.actors[vid] = state

            # Tick both CARLA worlds
            self.world_a.tick()
            self.world_b.tick()

            self._last_ids = ids

    # ----- helpers -----

    def _apply_sync(self, world: carla.World, dt: float):
        s = world.get_settings()
        s.synchronous_mode = True
        s.fixed_delta_seconds = dt
        world.apply_settings(s)

    def _assign_worlds(self, x: float, y: float):
        if self.cfg.zone.axis == "x":
            if x < self.cfg.zone.start:
                return {"A"}
            elif x > self.cfg.zone.end:
                return {"B"}
            else:
                return {"A", "B"}
        else:
            if y < self.cfg.zone.start:
                return {"A"}
            elif y > self.cfg.zone.end:
                return {"B"}
            else:
                return {"A", "B"}

    def _spawn(self, world: carla.World, bps, vid: str, x: float, y: float, z: float, yaw: float) -> Optional[carla.Actor]:
        idx = abs(hash(vid)) % len(bps)  # deterministic choice by id
        bp = bps[idx]
        # set role_name to veh_id for debugging
        if bp.has_attribute("role_name"):
            bp.set_attribute("role_name", vid)
        tf = carla.Transform(carla.Location(x=x, y=y, z=z), carla.Rotation(yaw=yaw))
        try:
            actor = world.try_spawn_actor(bp, tf)
            if actor is None:
                # fallback: blocking spawn (rarely needed)
                actor = world.spawn_actor(bp, tf)
            return actor
        except Exception as e:
            print(f"[spawn] failed for {vid}: {e}")
            return None

    def _safe_destroy(self, actor: carla.Actor):
        try:
            actor.destroy()
        except Exception:
            pass

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
