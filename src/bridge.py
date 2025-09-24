import json
import pprint
import signal
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass, asdict
from typing import Dict, List, Optional
import carla
import traci


# -------------------- Config --------------------
# Блок нужен для проверки файла конфигурации, чтобы в нем были все значения

@dataclass
class SumoConfig:
    config_file: str
    net_file: str
    step_length: float

@dataclass
class CarlaWorldConfig:
    name: str
    host: str
    port: int

@dataclass
class ZoneConfig:
    axis: str
    start: float
    end: float
    z_offset: float

@dataclass
class Boundary:
    minX: float
    minY: float
    maxX: float
    maxY: float

class Config:
    def __init__(self, path: str):
        with open(path, "r") as f:
            data = json.load(f)

        # SUMO
        self.sumo = SumoConfig(
            config_file=data["sumo"]["config_file"],
            net_file=data["sumo"]["net_file"],
            step_length=float(data["sumo"].get("step_length", 0.05))
        )

        # CARLA
        self.carla_worlds: List[CarlaWorldConfig] = [
            CarlaWorldConfig(name=w["name"], host=w["host"], port=int(w["port"]))
            for w in data["carla"]["worlds"]
        ]
        if len(self.carla_worlds) != 2:
            raise ValueError("Config must define exactly 2 CARLA worlds")

        # Zone
        z = data["zone"]
        if z["axis"] not in ("x", "y"):
            raise ValueError("zone.axis must be 'x' or 'y'")
        self.zone = ZoneConfig(
            axis=z["axis"],
            start=float(z["start"]),
            end=float(z["end"]),
            z_offset=float(z.get("z_offset", 0.1))
        )

    # Logs
    def to_dict(self):
        return {
            "sumo": asdict(self.sumo),
            "carla": [asdict(w) for w in self.carla_worlds],
            "zone": asdict(self.zone)
        }

    def print(self):
        pprint.pprint(self.to_dict())


# -------------------- Coord transform --------------------
# Блок нужен для трансформации координат net.xml и xodr
# В файле net.xml есть тег location, в котором указаны границы карты и их смещение
# Bз этого можно в реальном времени перевести координаты машины в SUMO в координаты Carla карты 

def read_boundaries_from_net(net_file: str):
    tree = ET.parse(net_file)
    root = tree.getroot()
    location = root.find("location")
    if location is None:
        raise ValueError("No <location> found in net.xml")

    conv_vals = list(map(float, location.get("convBoundary").split(",")))
    orig_vals = list(map(float, location.get("origBoundary").split(",")))

    conv = Boundary(*conv_vals)
    orig = Boundary(*orig_vals)
    return conv, orig

def create_coordinate_transformer(conv: Boundary, orig: Boundary):
    def transform(convX: float, convY: float):
        x = orig.minX + ((convX - conv.minX) / (conv.maxX - conv.minX)) * (orig.maxX - orig.minX)
        y = orig.minY + ((convY - conv.minY) / (conv.maxY - conv.minY)) * (orig.maxY - orig.minY)
        return x, y
    return transform


# -------------------- Bridge --------------------

class Bridge:
    def __init__(self, cfg: Config):
        self.cfg = cfg

        # Connect CARLA
        self.client_a = carla.Client(self.cfg.carla_worlds[0].host, self.cfg.carla_worlds[0].port)
        self.client_b = carla.Client(self.cfg.carla_worlds[1].host, self.cfg.carla_worlds[1].port)
        self.client_a.set_timeout(5.0)
        self.client_b.set_timeout(5.0)

        self.world_a = self.client_a.get_world()
        self.world_b = self.client_b.get_world()

        # Save settings
        self._orig_settings_a = self.world_a.get_settings()
        self._orig_settings_b = self.world_b.get_settings()

        # Sync
        self._apply_sync(self.world_a, self.cfg.sumo.step_length)
        self._apply_sync(self.world_b, self.cfg.sumo.step_length)

        # Blueprints
        self.bps_a = self.world_a.get_blueprint_library().filter("vehicle.*")
        self.bps_b = self.world_b.get_blueprint_library().filter("vehicle.*")

        # Actors registry
        self.actors: Dict[str, Dict[str, Optional[carla.Actor]]] = {}
        self._last_ids: set[str] = set()

        # Prepare coordinate transformer
        conv, orig = read_boundaries_from_net(self.cfg.sumo.net_file)
        self.transform = create_coordinate_transformer(conv, orig)

    # Настраивает CARLA в синхронный режим с фиксированным шагом симуляции
    def _apply_sync(self, world: carla.World, dt: float):
        s = world.get_settings()
        s.synchronous_mode = True
        s.fixed_delta_seconds = dt
        world.apply_settings(s)

    # Удаление актера без ошибки
    def _safe_destroy(self, actor: carla.Actor):
        try:
            actor.destroy()
        except Exception:
            pass

    # Удаляет актёра с указанным ID из обоих миров
    def _destroy_vehicle_everywhere(self, vid: str):
        if vid in self.actors:
            for key in ("A", "B"):
                if self.actors[vid].get(key):
                    self._safe_destroy(self.actors[vid][key])
            self.actors.pop(vid, None)

    # Запуск SUMO
    def start_sumo(self):
        traci.start([
            "sumo-gui",
            "-c", self.cfg.sumo.config_file,
            "--step-length", str(self.cfg.sumo.step_length),
            "--quit-on-end"
        ])
        print("[bridge] SUMO started")

    # net.xml в xodr
    def sumo_to_carla_transform(self, xs, ys, angle, z):
        x, y = self.transform(xs, ys)

        y = -y
        yaw = angle - 90
        return carla.Transform(
            carla.Location(x=x, y=y, z=z),
            carla.Rotation(pitch=0.0, yaw=yaw, roll=0.0)
        )

    # Создание актера
    def _spawn(self, world: carla.World, bps, vid: str, tf: carla.Transform) -> Optional[carla.Actor]:
        idx = abs(hash(vid)) % len(bps)
        bp = bps[idx]
        if bp.has_attribute("role_name"):
            bp.set_attribute("role_name", vid)

        try:
            actor = world.try_spawn_actor(bp, tf)
            if actor is None:
                # Это возможно костыль - увеление координаты по Z, потому что были проблемы при спавне в какой-то момент
                tf.location.z += 0.5
                actor = world.try_spawn_actor(bp, tf)
            return actor
        except Exception as e:
            print(f"[spawn] failed for {vid}: {e}")
            return None

    # определение целевого мира
    def _assign_worlds(self, x: float, y: float):
        if self.cfg.zone.axis == "x":
            if x < self.cfg.zone.start:
                return {"A"}
            elif x > self.cfg.zone.end:
                return {"B"}
            else:
                return {"A", "B"}
        
        # На данный момент заглушка, так как еще не внедрил обработку по оси Y
        else:
            if y < self.cfg.zone.start:
                return {"A"}
            elif y > self.cfg.zone.end:
                return {"B"}
            else:
                return {"A", "B"}

    # Обработка симуляции
    def run(self):
        self.start_sumo()

        while True:
            # Шаг симуляции
            traci.simulationStep()
            ids = set(traci.vehicle.getIDList())

            # Если машина исчезла из SUMO, то удаляем и из Carla
            for gone in list(self._last_ids - ids):
                self._destroy_vehicle_everywhere(gone)

            # Обновление машин и их координат
            for vid in ids:
                xs, ys = traci.vehicle.getPosition(vid)
                angle = traci.vehicle.getAngle(vid)
                # Снова костыль с осью Z, нужно потестить без него
                z = self.cfg.zone.z_offset

                tf = self.sumo_to_carla_transform(xs, ys, angle, z)
                worlds_needed = self._assign_worlds(tf.location.x, tf.location.y)

                state = self.actors.get(vid, {"A": None, "B": None})

                # Если машина должна быть в зоне - спавним
                if "A" in worlds_needed and state["A"] is None:
                    state["A"] = self._spawn(self.world_a, self.bps_a, vid, tf)
                if "B" in worlds_needed and state["B"] is None:
                    state["B"] = self._spawn(self.world_b, self.bps_b, vid, tf)

                # Если машина не должна быть в зоне - удаляем
                if "A" not in worlds_needed and state["A"]:
                    self._safe_destroy(state["A"]); state["A"] = None
                if "B" not in worlds_needed and state["B"]:
                    self._safe_destroy(state["B"]); state["B"] = None

                if state["A"]: state["A"].set_transform(tf)
                if state["B"]: state["B"].set_transform(tf)

                self.actors[vid] = state

            self.world_a.tick()
            self.world_b.tick()
            self._last_ids = ids

    def close(self):
        for vid in list(self.actors.keys()):
            self._destroy_vehicle_everywhere(vid)
        try:
            self.world_a.apply_settings(self._orig_settings_a)
        except Exception:
            pass
        try:
            self.world_b.apply_settings(self._orig_settings_b)
        except Exception:
            pass
        try:
            traci.close(False)
        except Exception:
            pass


# -------------------- APP --------------------

def main():
    if len(sys.argv) != 2:
        sys.exit(1)

    cfg = Config(sys.argv[1])
    # cfg.print()

    bridge = Bridge(cfg)

    def _safe_exit(sig, frame):
        bridge.close()
        sys.exit(0)

    signal.signal(signal.SIGINT, _safe_exit)
    signal.signal(signal.SIGTERM, _safe_exit)

    try:
        bridge.run()
    except Exception as e:
        print(f"[bridge] Error: {e}")
        bridge.close()
        sys.exit(2)

if __name__ == "__main__":
    main()
