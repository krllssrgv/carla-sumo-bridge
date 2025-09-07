import json
from dataclasses import dataclass, asdict
from typing import List

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
    axis: str
    start: float
    end: float
    z_offset: float

class Config:
    def log_config(self):
        import pprint
        pprint.pprint({
            "sumo": asdict(self.sumo),
            "carla": [asdict(w) for w in self.carla_worlds],
            "zone": asdict(self.zone)
        })

    def __init__(self, path: str):
        with open(path, "r") as f:
            data = json.load(f)

        # SUMO
        if "sumo" not in data:
            raise ValueError("Config must contain 'sumo'")

        self.sumo = SumoConfig(
            config_file=data["sumo"]["config_file"],
            step_length=float(data["sumo"].get("step_length", 0.05))
        )

        # CARLA
        if "carla" not in data or "worlds" not in data["carla"]:
            raise ValueError("Config must contain 'carla.worlds'")

        self.carla_worlds: List[CarlaWorldConfig] = []
        for w in data["carla"]["worlds"]:
            self.carla_worlds.append(
                CarlaWorldConfig(
                    name=w["name"],
                    host=w["host"],
                    port=int(w["port"])
                )
            )

        if len(self.carla_worlds) != 2:
            raise ValueError("Config must contain 2 CARLA worlds")

        # Zone
        if "zone" not in data:
            raise ValueError("Config must contain 'zone'")

        z = data["zone"]
        if z["axis"] not in ["x"]:
            raise ValueError("zone.axis must be 'x'")

        self.zone = ZoneConfig(
            axis=z["axis"],
            start=float(z["start"]),
            end=float(z["end"]),
            z_offset=float(z.get("z_offset", 0.1))
        )


cfg = Config("config.json")
cfg.log_config()