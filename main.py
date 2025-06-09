# main.py

import time

from sumo.sumo_manager import SumoManager
from carla.carla_manager import CarlaManager
from core.zone_manager import ZoneManager
from config import CARLA_SERVERS

def main():
    sumo = SumoManager()
    sumo.start()

    carlas = {
        0: CarlaManager(**CARLA_SERVERS[0]),
        1: CarlaManager(**CARLA_SERVERS[1]),
    }

    zone = ZoneManager()

    try:
        while True:
            sumo.step()
            vehicle_ids = sumo.get_vehicle_ids()

            for veh_id in vehicle_ids:
                x, y = sumo.get_vehicle_position(veh_id)
                server = zone.get_server_for_x(x)
                if server is not None:
                    carlas[server].spawn_or_update_vehicle(veh_id, x, y)

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("Shutting down.")
    finally:
        sumo.close()
        for cm in carlas.values():
            for vid in list(cm.vehicles.keys()):
                cm.destroy_vehicle(vid)

if name == "main":
    main()