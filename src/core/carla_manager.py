# carla/carla_manager.py

import carla
import random

from src.config import VEHICLE_BLUEPRINT

class CarlaManager:
    def init(self, host, port):
        self.client = carla.Client(host, port)
        self.client.set_timeout(5.0)
        self.world = self.client.get_world()
        self.blueprint_lib = self.world.get_blueprint_library()
        self.vehicles = {}

    def spawn_or_update_vehicle(self, veh_id, x, y):
        location = carla.Location(x=x, y=y, z=0.5)
        transform = carla.Transform(location)

        if veh_id in self.vehicles:
            self.vehicles[veh_id].set_transform(transform)
        else:
            bp = self.blueprint_lib.find(VEHICLE_BLUEPRINT)
            spawn = self.world.try_spawn_actor(bp, transform)
            if spawn:
                self.vehicles[veh_id] = spawn

    def destroy_vehicle(self, veh_id):
        if veh_id in self.vehicles:
            self.vehicles[veh_id].destroy()
            del self.vehicles[veh_id]