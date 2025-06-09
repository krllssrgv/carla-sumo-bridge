# core/zone_manager.py

from src.config import ZONE_X

class ZoneManager:
    def init(self):
        self.x_min, self.x_max = ZONE_X

    def in_zone(self, x):
        return self.x_min <= x <= self.x_max

    def get_server_for_x(self, x):
        if x < self.x_min:
            return 0  # левая CARLA
        elif x > self.x_max:
            return 1  # правая CARLA
        else:
            return None  # внутри буферной зоны