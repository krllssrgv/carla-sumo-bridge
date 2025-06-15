# sumo/sumo_manager.py

import traci
import traci.constants as tc
import subprocess

from src.config import SUMO_CFG_FILE

class SumoManager:
    def init(self):
        self.sumo_process = None
        self.started = False

    def start(self):
        sumo_binary = "sumo"
        traci.start([sumo_binary, "-c", SUMO_CFG_FILE, "--start"])
        self.started = True

    def step(self):
        if self.started:
            traci.simulationStep()

    def get_vehicle_ids(self):
        return traci.vehicle.getIDList()

    def get_vehicle_position(self, veh_id):
        return traci.vehicle.getPosition(veh_id)

    def close(self):
        if self.started:
            traci.close()
        if self.sumo_process:
            self.sumo_process.terminate()