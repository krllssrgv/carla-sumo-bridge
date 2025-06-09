# sumo/sumo_manager.py

import os
import sys
import traci
import traci.constants as tc
import subprocess

from config import SUMO_CFG_FILE

class SumoManager:
    def init(self):
        self.sumo_process = None
        self.started = False

    def start(self):
        sumo_binary = os.path.join(os.environ["SUMO_HOME"], "bin", "sumo")
        self.sumo_process = subprocess.Popen(
            [sumo_binary, "-c", SUMO_CFG_FILE, "--start", "--remote-port", "8813"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        traci.init(8813)
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