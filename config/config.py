# config.py

SUMO_CFG_FILE = "./m.sumocfg"

CARLA_SERVERS = {
    0: {"host": "localhost", "port": 2000},
    1: {"host": "localhost", "port": 3000},
}

# Граница буферной зоны в координатах X
ZONE_X = [-8.5, 8.5]

# Какой blueprint использовать для машин
VEHICLE_BLUEPRINT = "vehicle.tesla.model3"