import traci
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass

@dataclass
class Boundary:
    minX: float
    minY: float
    maxX: float
    maxY: float

def create_coordinate_transformer(conv: Boundary, orig: Boundary):
    def transform(convX: float, convY: float):
        x = orig.minX + ((convX - conv.minX) / (conv.maxX - conv.minX)) * (orig.maxX - orig.minX)
        y = orig.minY + ((convY - conv.minY) / (conv.maxY - conv.minY)) * (orig.maxY - orig.minY)
        return x, y
    return transform

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

def main(cfg_path, net_path, step_length=0.05):
    conv, orig = read_boundaries_from_net(net_path)
    transform = create_coordinate_transformer(conv, orig)

    traci.start([
        "sumo-gui",
        "-c", cfg_path,
        "--step-length", str(step_length),
        "--quit-on-end"
    ])
    print("[debug] SUMO started")
    print(f"[debug] convBoundary={conv}")
    print(f"[debug] origBoundary={orig}")

    try:
        while True:
            traci.simulationStep()
            ids = traci.vehicle.getIDList()
            for vid in ids:
                xs, ys = traci.vehicle.getPosition(vid)
                xo, yo = transform(xs, ys)
                print(f"{vid}: SUMO=({xs:.2f},{ys:.2f})  XODR=({xo:.2f},{yo:.2f})")
    except KeyboardInterrupt:
        print("[debug] stopped by user")
    finally:
        traci.close()

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python debug_sumo_transform.py path/to/scenario.sumocfg path/to/net.net.xml")
        sys.exit(1)
    cfg_path = sys.argv[1]
    net_path = sys.argv[2]
    main(cfg_path, net_path)
