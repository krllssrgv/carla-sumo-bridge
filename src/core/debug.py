import traci
import sys

def main(cfg_path, step_length=0.05):
    traci.start([
        "sumo-gui",
        "-c", cfg_path,
        "--step-length", str(step_length),
        "--quit-on-end"
    ])
    print("[debug] SUMO started")

    try:
        while True:
            traci.simulationStep()
            ids = traci.vehicle.getIDList()
            if not ids:
                continue

            for vid in ids:
                x, y = traci.vehicle.getPosition(vid)
                angle = traci.vehicle.getAngle(vid)
                print(f"Vehicle {vid}: x={x:.2f}, y={y:.2f}, angle={angle:.1f}")
    except KeyboardInterrupt:
        print("[debug] stopped by user")
    finally:
        traci.close()

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python debug_sumo.py path/to/scenario.sumocfg")
        sys.exit(1)
    cfg_path = sys.argv[1]
    main(cfg_path)
