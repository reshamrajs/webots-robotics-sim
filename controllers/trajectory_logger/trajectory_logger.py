"""Supervisor that records every e-puck's pose to analysis/trajectories.csv.

Writes one row per robot every 0.32 s of simulated time and stops after
SIM_SECONDS, so a run produces the same amount of data each time.
"""
import csv
import math
import os
from controller import Supervisor

SIM_SECONDS = 120.0
EVERY = 10  # steps between samples

sup = Supervisor()
dt = int(sup.getBasicTimeStep())

robots = []
i = 1
while (node := sup.getFromDef(f"EPUCK{i}")) is not None:
    robots.append((f"e-puck{i}", node))
    i += 1

out = os.path.join(os.path.dirname(__file__), "..", "..", "analysis", "trajectories.csv")
out = os.path.abspath(out)
with open(out, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["t", "robot", "x", "y", "yaw"])
    step = 0
    while sup.step(dt) != -1:
        step += 1
        t = sup.getTime()
        if step % EVERY == 0:
            for name, node in robots:
                x, y, _ = node.getPosition()
                R = node.getOrientation()
                w.writerow([f"{t:.3f}", name, f"{x:.4f}", f"{y:.4f}", f"{math.atan2(R[3], R[0]):.4f}"])
        if t >= SIM_SECONDS:
            f.flush()
            print(f"Logged {len(robots)} robots for {SIM_SECONDS:.0f} s -> {out}")
            sup.simulationSetMode(Supervisor.SIMULATION_MODE_PAUSE)
            break
