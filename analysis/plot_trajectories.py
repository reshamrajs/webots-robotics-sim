"""Plot e-puck paths and arena coverage from trajectories.csv."""
import csv
import os
from collections import defaultdict

import matplotlib.pyplot as plt
import numpy as np

here = os.path.dirname(os.path.abspath(__file__))
paths = defaultdict(list)
with open(os.path.join(here, "trajectories.csv")) as f:
    for row in csv.DictReader(f):
        paths[row["robot"]].append((float(row["x"]), float(row["y"])))

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 5))
for name, p in sorted(paths.items()):
    p = np.array(p)
    ax1.plot(p[:, 0], p[:, 1], lw=1, label=name)
    ax1.plot(*p[0], "o", ms=4, color=ax1.lines[-1].get_color())
ax1.set(xlim=(-0.8, 0.8), ylim=(-0.8, 0.8), aspect="equal", title="Paths (dot = start)")
ax1.legend(fontsize=8)

allp = np.vstack([np.array(p) for p in paths.values()])
H, _, _ = np.histogram2d(allp[:, 0], allp[:, 1], bins=16, range=[[-0.8, 0.8], [-0.8, 0.8]])
ax2.imshow(H.T > 0, origin="lower", extent=(-0.8, 0.8, -0.8, 0.8), cmap="Greens")
ax2.set(title=f"Cells visited: {(H > 0).mean():.0%} of 10 cm grid", aspect="equal")
fig.tight_layout()
fig.savefig(os.path.join(here, "trajectories.png"), dpi=150)
print("saved analysis/trajectories.png")
