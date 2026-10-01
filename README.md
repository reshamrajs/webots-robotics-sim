# Webots robotics simulations

Two Webots R2025a worlds (https://cyberbotics.com), Python controllers.

| World | Robot | What it does |
|---|---|---|
| `worlds/swarm_arena.wbt` | 5 × e-puck + supervisor | Braitenberg obstacle avoidance among boxes and barrels; supervisor logs 120 s of poses to `analysis/trajectories.csv`, then pauses |
| `worlds/pioneer_lidar.wbt` | Pioneer 3-DX + SICK LMS 291 | Follow-the-gap lidar navigation through an 8 × 8 m cluttered room; prints nearest-obstacle / heading every ~2 s |

Run: open Webots → File → Open World → pick a `.wbt` → press Play.
Controllers use `/opt/homebrew/bin/python3.11` (set in each `controllers/*/runtime.ini`); change that path to your own Python 3, or delete the `runtime.ini` files to use Webots' default.

For the plot: `python3 -m venv .venv && .venv/bin/pip install matplotlib numpy`. After the swarm run: `.venv/bin/python analysis/plot_trajectories.py` → `analysis/trajectories.png`.

## Results from the first runs

- **Swarm (120 s):** each e-puck travelled 6.1–6.8 m with no robot stuck; together they visited 86% of the arena's 10 cm cells (most unvisited cells sit under obstacles).
- **Pioneer (134 s):** no stall, top speed 0.6 m/s, closest approach 0.26 m. Its readings repeat about every 23 s, so it is likely circling one loop rather than covering the room.

![Swarm trajectories and coverage](analysis/trajectories.png)
