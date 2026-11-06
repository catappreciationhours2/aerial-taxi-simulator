# Swagmo benchmark

Fleet 40, 420 s simulated, seeds 1,2, wind 5 m/s (10 m) with 1.3x gusts, stadium TFR active, demand 2x.

## Ablation (means over seeds)

| preset | completed | loss of sep. | collisions | crashes | emergencies | delay ratio | energy/trip kWh | plan ms p95 |
|---|---|---|---|---|---|---|---|---|
| A: Independent A* | 18.0 | 0.0 | 0.0 | 0.0 | 0.0 | 1.22 | 17.0 | 29 |
| B: Space-time A* | 19.0 | 1.0 | 0.0 | 0.0 | 0.0 | 1.23 | 16.6 | 18 |
| C: Physics-aware | 14.5 | 0.5 | 0.0 | 0.0 | 0.5 | 1.34 | 19.1 | 79 |
| D: Full system | 15.5 | 0.5 | 0.0 | 0.0 | 0.5 | 1.29 | 17.2 | 134 |

## Scalability (preset D)

| fleet | airborne (peak) | completed | loss of sep. | crashes | wall s per sim-min | plan ms p95 |
|---|---|---|---|---|---|---|
| 20 | 12 | 5 | 1 | 0 | 0.2 | 78 |
| 100 | 40 | 8 | 0 | 0 | 1.8 | 167 |
