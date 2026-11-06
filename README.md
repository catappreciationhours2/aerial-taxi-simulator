# Swagmo — routing 500 aerial taxis over Pittsburgh

A simulator and planner for a fleet of autonomous aerial taxis. It handles restricted airspace, wind and gusts near terrain, other air traffic, and injected failures.

# Available at https://catappreciationhours2.github.io/aerial-taxi-simulator/


## Run it Locally (3 steps, Node 18+ only, no `npm install`)

```bash
# Clone the repository 
cd swagmo
npm start            # then open http://localhost:8080
```

Other commands:

```bash
npm test             # about 160 tests, about 3 minutes
npm run bench:quick  # preset A-D ablation + scalability sweep, about 1 minute -> results/bench-quick.md
npm run bench        # full benchmark
```

The UI is static files served by `scripts/serve.js`. For GitHub Pages, publish the repo root: `index.html` redirects to `web/`.

## Guided tutorial

The first time the page opens, a live tutorial walks you through pausing, views, selecting a taxi, injecting a failure, toggling features, weather, demand and stress mode. **Skip tutorial** (or Esc) is always available. The **? Tutorial** button at the top replays it. Add `?notutorial` to the URL to suppress it.

## Stress mode

**💥 Stress: launch all** (left panel) restarts the simulation with the whole fleet size from the slider (up to 500) and makes every idle, charged taxi request a ride immediately and keep requesting. Click it again to end stress mode. Each vertiport has one take-off/landing column, so stress mode saturates the vertiports. The optional **parallel take-offs (unsafe)** box allows 3 simultaneous operations per vertiport; that roughly doubled airborne taxis in the same test but produced dozens of loss-of-separation events and collisions, which is the point of showing it. 500 airborne at once is not reachable in this world - mostly because I got lazy and dint want to make more take off points.

## Algorithm (the bonus answer)

**Problem.** 500 taxis, 4 flight levels, a 7.2 × 4.8 km Pittsburgh-inspired world with terrain, buildings, a permanent airport restriction and a temporary stadium TFR. Wind is stronger and gustier near terrain, and external traffic crosses the area.

**Planner: Wind-aware SIPP with a risk-weighted cost and a relaxed reverse-Dijkstra heuristic.**
1. **Graph.** A 3-D lattice (100 m cells × 4 levels). Blocked nodes come from buildings and terrain (with 35 m vertical clearance), restricted zones, and closed vertiports.
2. **Edge cost** `J = time + energy + risk + noise`. Ground speed uses the forecast wind (plus a gust uncertainty margin), power uses a momentum-theory model, risk is higher near obstacles and in forecast-gust hazards, and noise penalises residential cells.
3. **Heuristic.** A reverse Dijkstra over a relaxed problem (best-case tailwind, no reservations) gives an admissible `h`. It is computed once per destination and cached. Weighted A* (ε = 1.5) trades about 5% optimality for large speed-ups. A test verifies `h(start) ≤ true cost` at ε = 1.
4. **Traffic coordination.** SIPP finds a path through safe intervals of a space-time reservation table (padded in time). Planning is prioritised, and an LNS repair re-plans the aircraft that conflict. Each vertiport has one FATO (takeoff/landing column), so operations there are serialised.
5. **Contingency.** A reverse-time field to the nearest emergency site is kept for every cell. The viability monitor checks that each aircraft can still reach a site with energy reserve. If not (or if the MPC cannot track), runtime assurance (RTA) switches to an emergency landing along the field.
6. **Tactical layer.** The raw path is smoothed (7 s box filter) into a reference. An MPC tracks it. A CBF safety filter (QP solved with Dykstra projection) guarantees clearance to buildings, terrain, other taxis and external traffic.
7. **Estimation.** Each aircraft has a navigation filter with GNSS fault detection (innovation test + INS disagreement). The fleet shares wind measurements through a Kalman wind map, so gusts seen by one taxi are known by the others.

**Departures from the original design doc:** one FATO per vertiport (found necessary for takeoff-column safety), a smoothed reference instead of flying the raw SIPP path (the raw path has velocity jumps), a collision-cone gate on the CBF (to stop it firing on routine passes), and 12 injectable faults with detection/response/resolution tracking.

## Fault injection

| Fault | What happens |
|---|---|
| GPS bias | 60 m position error, detected, GNSS isolated, aircraft diverts |
| Comm loss | Plan frozen, then restored |
| Propulsion | Thrust loss, emergency landing |
| Battery | Capacity and drain penalty, diversion |
| Gust | Local gust cell, shared as a hazard |
| Route deviation | Aircraft pushed off path, replans |
| Sensor failure | Noisier navigation filter |
| Traffic intruder | Helicopter/GA track on a collision course |
| Vertiport closed | Site blocked 150 s, traffic reassigned |
| Planner timeout | Falls back to validated plans |
| Stale weather | Forecast margins widened |
| Storm | True wind raised |

Turn "faultResponse" off in the toggles to see the ablation (the same fault with no handling).

## Tests and results

About 160 tests cover geometry, wind, power model, dynamics, estimator, MPC, CBF, SIPP, reservations, heuristic admissibility, vertiports, engine determinism, each fault type, loaders, renderer, server, tutorial steps and stress mode. All pass on Node 18/20/22.

Latest quick benchmark (`results/bench-quick.md`, 40 aircraft, 420 s): zero crashes and zero collisions for all presets. The baselines (A, B) also show no losses because 40 aircraft is sparse. The scalability sweep shows the **known limits**: at 100 aircraft there were 2 loss-of-separation events and 2 crashes in the quick run.

## Assumptions and known limitations

- **Real data.** Only buildings are optional real data: `npm run fetch-osm` downloads OpenStreetMap footprints for the 7.2 x 4.8 km box into `data/buildings.geojson` (footprint, plus `height` or `building:levels` x 3.2 m, else 12 m). Terrain, wind and the vertiports stay synthetic; there is no elevation data in it. Live wind is separate: the "load live Pittsburgh wind" button fetches the current wind speed, direction and gusts at one point from Open-Meteo each time you click it, and nothing is stored.
- All vehicle and world parameters are assumed, not certified. The city is a synthetic Pittsburgh terrain using real data from osm already pre-loaded. 
- The static CBF barrier is conservative near tall obstacles in tailwind. At fleet sizes of about 100+ there are rare crashes.
- Single-FATO vertiports limit throughput.
- Planning dominates CPU (200–550 ms p95). Under heavy congestion some requests are rejected as exhausted.
- 500 aircraft run slower than real time on a laptop. Can crash laptop because of heavy graphics. use the fleet slider for demos and testing the simulator .

## Sources

Code adapted from these works carries an "adapted from" comment at the relevant place:

- Phillips & Likhachev, [SIPP: Safe Interval Path Planning for Dynamic Environments](https://www.cs.cmu.edu/~maxim/files/sipp_icra11.pdf), ICRA 2011 (`src/planning/sipp.js`)
- Silver, [Cooperative Pathfinding](https://www.davidsilver.org/wp-content/uploads/2020/03/coop-path-AIWisdom.pdf), AIIDE 2005 (reservation table, true-distance heuristic)
- Li et al., [Anytime Multi-Agent Path Finding via Large Neighborhood Search](https://www.ijcai.org/proceedings/2021/0568.pdf), IJCAI 2021 (LNS repair)
- Bennewitz, Burgard & Thrun, [Finding and optimizing solvable priority schemes for decoupled path planning](https://doi.org/10.1016/S0921-8890(01)00113-0), RAS 2002 (randomised priorities)
- Leishman, *Principles of Helicopter Aerodynamics*, 2nd ed., 2006 (power model)
- Wang, Ames & Egerstedt, [Safety Barrier Certificates for Collisions-Free Multirobot Systems](https://doi.org/10.1109/TRO.2017.2659727), IEEE T-RO 2017
- Ames et al., [Control Barrier Function Based Quadratic Programs for Safety Critical Systems](https://doi.org/10.1109/TAC.2016.2638961), IEEE TAC 2017
- Boyle & Dykstra, "A method for finding projections onto the intersection of convex sets", 1986 (QP projection)
- Mellinger & Kumar, [Minimum snap trajectory generation](https://doi.org/10.1109/ICRA.2011.5980409), ICRA 2011 (reference smoothing, as background)
- Acklam's rational approximation of the inverse normal CDF
- Live data: [Open-Meteo](https://open-meteo.com), [OpenSky Network](https://opensky-network.org), [OpenStreetMap Overpass](https://overpass-api.de)

