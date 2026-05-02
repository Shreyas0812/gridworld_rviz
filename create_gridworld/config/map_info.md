# Map Configuration Reference

## Maps

| Map | Grid | Agents | Induct | Eject | Charging | Idle-wait |
|-----|------|--------|--------|-------|----------|-----------|
| `gridworld_warehouse_small` | 30x30 | 6 | 8 | 38 | 6 | 6 |
| `gridworld_warehouse_large` | 60x60 | 18 | 12 | 80 | 12 | 16 |
| `gridworld_kiva` | 36x36 | 20 | 5 | 10 | 12 | 10 |
| `gridworld_kiva_large` | 72x72 | 80 | 5 | 10 | 12 | 10 |
| `gridworld_crossdock` | 44x28 | 12 | 6 | 10 | 8 | 12 |
| `gridworld_shelf_aisle` | 161x63 | 200 | 20 | 19 | 14 | 21 |

---

## Verified Conditions (all maps pass as of 2026-04-23)

All maps were audited with `check_configs.py` and satisfy the following:

### Station overlap — none
- No idle-wait station overlaps with: obstacles, induct stations, eject stations, charging stations, or agent starting positions
- Idle-wait and charging stations are distinct cells on every map (separate flows)

### Station accessibility — all stations have >= 2 free neighbors
- **Idle-wait stations**: all have 4 free (non-obstacle) neighbors
- **Induct stations**: all have 2–4 free neighbors (left-wall placement with open floor to the right)
- **Eject stations**: all have >= 2 free neighbors — no isolated or single-access eject cells

No station is a dead-end or bottleneck (single-file only access).

### Idle-wait station placement rationale
Idle-wait stations are placed along the left highway, 1–2 cells right of the left-wall induct stations. This keeps idle agents out of the main task floor while remaining reachable from anywhere on the map.

| Map | Idle-wait x | y-levels |
|-----|-------------|----------|
| `warehouse_small` | x=3,4 | induct-dock rows |
| `warehouse_large` | x=3,4 | y=5,12,19,26,33,40,47,54 |
| `kiva` | x=3,4 | y=6,12,18,24,30 (x=5 excluded — agent conflict) |
| `kiva_large` | x=5,7 | y=12,24,36,48,60 (x=6 excluded — agent conflict) |
| `crossdock` | x=3,4 | y=3,7,11,15,19,23 (one per lane) |
| `shelf_aisle` | x=3,4,5 | y=4,13,22,31,40,49,58 |

### Crossdock obstacle fix
The maintenance bay obstacle was trimmed from `y=22–25` to `y=24–25`. The original region fully enclosed eject station (40,23,0), leaving it with 0 reachable neighbors. After trimming, it has 3 free neighbors.

---

## Known intentional challenges (not bugs)

- **Narrow aisles**: `kiva`, `kiva_large`, `shelf_aisle` have single-cell-wide vertical aisles. Head-on agent conflicts are expected and are the reason CA*/RHCR is needed.
- **High agent density**: `shelf_aisle` (200 agents) and `kiva_large` (80 agents) are congestion-heavy by design, matching Li et al. 2021 and Wurman et al. 2008 benchmarks.
- **Directional flow**: `crossdock` has left-to-right task flow; agents traveling back to induct or charging move against the predominant direction.

## Not yet checked

- **Global connectivity**: whether every passable cell is reachable from every other. The audit only verifies local neighborhood (1-hop) access. Given that all maps use clear aisle corridors connecting all regions, connectivity is expected to hold, but no formal BFS check has been run.

---

## Energy tuning (approx 10 tasks per charge cycle)

| Map | `max_energy` | `charge_speed` | `charge_duration` | `charging_trigger_multiplier` |
|-----|-------------|----------------|-------------------|-------------------------------|
| `warehouse_small` | 650 | 1 | 20 | 5.0 |
| `warehouse_large` | 1250 | 2 | 20 | 5.0 |
| `kiva` | 800 | 1 | 20 | 5.0 |
| `kiva_large` | 1600 | 1 | 30 | 5.0 |
| `crossdock` | 1150 | 1 | 20 | 5.0 |
| `shelf_aisle` | 4300 | 1 | 40 | 5.0 |

`charge_speed` controls how fast the "charge remaining" counter decrements during docking. Docking time in timesteps = `charge_duration / charge_speed`. `max_energy` was tuned so an agent completes roughly 10 tasks before the charging trigger fires.
