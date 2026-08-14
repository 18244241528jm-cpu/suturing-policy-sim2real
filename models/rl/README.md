# Released RL checkpoints

These are project-generated checkpoints, not third-party model weights.

| File | Contract | Use |
|---|---|---|
| `m3_measured_r3_100k.zip` | measured observation, historical R3 data, 100k checkpoint | Exact SIM-S4 evaluator initialization and historical policy comparison |
| `r6_unified_single_goal_yaw15_seed1_final.zip` | measured observation, single frozen goal, yaw ±15°, seed 1 | Latest semantics-consistent RL baseline |

SIM-S4 selects D2 `goal-servo`, so its action does not come from M3; the
historical evaluator still constructs the policy stack and therefore needs the
M3 file. Do not describe SIM-S4 29/30 as an RL success rate.

Verify both files against `MANIFEST.sha256` before use.
