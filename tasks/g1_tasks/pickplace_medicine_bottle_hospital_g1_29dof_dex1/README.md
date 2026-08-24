# Hospital Two-Pill Pick-Place (G1 29-DoF + Dex1)

Meta Quest teleoperation and evaluation task in the hospital room. The goal is:

> Place both pill bottles into the rear containers.

Gym task id: `Isaac-PickPlace-MedicineBottle-Hospital-G129-Dex1-Joint`

## Tabletop objects and physics

The task spawns the exact six natural-scale (`1.0`) objects selected in
`Screenshot-2026-08-23_16-05-21.png`:

- Hospital: `SM_PillBottle_01t`, `SM_PillBottle_01v`, `SM_BottleA`, `SM_BottleF`
- Office: `SM_MarkerBlue`, `SM_MarkerYellow`

No object is scaled up or down. Every prop is a positive-mass rigid body with
gravity, high-friction contact material, an enabled mesh collider, and
`convexDecomposition` collision approximation. The pill bottles are the only
scored targets; BottleA, BottleF, and the markers are physical clutter.

Because the natural pill bottles are narrower than the stock Dex1 aperture at
its mapped full-close target, this task adds invisible spherical contact pads
at the distal finger colliders (9 mm left, 10 mm right). This changes only
contact geometry: the rendered hands and object scales remain unchanged.

In Quest controller mode, each index trigger is a binary hold/release control:
hold it to close and grasp; release it to open and relax. It commands the fast
task-local actuator between the fully open Dex1 target (`-0.020 m`) when
released and the authored fully closed limit (`0.0245 m`) when pressed.

All six are randomized collision-free in the handward table-local area
`x=[-0.32, 0.18]`, `y=[-0.16, 0.12]`, with 15 mm placement clearance applied to
each footprint. The bounds are centered between the default Dex1 hands and
keep every footprint inside the front camera's 96-degree horizontal FOV. The
region follows the table when the complete room layout is randomized.

## Rear-container goal

One green crate remains a collision child of the kinematic Ridgeback platform.
On each scene reset, the platform cycles through one of fourteen equal-radius points
on the side arc around G1's feet. It starts outside the rear no-spawn zone,
advances in 5-degree steps toward the table, and stops before table contact.
It always turns its crate side toward G1 and keeps the screenshot-calibrated
G1-foot-to-Ridgeback-tip clearance at 10 cm at every point. The 0.85 m value is
the root-to-root radius, not the visible edge clearance. The five added points
on each side continue at 25, 30, 35, 40, and 45 degrees. The arc deliberately
does not reject possible G1/table OBB overlaps, as requested. Scoring composes the crate's local transform with its live Ridgeback
pose, so it remains correct after fixed or randomized room resets.

Both pill bottles are placed in this crate. Reward is `0.0`, `0.5`, or `1.0`
for zero, one, or two contained
pill bottles. When both are contained, the terminal reward is emitted and the
episode immediately performs the Y-style reset: fixed table, centered torso,
and newly scrambled room and tabletop props.

## Reset behavior

- Reset category `1` respawns all six tabletop objects without moving the
  table, G1, room furniture, or Ridgeback containers.
- Reset category `2` keeps its existing full-reset behavior and randomizes the
  complete table/robot/logistics group.
- Reset category `3` keeps its existing Y-button behavior: scramble the room
  and all six objects while restoring the calibrated fixed table.
- Reset category `4` / Quest **A** moves only the Ridgeback to the next
  collision-free arc point. G1, the table, and all tabletop props stay put.
- Reset category `5` / Quest **X** toggles recording of every live G1 camera.
  Each recording session is saved under `~/Desktop/G1_Camera_Recordings`.

The existing Quest Y/B bindings and their additive torso-centering behavior
are unchanged.

## Running

```bash
python sim_main.py --device cpu --enable_cameras \
  --task Isaac-PickPlace-MedicineBottle-Hospital-G129-Dex1-Joint \
  --enable_dex1_dds --robot_type g129
```

The task streams the front and two wrist cameras. In controller mode, the right
stick controls simulation-only waist yaw and forward pitch; waist roll and the
lower body remain locked.

To run bilateral Dex1 contact validation for both scored targets:

```bash
python tools/validate_dex1_grasp_contacts.py --headless --device cpu
```

The first run requires access to NVIDIA's asset server. Isaac Sim caches the
USD assets for subsequent runs.
