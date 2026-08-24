# Hospital Teleoperation Integration Notes

## Summary

This update integrates the Unitree G1 29-DoF + Dex1 teleoperation work with
the randomized hospital layout. Robot, packing table, target, hospital props,
and Ridgeback now share one per-environment layout state.

## Included changes

- Registers `Isaac-PickPlace-Hospital-G129-Dex1-Wholebody` alongside the
  existing fixed-base Dex1 task.
- Stores robot/table world poses, target/tabletop local poses, wall-facing
  layout, and Ridgeback waiting/route targets in `env._room_layout_state`.
- Validates Ridgeback waiting, left/right staging, left/right delivery, and
  swept movement corridors together with the table/robot group.
- Makes the medicine bottle, hand sanitizer, gauze box, and specimen cup
  mandatory teleoperation objects. Generic lamp/cube distractors are
  intentionally inactive in this task, so object-count behavior is explicit.
- Uses the current table transform for manual target reset, dropped-object
  respawn, post-delivery reset, initial reset, and full-scene reset.
- Uses one reset function with a persistent table-randomization switch. The
  initial Meta Quest/DDS teleoperation layout keeps the table at the validated
  hospital anchor while wall and tabletop props still scramble. DDS reset
  category `2` (the full-reset button) enables full table/robot/Ridgeback-group
  randomization for that reset and subsequent resets. Category `3` restores
  fixed-table mode while still scrambling the room and tabletop objects.
- Resolves hand side and Ridgeback routes in the randomized G1 frame and
  returns to the stored waiting pose.
- Restricts DDS/Ridgeback teleoperation to one environment with a clear error,
  and uses 16 m scene spacing for both fixed-base and Wholebody variants.
- Updates DDS and Wholebody action handling and avoids redundant simulation
  resets that can stall RTX-camera startup.
- Updates the `teleimager` submodule pointer to the available upstream `sim`
  commit `b81de44`; the previous target commit was no longer fetchable from the
  public submodule remote.

## Run examples

Fixed-base G1 + Dex1:

```bash
python sim_main.py \
  --headless \
  --device cuda:0 \
  --enable_cameras \
  --task Isaac-PickPlace-Cylinder-G129-Dex1-Joint \
  --enable_dex1_dds \
  --robot_type g129
```

Wholebody hospital task:

```bash
python sim_main.py \
  --headless \
  --device cuda:0 \
  --enable_cameras \
  --task Isaac-PickPlace-Hospital-G129-Dex1-Wholebody \
  --enable_dex1_dds \
  --enable_wholebody_dds \
  --robot_type g129
```

For GUI validation, remove `--headless` from either command. The same commands
are used for RTX-camera startup checks; keep `--enable_cameras` enabled.

## Reset and assistant validation matrix

On a GPU host with the complete downloaded asset bundle, validate each task in
both GUI and headless modes:

1. Start the task and confirm the table stays at `(-7.5, -7.5)` while wall and
   tabletop props are randomized.
2. Send DDS reset category `1`; only the target must move, and it must remain
   on the current packing table with zero velocity.
3. Send DDS reset category `2`; the table-randomization switch must turn on and
   the scene must generate one new valid table/robot/Ridgeback layout with all
   wall and tabletop props scrambled as usual.
4. Send DDS reset category `3`; wall and tabletop props must scramble while
   the table returns to and remains at its authored fixed pose.
5. Drop the target below the respawn threshold and confirm it respawns on the
   current table rather than at an authored world pose.
6. Grasp with each hand in turn; confirm staging, delivery, basket placement,
   return, and collision-free target respawn.
7. Run at least 100 consecutive full resets and check logs for
   `PLACEMENT_ERROR`, Python/config/import errors, and duplicate reset/render
   stalls.
7. Confirm DDS joint commands are received for fixed-base and Wholebody
   control and that RTX cameras publish after startup and resets.

## Validation report (2026-08-21)

- `python isaac-projects/room_randomizer_lab/test_placement.py`: passed 1,000
  deterministic seeds. Coverage includes wall/furniture clearance, every
  Ridgeback pose and corridor, room bounds, tabletop bounds, tabletop OBB
  separation, reserved-area clearance, target respawn, full-reset regeneration,
  and same/different-seed behavior.
- `python -m py_compile ...`: passed for all modified Python modules.
- `git diff --check`: passed.
- Full Isaac Sim GUI/headless execution is not verified in this checkout. The
  local ignored asset bundle is missing `ridgeback_base_only.usda`, the basket,
  and the four hospital-object USDs, so launching either task would fail before
  meaningful runtime validation. Install the project asset bundle with
  `fetch_assets.sh` on the GPU host, then run the matrix above.

## Collaboration notes

- Target branch: `Cognitive-Software-Labs/core_unitree_sim_isaaclab:hospital_env`
- Personal backup branch:
  `ShidanChen/unitree_sim_isaaclab:integration/hospital-env-20260821`
- Companion XR client branch:
  `ShidanChen/xr_teleoperate:integration/hospital-env-teleoperation-20260821`
  (Quest button mapping, motion-stick commands, reset publishing, and
  fixed-base waist-yaw control).
- Integration branch: `fix/teleop-randomized-hospital-integration`
- Starting commit: `62eda54829a82a22e4fbf9a09558641ba20e7735`
- Future updates should fetch and merge the latest `hospital_env` before
  pushing, because multiple contributors are working on the same branch.
