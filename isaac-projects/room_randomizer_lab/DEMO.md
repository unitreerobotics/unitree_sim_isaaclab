# Hospital room randomizer demo

Run the interactive demo from the repository root:

```bash
/home/vilmos/IsaacLab/isaaclab.sh -p isaac-projects/room_randomizer_lab/run_randomizer.py --num_envs 1 --enable_cameras --visualize_obbs --visualize_obb_margins --reset_interval 600
```

The overlay shows the 2D oriented collision footprint used by the placement
algorithm. It is not a full 3D mesh bounding box.

- Blue: wall props
- Green: desk and chair
- Purple: Ridgeback
- Orange: tabletop objects
- Yellow: collision-clearance margin

`--reset_interval 600` keeps each layout visible for roughly ten seconds. Omit
`--visualize_obb_margins` for a cleaner view containing only the object
footprints. Press `Ctrl+C` to stop the demo.
