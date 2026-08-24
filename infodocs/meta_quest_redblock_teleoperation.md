# Meta Quest red-block teleoperation

For the complete operator manual—including installation, every task/profile,
all runtime settings, ports and DDS topics, architecture, reset behavior,
troubleshooting, and validation—use
[`Meta_Quest_RedBlock_Operator_Guide.pdf`](Meta_Quest_RedBlock_Operator_Guide.pdf).
Its maintainable source is
[`meta_quest_redblock_operator_guide.md`](meta_quest_redblock_operator_guide.md).

Use `--meta_quest` when running a red-block task with Unitree
`xr_teleoperate`. The option enables the head and wrist cameras, selects the
correct robot and hand DDS bridge, uses DDS action input, and publishes camera
frames through teleimager's ZMQ transport.

Quest mode is deliberately limited to one environment by `sim_main.py`. The
Quest browser connects to `xr_teleoperate`; `xr_teleoperate` connects to this
simulator's ZMQ camera endpoints and DDS domain 1.

## Before launching

1. Fetch the simulator assets with `. fetch_assets.sh` if `assets/` is absent.
2. Install and configure Unitree `xr_teleoperate` on the computer that will
   bridge the Quest headset to DDS.
3. Put the simulator and the xr_teleoperate computer on the same reachable
   network. In xr_teleoperate, set the image-server address to the simulator's
   IP address.
4. Ensure ports `55555`, `55556`, and `55557` are reachable from the
   xr_teleoperate computer. All DDS participants must use domain/channel 1.
5. Keep real robots off this DDS domain while testing unless sending them the
   same commands is intentional.

No teleimager TLS certificate is required for this path: Quest mode uses ZMQ
between Isaac Sim and xr_teleoperate and disables teleimager's optional direct
WebRTC endpoints.

Do not add Isaac Lab's `--xr` option. That enables Isaac Sim's native XR
experience; this repository's supported Quest path is the xr_teleoperate DDS +
ZMQ bridge configured by `--meta_quest`.

## Launch commands

Run one command from the repository root. For a local GUI, omit `--headless`.
For a machine without a display, retain `--headless`; never replace it with
`--no_render`, because that prevents RTX camera frames from updating.

```bash
# G1 29-DoF, Dex1 gripper
python sim_main.py --device cpu --headless --meta_quest \
  --task Isaac-PickPlace-RedBlock-G129-Dex1-Joint

# G1 29-DoF, Dex1 gripper, hospital medicine-bottle scene
python sim_main.py --device cpu --headless --meta_quest \
  --task Isaac-PickPlace-MedicineBottle-Hospital-G129-Dex1-Joint

# G1 29-DoF, Dex3 hand
python sim_main.py --device cpu --headless --meta_quest \
  --task Isaac-PickPlace-RedBlock-G129-Dex3-Joint

# G1 29-DoF, Inspire hand
python sim_main.py --device cpu --headless --meta_quest \
  --task Isaac-PickPlace-RedBlock-G129-Inspire-Joint

# H1-2 27-DoF, Inspire hand
python sim_main.py --device cpu --headless --meta_quest \
  --task Isaac-PickPlace-RedBlock-H12-27dof-Inspire-Joint

# G1 drawer variants
python sim_main.py --device cpu --headless --meta_quest \
  --task Isaac-Pick-Redblock-Into-Drawer-G129-Dex1-Joint
python sim_main.py --device cpu --headless --meta_quest \
  --task Isaac-Pick-Redblock-Into-Drawer-G129-Dex3-Joint
```

Do not also pass a hand flag or `--robot_type`; Quest mode derives those values
from the task. It rejects an incompatible hand flag with a clear startup error.

For the hospital medicine-bottle task in controller mode, the right thumbstick
controls the simulation torso. Move it left/right to rotate toward the matching
rear crate. Push it forward to lean over the crate and lower the hands; pull it
back to return upright. Quest X starts/stops recording from every available G1
camera into `~/Desktop/G1_Camera_Recordings`; A moves the Ridgeback to its next arc point; Y/B
scene resets return the torso to centered and
upright. Waist roll and every lower-body joint remain locked. Torso control is
disabled automatically in hand-tracking or locomotion mode.

## Expected startup and troubleshooting

The simulator should print a `[Meta Quest]` line followed by successful image
server and DDS creation. The camera mapping is:

| View | ZMQ port | Isaac sensor |
| --- | ---: | --- |
| Head | 55555 | `front_camera` |
| Left wrist | 55556 | `left_wrist_camera` |
| Right wrist | 55557 | `right_wrist_camera` |

- Frozen/black Quest images: confirm `--enable_cameras` appears in the effective
  Quest configuration, remove `--no_render`, and verify the three ZMQ ports.
- Robot moves but hands do not: select the same robot/hand type in
  xr_teleoperate as the task name indicates.
- No DDS commands: verify every process uses DDS domain/channel 1 and that the
  network interface selected by CycloneDDS reaches the xr_teleoperate host.
- Task fails before the stage opens: run `. fetch_assets.sh` and check that the
  task's robot, room, and object USD files exist under `assets/`.

## Static validation

The profile and launch guards can be checked without starting Isaac Sim:

```bash
python -m unittest tests.test_meta_quest_redblocks
```

This verifies every listed task's robot/hand mapping, camera setup, ZMQ-only
transport, and rejection of `--no_render`, replay, unverified tasks, and
conflicting hand options.

A bounded end-to-end startup check that creates the stage, renders all three
cameras, starts teleimager and DDS, runs 25 control steps, and exits is:

```bash
python sim_main.py --device cuda:0 --headless --meta_quest --max_steps 25 \
  --task Isaac-PickPlace-RedBlock-G129-Dex1-Joint
```

## Validation report

Validated on 2026-08-21 with Isaac Sim 5.1, Isaac Lab, CUDA/Vulkan, and an
NVIDIA RTX PRO 6000 Blackwell. Each of the seven launch commands above was run
headless with `--max_steps`. All seven exited successfully after:

- constructing the selected robot, hand, red-block scene, and three RTX
  cameras;
- reporting the head and both wrist cameras ready at 480x640;
- starting ZMQ publishers on 55555-55557;
- registering the expected Dex1, Dex3, or Inspire DDS endpoint and the shared
  humanoid low-state/low-command endpoint;
- starting the DDS action provider and completing control steps.

An actual headset/xr_teleoperate connection was not available in this
workspace, so headset tracking packets were not physically exercised. The
simulator-side endpoints they connect to were started and verified.

Known non-blocking asset limitation: both drawer variants log an MDL compiler
error for the small-warehouse traffic-cone material (`OmniUe4Base_1`). The
material falls back, and both drawer scenes, all three cameras, DDS, and the
control loop still start and exit successfully. This affects that background
prop's appearance, not Quest control.
