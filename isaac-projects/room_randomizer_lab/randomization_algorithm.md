# Randomization Algorithm: Deep Dive

This document explains exactly how objects are randomly placed in the hospital room on every episode reset. It covers the full pipeline from room geometry definitions through collision detection to final physics state writes.

---

## 1. Room Geometry

The room is a rectangular area defined in [constants.py](file:///Users/cezarioa/Projects/isaac-projects/room_randomizer_lab/constants.py#L16-L25):

```
ROOM_X_MIN = -13.0      ROOM_X_MAX = -0.5
ROOM_Y_MIN = -11.25     ROOM_Y_MAX = -5.0
FLOOR_Z    =  0.0       ROBOT_Z    =  0.0328
```

All coordinates are in **metres**, using the coordinate system from the original `new_base_room.usda` file. The room is 12.5 m wide (X) and 6.25 m deep (Y), with z = 0 at floor level.

```
  Y = -5.0 (front of room)
  +-----------------------------------------------+
  |                                               |
  |              ROOM INTERIOR                    |
  |                                               |
  |    Table group samples from:                  |
  |    x in [-10, -5], y in [-9, -6]              |
  |                                               |
  |                                               |
  |                                               | X = -0.5
  |                                     right     |  (right wall)
  |                                     wall      |
  |                                     zone      |
  |                                     samples   |
  |                                     y in      |
  |                                     [-10,-7]  |
  |                                               |
  +-----------------------------------------------+
  Y = -11.25 (back wall)
  X = -13.0                            X = -0.5
       ^-- back wall zone samples x in [-12, -4] --^
```

The two physical walls where props can be placed are:
- **Back wall** at Y = -10.95 (the room-facing surface)
- **Right wall** at X = -2.5 (the room-facing surface)

---

## 2. Oriented Bounding Boxes (OBB)

Every placeable object has a 2D footprint defined as a `BBox(half_w, half_d)` in [constants.py](file:///Users/cezarioa/Projects/isaac-projects/room_randomizer_lab/constants.py#L36-L44). `half_w` is half the width along the object's local X axis, and `half_d` is half the depth along the object's local Y axis.

An OBB in this system is a 5-element tuple: `(cx, cy, half_w, half_d, yaw_rad)`.

### How corners are computed

Given an OBB at centre `(cx, cy)` with half-extents `(hw, hd)` and rotation `yaw`, the 4 world-space corners are computed by [obb_corners()](file:///Users/cezarioa/Projects/isaac-projects/room_randomizer_lab/placement_utils.py#L55-L69):

```
For each local corner (+hw,+hd), (-hw,+hd), (-hw,-hd), (+hw,-hd):
    world_x = cx + local_x * cos(yaw) - local_y * sin(yaw)
    world_y = cy + local_x * sin(yaw) + local_y * cos(yaw)
```

This is a standard 2D rotation matrix applied to each local corner, then translated to the world centre.

### Room bounds check

[obb_inside_room()](file:///Users/cezarioa/Projects/isaac-projects/room_randomizer_lab/placement_utils.py#L122-L129) checks that **all 4 corners** of an OBB fall within the room rectangle:

```
For each corner (wx, wy):
    ROOM_X_MIN <= wx <= ROOM_X_MAX   AND
    ROOM_Y_MIN <= wy <= ROOM_Y_MAX
```

If any corner is outside, the placement is rejected.

---

## 3. Collision Detection: Separating Axis Theorem (SAT)

Two OBBs are tested for overlap using the SAT algorithm in [obb_overlap()](file:///Users/cezarioa/Projects/isaac-projects/room_randomizer_lab/placement_utils.py#L82-L115). Here is how it works step by step:

**Step 1: Inflate both boxes by the margin.**

Both boxes get `OBB_PLACEMENT_MARGIN = 0.15 m` added to their half-extents. This enforces a minimum gap between objects.

```
a_inflated = (a.cx, a.cy, a.half_w + 0.15, a.half_d + 0.15, a.yaw)
b_inflated = (b.cx, b.cy, b.half_w + 0.15, b.half_d + 0.15, b.yaw)
```

**Step 2: Compute the 4 world-space corners of each inflated box.**

**Step 3: Build 4 candidate separating axes.**

For each box, compute 2 edge-normal directions from its yaw angle:

```
Box A with yaw_a:
    axis_1 = (cos(yaw_a),  sin(yaw_a))    -- along A's local X
    axis_2 = (-sin(yaw_a), cos(yaw_a))    -- along A's local Y

Box B with yaw_b:
    axis_3 = (cos(yaw_b),  sin(yaw_b))    -- along B's local X
    axis_4 = (-sin(yaw_b), cos(yaw_b))    -- along B's local Y
```

**Step 4: Project all corners onto each axis and check for gaps.**

For each of the 4 axes, project all 4 corners of A and all 4 corners of B onto it using the dot product:

```
projection = corner_x * axis_x + corner_y * axis_y
```

This gives a `(min, max)` interval for A and a `(min, max)` interval for B on that axis.

**Step 5: If there is a gap on ANY axis, the boxes do NOT overlap.**

```
if max_a < min_b  OR  max_b < min_a:
    return False   (no collision on this axis = no collision at all)
```

**Step 6: If ALL 4 axes show overlap, the boxes DO overlap.**

```
return True   (collision!)
```

### Why SAT works

The SAT theorem states: two convex shapes do NOT overlap if and only if there exists a line (axis) onto which their projections are separated. For two rectangles, the only candidate axes you need to check are the 4 edge normals (2 per rectangle). If none of them separate the shapes, the shapes must overlap.

---

## 4. Wall Prop Metadata

Each wall prop has metadata defined in [WALL_PROP_META](file:///Users/cezarioa/Projects/isaac-projects/room_randomizer_lab/constants.py#L148-L194):

| Config Field | Meaning |
|---|---|
| `bbox` | The 2D footprint as `BBox(half_w, half_d)` |
| `tall` | If True, gets placement priority (placed first) |
| `wall_offset` | Extra distance pushed away from the wall surface (metres) |
| `yaw_offset` | Extra rotation added to the wall's base yaw (radians) |
| `allowed_walls` | Which walls this prop can be placed on: `("back",)`, `("right",)`, or `("back", "right")` |

### Current wall prop OBB dimensions

| Object | half_w | half_d | Tall | wall_offset | yaw_offset | Allowed |
|---|---|---|---|---|---|---|
| medical_cabinet | 0.436 | 0.328 | Yes | +0.25 | pi | right |
| shelf_set | 0.861 | 0.280 | Yes | -0.22 | pi | right |
| supply_cabinet | 0.367 | 0.737 | Yes | +0.167 | pi/2 | back |
| trash_can | 0.150 | 0.150 | No | 0.0 | 0.0 | both |
| plant_a | 0.352 | 0.404 | No | 0.0 | 0.0 | both |
| plant_b | 0.252 | 0.300 | No | 0.0 | 0.0 | both |
| supply_cart_a | 0.421 | 0.228 | No | 0.0 | 0.0 | both |
| supply_cart_b | 0.298 | 0.556 | No | 0.0 | pi/2 | both |

---

## 5. Wall Zones

Wall zones are continuous strips along a wall where props can be placed. Defined in [WALL_ZONES](file:///Users/cezarioa/Projects/isaac-projects/room_randomizer_lab/constants.py#L66-L83):

| Zone | Free axis | Range | Fixed coord | Base yaw |
|---|---|---|---|---|
| Back wall | X | [-12.0, -4.0] | Y = -10.75 | 0.0 (face +Y) |
| Right wall | Y | [-10.0, -7.0] | X = -3.0 | pi/2 (face -X) |

**How a position is sampled** from a wall zone (in [_sample_wall_position()](file:///Users/cezarioa/Projects/isaac-projects/room_randomizer_lab/room_events.py#L136-L155)):

For the **back wall**:
```
cx = random_uniform(sample_min, sample_max)    # random X along the wall
cy = fixed_coord + wall_offset                 # fixed Y near the wall + prop offset
yaw = base_yaw + yaw_offset                   # 0.0 + prop's yaw_offset
```

For the **right wall**:
```
cx = fixed_coord - wall_offset                 # fixed X near the wall - prop offset
cy = random_uniform(sample_min, sample_max)    # random Y along the wall
yaw = base_yaw + yaw_offset                   # pi/2 + prop's yaw_offset
```

Note the sign difference: on the back wall, positive `wall_offset` pushes the object into the room (+Y). On the right wall, positive `wall_offset` also pushes into the room (-X direction), achieved by subtracting the offset.

---

## 6. Phase 1: Wall Prop Placement

Implemented in [_place_wall_props()](file:///Users/cezarioa/Projects/isaac-projects/room_randomizer_lab/room_events.py#L277-L372).

### Algorithm

```
1. SORT wall props by priority: tall props first, then non-tall.
   This gives large furniture (cabinets, shelves) first pick of positions.

2. FOR EACH wall prop (in priority order):
   FOR EACH environment:

       placed_obbs = [all OBBs already placed in this env]

       3. TRY up to 100 random positions:
           a. Pick a random wall zone from allowed_walls
           b. Sample a random (cx, cy, yaw) from that zone
           c. Build candidate OBB: make_obb(cx, cy, bbox, yaw)
           d. CHECK: obb_inside_room(candidate)?
              If NO -> try again
           e. CHECK: obb_overlap_any(candidate, placed_obbs, margin=0.15)?
              If YES -> try again
           f. VALID! Add to placed_obbs, record position, break

       4. If all 100 tries failed:
           Despawn the object (set z = -100)

   5. Write all positions to simulation:
       build_root_state(positions, yaws, env_origins, env_ids, default_state)
       asset.write_root_state_to_sim(root_state, env_ids)
```

### Example walkthrough

Suppose we are placing `medical_cabinet` (tall, right-wall only):

1. Pick wall zone: right wall (only option since `allowed_walls=("right",)`)
2. Sample position: `cy = random(-10.0, -7.0)`, `cx = -3.0 - 0.25 = -3.25`
3. Compute yaw: `pi/2 + pi = 3pi/2`
4. Build OBB: `(cx=-3.25, cy=random, hw=0.436, hd=0.328, yaw=3pi/2)`
5. Compute 4 corners, check all inside room bounds
6. Check overlap with all previously placed OBBs (inflated by 0.15m margin)
7. If valid, accept. If not, try a new random Y position.

---

## 7. Table Group: Desk + Chair + Robot

### Orbital offsets

The chair and robot are positioned relative to the desk using fixed **orbital offsets** defined in [constants.py](file:///Users/cezarioa/Projects/isaac-projects/room_randomizer_lab/constants.py#L115-L116):

```
CHAIR_ORBIT_OFFSET = (0.0, -1.00)    # 1.0m behind the desk (local Y)
ROBOT_ORBIT_OFFSET = (-1.95, 1.10)   # 1.95m left, 1.10m in front (local frame)
```

These offsets are in the desk's local frame and get rotated by the desk's yaw to produce world positions.

### How orbital positions are computed

[offset_from_yaw()](file:///Users/cezarioa/Projects/isaac-projects/room_randomizer_lab/placement_utils.py#L164-L177) applies a 2D rotation:

```
Given desk at (dx, dy) with yaw, and local offset (lx, ly):
    world_x = dx + lx * cos(yaw) - ly * sin(yaw)
    world_y = dy + lx * sin(yaw) + ly * cos(yaw)
```

So if the desk is at (-7, -8) facing yaw=0:
- Chair: (-7 + 0, -8 + (-1.0)) = (-7.0, -9.0) -- 1m directly behind
- Robot: (-7 + (-1.95), -8 + 1.10) = (-8.95, -6.9) -- left and in front

If the desk is rotated by yaw = pi/2 (90 degrees, facing left):
- The chair offset (-1.0 in local Y) now points in the -X direction
- The robot offset rotates accordingly

### Table group OBB dimensions

| Object | half_w | half_d |
|---|---|---|
| Desk | 0.745 | 0.227 |
| Chair | 0.347 | 0.343 |
| Robot (Ridgeback) | 0.65 | 0.50 |

---

## 8. Phase 2: Table Group Placement

Implemented in [_place_table_group()](file:///Users/cezarioa/Projects/isaac-projects/room_randomizer_lab/room_events.py#L378-L504).

### Algorithm

```
FOR EACH environment:

    1. TRY up to 300 random configurations:
        a. Sample random desk position:
             dx = random(-10.0, -5.0)
             dy = random(-9.0, -6.0)
             dyaw = random(0, 2*pi)

        b. Compute satellite positions:
             chair_x, chair_y = offset_from_yaw(dx, dy, dyaw, 0.0, -1.0)
             robot_x, robot_y = offset_from_yaw(dx, dy, dyaw, -1.95, 1.10)

        c. Compute satellite yaws:
             chair_yaw = dyaw + pi        (faces opposite the desk)
             robot_yaw = dyaw - pi/2      (faces 90 degrees from desk)

        d. Build 3 OBBs: desk, chair, robot

        e. VALIDATE the group:
             - All 3 OBBs inside room bounds?
             - No overlap between desk/chair/robot (mutual check)?
             - No overlap with any wall prop already placed?

        f. If valid -> accept, break

    2. If all 300 tries failed, FALLBACK:
        Fix position to (-7.50, -7.50), try 300 random yaws only.

    3. If fallback also failed:
        Despawn all 3 objects (z = -100)

    4. Write desk, chair, ridgeback positions to simulation.
```

### Why 300 tries?

The table group is 3 objects that must ALL fit simultaneously. The chair and robot extend significantly from the desk centre (up to ~2m), so many random positions will place one of them outside the room or overlapping a wall prop. 300 tries provides high confidence of finding a valid configuration.

---

## 9. Tabletop Object Metadata

Defined in [TABLE_PROP_META](file:///Users/cezarioa/Projects/isaac-projects/room_randomizer_lab/constants.py#L215-L219):

| Object | half_w | half_d |
|---|---|---|
| coffee_cup | 0.043 | 0.043 |
| desk_lamp | 0.241 | 0.134 |
| box_portable | 0.195 | 0.145 |

The desk surface sampling area is defined by:
```
DESK_LOCAL_X_MIN = -0.38    DESK_LOCAL_X_MAX = 0.38
DESK_LOCAL_Y_MIN = -0.22    DESK_LOCAL_Y_MAX = 0.22
DESK_OBJECT_MARGIN = 0.03   (3cm gap between tabletop OBBs)
```

---

## 10. Phase 3: Tabletop Object Placement

Implemented in [_place_desk_objects()](file:///Users/cezarioa/Projects/isaac-projects/room_randomizer_lab/room_events.py#L511-L608).

### Algorithm

```
FOR EACH environment:

    1. If the desk was despawned (z = -100), skip tabletop objects.

    2. Randomly pick how many objects to show:
         count = random_int(min_table_objects, total_objects)
         (currently: random between 2 and 3)

    3. FOR EACH tabletop object (in order):
        If index >= count -> despawn (z = -100)

        Otherwise:
        a. TRY up to 100 positions on the desk surface:
             lx = random(-0.38, 0.38)     -- local X on desk
             ly = random(-0.22, 0.22)     -- local Y on desk
             obj_yaw = random(0, 2*pi)    -- random rotation

        b. Build candidate OBB in desk-local space:
             make_obb(lx, ly, bbox, obj_yaw)

        c. CHECK overlap with other tabletop OBBs (margin = 0.03m):
             obb_overlap_any(candidate, desk_placed_obbs, margin=0.03)

        d. If valid:
             - Transform local (lx, ly) to world coordinates:
                 wx, wy = offset_from_yaw(desk_x, desk_y, desk_yaw, lx, ly)
             - Compute world yaw: desk_yaw + obj_yaw
             - Write to simulation at z = DESK_OBJECT_Z (0.82m)

        e. If all 100 tries failed: despawn the object
```

### Local-to-world transform

Tabletop objects are first placed in the desk's local coordinate frame (a small rectangle on the desk surface). They are then transformed to world coordinates using the desk's position and yaw:

```
world_x = desk_x + local_x * cos(desk_yaw) - local_y * sin(desk_yaw)
world_y = desk_y + local_x * sin(desk_yaw) + local_y * cos(desk_yaw)
world_yaw = desk_yaw + local_yaw
world_z = DESK_TOP_Z + 0.04 = 0.82m
```

---

## 11. Despawning Failed Objects

Any object that fails placement is "despawned" by teleporting it underground:

```
DESPAWN_Z = -100.0
```

The object still exists in the simulation, but at z = -100 it is far below the floor and invisible. This avoids the complexity of dynamically adding/removing prims.

---

## 12. Writing to Physics (build_root_state)

After computing positions, the results are written to the GPU PhysX simulation via [build_root_state()](file:///Users/cezarioa/Projects/isaac-projects/room_randomizer_lab/placement_utils.py#L232-L251):

```
1. Clone the asset's default root state for the given env IDs
   -> (N, 13) tensor: [x, y, z, qw, qx, qy, qz, vx, vy, vz, wx, wy, wz]

2. Set positions (add env_origins offset for multi-env):
   state[:, 0] = pos_x + env_origin_x
   state[:, 1] = pos_y + env_origin_y
   state[:, 2] = pos_z + env_origin_z

3. Convert yaw to quaternion:
   half = yaw / 2
   qw = cos(half),  qx = 0,  qy = 0,  qz = sin(half)

4. Zero all velocities:
   state[:, 7:] = 0.0

5. Call asset.write_root_state_to_sim(state, env_ids)
```

The `env_origins` offset is critical for multi-environment setups: each environment is placed 16m apart in a grid, so the local room coordinates (-13 to -0.5 in X) must be shifted by the environment's world-space origin.

---

## 13. Hiding Duplicate Visual Props

The room shell USD (`new_base_room.usda`) contains the original authored furniture meshes at their default positions. Since we spawn separate rigid objects for randomization, these originals would cause visual doubling.

[_hide_duplicate_visual_props()](file:///Users/cezarioa/Projects/isaac-projects/room_randomizer_lab/room_events.py#L72-L90) runs **once** on the first reset and calls `UsdGeom.Imageable.MakeInvisible()` on 10 hardcoded prim paths:

```
RoomShell/Environment/props/room_props/SM_Desk_04a
RoomShell/Environment/props/room_props/SM_Chair_04a
RoomShell/Environment/props/wall_props/SM_MedicalCabinet_01a
RoomShell/Environment/props/wall_props/SM_ShelfSet_01a
RoomShell/Environment/props/wall_props/SM_SupplyCabinet_01c
RoomShell/Environment/props/wall_props/SM_SupplyCart_02a
RoomShell/Environment/props/wall_props/SM_SupplyCart_03a
RoomShell/Environment/props/wall_props/SM_TrashCan
RoomShell/Environment/props/wall_props/SM_Plant01
RoomShell/Environment/props/wall_props/SM_Plant02
```

---

## 14. Summary: What Happens on Each Reset

```
1. [First reset only] Hide 10 duplicate visual props in room shell

2. [Phase 1] Place 8 wall props:
   - Sort by priority (tall first)
   - For each prop, try 100 random positions on allowed walls
   - SAT collision check against all previously placed OBBs
   - Write positions to GPU PhysX

3. [Phase 2] Place table group (desk + chair + robot):
   - Try 300 random (x, y, yaw) in room interior
   - Compute chair and robot via orbital offsets
   - Validate all 3 OBBs (room bounds + no overlap with wall props + no mutual overlap)
   - Fallback to fixed position if needed
   - Write positions to GPU PhysX

4. [Phase 3] Place 2-3 tabletop objects on desk:
   - For each object, try 100 random (lx, ly, yaw) on desk surface
   - OBB collision check against other tabletop objects (3cm margin)
   - Transform local to world coordinates using desk yaw
   - Write positions to GPU PhysX
```

All 3 phases share a common `all_placed` list of OBBs per environment, ensuring that no object overlaps any other across the entire room.
