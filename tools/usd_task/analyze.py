# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""Analysis of an arbitrary USD scene for task generation (pxr only, no simulator needed).

Finds what a manipulation task needs: free rigid bodies (graspable objects and heavy furniture), articulations,
static colliders, support surfaces (the collider top an object rests on), floor, lights, cameras, physics scenes,
markers (``RobotBase``, ``TargetZone``), units and up axis. Collider bounds are axis-aligned world boxes.
"""

from __future__ import annotations

import math
import os
from dataclasses import dataclass, field

import numpy as np
from pxr import Gf, Usd, UsdGeom, UsdLux, UsdPhysics

GRASPABLE_MAX_MASS = 3.0  # [kg]
GRASPABLE_MAX_SIZE = 0.30  # [m] largest bounding box edge
FLOOR_MAX_Z = 0.05  # [m] a collider whose top is below this is floor
SUPPORT_MAX_Z = 2.0  # [m] highest work surface considered


@dataclass
class Box:
    lo: tuple
    hi: tuple

    @property
    def center(self):
        return tuple((a + b) / 2 for a, b in zip(self.lo, self.hi))

    @property
    def size(self):
        return tuple(b - a for a, b in zip(self.lo, self.hi))

    def contains_xy(self, x, y, margin=0.0):
        return self.lo[0] - margin <= x <= self.hi[0] + margin and self.lo[1] - margin <= y <= self.hi[1] + margin

    def overlaps(self, other: "Box") -> bool:
        return all(self.lo[i] < other.hi[i] and other.lo[i] < self.hi[i] for i in range(3))

    def __str__(self):
        return "x[%.2f,%.2f] y[%.2f,%.2f] z[%.2f,%.2f]" % (self.lo[0], self.hi[0], self.lo[1], self.hi[1], self.lo[2], self.hi[2])


@dataclass
class Collider:
    path: str
    box: Box
    body: str | None  # rigid body / articulation link it belongs to (None: static)
    dynamic: bool  # belongs to a free (non-kinematic) rigid body outside articulations
    tris: np.ndarray | None = None  # (N, 3, 3) world triangles of a mesh collider (its box can be a whole room)

    def top_at(self, x, y, z_max):
        """Highest surface point of this collider at (x, y) not above z_max, or None."""
        if self.tris is None:
            if self.box.contains_xy(x, y) and self.box.hi[2] <= z_max:
                return self.box.hi[2]
            return None
        t = self.tris
        a, b, c = t[:, 0], t[:, 1], t[:, 2]
        v0, v1 = b[:, :2] - a[:, :2], c[:, :2] - a[:, :2]
        v2 = np.array([x, y]) - a[:, :2]
        den = v0[:, 0] * v1[:, 1] - v1[:, 0] * v0[:, 1]
        ok = np.abs(den) > 1e-12
        den = np.where(ok, den, 1.0)
        u = (v2[:, 0] * v1[:, 1] - v1[:, 0] * v2[:, 1]) / den
        v = (v0[:, 0] * v2[:, 1] - v2[:, 0] * v0[:, 1]) / den
        inside = ok & (u >= -1e-9) & (v >= -1e-9) & (u + v <= 1 + 1e-9)
        if not inside.any():
            return None
        z = a[:, 2] + u * (b[:, 2] - a[:, 2]) + v * (c[:, 2] - a[:, 2])
        z = z[inside & (z <= z_max + 1e-6)]
        return float(z.max()) if z.size else None

    def overlaps(self, box: Box) -> bool:
        if not self.box.overlaps(box):
            return False
        if self.tris is None:
            return True
        lo, hi = np.array(box.lo), np.array(box.hi)
        tlo, thi = self.tris.min(axis=1), self.tris.max(axis=1)
        return bool(((tlo < hi) & (thi > lo)).all(axis=1).any())


@dataclass
class Body:
    path: str
    box: Box
    mass: float | None
    kinematic: bool
    in_articulation: bool

    @property
    def graspable(self):
        return (not self.in_articulation and not self.kinematic and (self.mass is None or self.mass <= GRASPABLE_MAX_MASS)
                and max(self.box.size) <= GRASPABLE_MAX_SIZE)


@dataclass
class SceneInfo:
    usd_path: str
    root: str  # path of the default prim
    meters_per_unit: float
    up_axis: str
    bodies: list = field(default_factory=list)
    articulations: list = field(default_factory=list)
    colliders: list = field(default_factory=list)
    physics_scenes: list = field(default_factory=list)
    cameras: list = field(default_factory=list)
    lights: int = 0
    markers: dict = field(default_factory=dict)  # name -> (Box, yaw [rad])
    has_floor: bool = False
    bounds: Box | None = None

    # ------------------------------------------------------------------ queries
    def rel(self, path: str) -> str:
        """Path relative to the default prim (how the scene is referenced in the task)."""
        return path[len(self.root):].lstrip("/") if path.startswith(self.root) else path.lstrip("/")

    def body(self, path: str) -> Body | None:
        p = path if path.startswith("/") else f"{self.root}/{path}"
        return next((b for b in self.bodies if b.path == p), None)

    def body_of(self, path: str) -> Body | None:
        """Rigid body containing ``path`` (or the body itself)."""
        p = path if path.startswith("/") else f"{self.root}/{path}"
        best = None
        for b in self.bodies:
            if p == b.path or p.startswith(b.path + "/"):
                if best is None or len(b.path) > len(best.path):
                    best = b
        return best

    def graspables(self):
        return [b for b in self.bodies if b.graspable]

    def support_below(self, x, y, z_max, exclude=()):
        """Collider with the highest surface at (x, y) not above ``z_max`` (and not floor); (Collider, top) or None."""
        best, best_z = None, None
        z_max = min(z_max, SUPPORT_MAX_Z)
        for c in self.colliders:
            if any(c.path == e or c.path.startswith(e + "/") for e in exclude):
                continue
            if not c.box.contains_xy(x, y) or c.box.lo[2] > z_max:
                continue
            z = c.top_at(x, y, z_max + 0.02)
            if z is None or z <= FLOOR_MAX_Z:
                continue
            if best is None or z > best_z:
                best, best_z = c, z
        return (best, best_z) if best is not None else None

    def support_surface(self, support: Collider, top: float, x: float, y: float, tol=0.01, exclude=()):
        """Extent of the level surface at height ``top`` around (x, y): the support collider (for a mesh, its
        triangles at that height) grown by touching colliders with the same top (one table top / counter)."""
        if support.tris is not None:
            near = (np.abs(support.tris[:, :, 2] - top) < tol).all(axis=1)
            t = support.tris[near]
            if len(t):
                lo, hi = list(t.reshape(-1, 3).min(axis=0)), list(t.reshape(-1, 3).max(axis=0))
            else:
                lo, hi = [x - 0.05, y - 0.05, top], [x + 0.05, y + 0.05, top]
        else:
            lo, hi = list(support.box.lo), list(support.box.hi)
        changed = True
        members = {support.path}
        while changed:
            changed = False
            for c in self.colliders:
                if c.path in members or any(c.path == e or c.path.startswith(e + "/") for e in exclude):
                    continue
                if abs(c.box.hi[2] - top) > tol or c.box.size[0] * c.box.size[1] < 0.01:
                    continue
                if c.box.lo[0] <= hi[0] + 0.01 and c.box.hi[0] >= lo[0] - 0.01 and c.box.lo[1] <= hi[1] + 0.01 and c.box.hi[1] >= lo[1] - 0.01:
                    members.add(c.path)
                    for i in range(3):
                        lo[i] = min(lo[i], c.box.lo[i])
                        hi[i] = max(hi[i], c.box.hi[i])
                    changed = True
        return Box(tuple(lo), tuple(hi)), members

    def obstacles(self, box: Box, exclude=()):
        """Colliders (not floor) overlapping ``box``."""
        hits = []
        for c in self.colliders:
            if c.box.hi[2] <= FLOOR_MAX_Z:
                continue
            if any(c.path == e or c.path.startswith(e + "/") for e in exclude):
                continue
            if c.overlaps(box):
                hits.append(c)
        return hits


# ---------------------------------------------------------------------- geometry
def _world_box(prim, xform_cache, bbox_cache):
    m = xform_cache.GetLocalToWorldTransform(prim)
    pts = None
    if prim.IsA(UsdGeom.Cube):
        h = (prim.GetAttribute("size").Get() or 2.0) / 2
        pts = [(x, y, z) for x in (-h, h) for y in (-h, h) for z in (-h, h)]
    elif prim.IsA(UsdGeom.Sphere):
        r = prim.GetAttribute("radius").Get() or 1.0
        pts = [(x, y, z) for x in (-r, r) for y in (-r, r) for z in (-r, r)]
    elif prim.IsA(UsdGeom.Cylinder) or prim.IsA(UsdGeom.Capsule) or prim.IsA(UsdGeom.Cone):
        r = prim.GetAttribute("radius").Get() or 1.0
        hh = (prim.GetAttribute("height").Get() or 2.0) / 2 + (r if prim.IsA(UsdGeom.Capsule) else 0.0)
        ax = prim.GetAttribute("axis").Get() or "Z"
        pts = []
        for a in (-r, r):
            for b in (-r, r):
                for c in (-hh, hh):
                    pts.append({"X": (c, a, b), "Y": (a, c, b), "Z": (a, b, c)}[ax])
    elif prim.IsA(UsdGeom.Mesh):
        p = prim.GetAttribute("points").Get()
        if p:
            p = list(p)
            lo = [min(v[i] for v in p) for i in range(3)]
            hi = [max(v[i] for v in p) for i in range(3)]
            pts = [(x, y, z) for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])]
    if pts is None:  # anything else (incl. collision groups): imageable bounds of all purposes
        r = bbox_cache.ComputeWorldBound(prim).ComputeAlignedRange()
        if r.IsEmpty():
            return None
        return Box(tuple(r.GetMin()), tuple(r.GetMax()))
    w = [m.Transform(Gf.Vec3d(*q)) for q in pts]
    return Box(tuple(min(v[i] for v in w) for i in range(3)), tuple(max(v[i] for v in w) for i in range(3)))


def _world_triangles(prim, xform_cache):
    pts = prim.GetAttribute("points").Get()
    counts = prim.GetAttribute("faceVertexCounts").Get()
    idx = prim.GetAttribute("faceVertexIndices").Get()
    if not pts or not counts or not idx:
        return None
    m = np.array(xform_cache.GetLocalToWorldTransform(prim), dtype=np.float64)  # row-vector convention
    p = np.asarray(pts, dtype=np.float64)
    p = p @ m[:3, :3] + m[3, :3]
    counts = np.asarray(counts)
    idx = np.asarray(idx)
    tris = []
    start = 0
    if (counts == 3).all():
        return p[idx.reshape(-1, 3)]
    for n in counts:  # fan triangulation of polygons
        f = idx[start:start + n]
        for k in range(1, n - 1):
            tris.append((f[0], f[k], f[k + 1]))
        start += n
    return p[np.asarray(tris)] if tris else None


def yaw_of(prim, xform_cache) -> float:
    m = xform_cache.GetLocalToWorldTransform(prim)
    x = m.TransformDir(Gf.Vec3d(1, 0, 0))
    return math.atan2(x[1], x[0])


def analyze(usd_path: str) -> SceneInfo:
    usd_path = os.path.abspath(os.path.expanduser(usd_path))
    stage = Usd.Stage.Open(usd_path)
    if stage is None:
        raise RuntimeError(f"cannot open {usd_path}")
    root = stage.GetDefaultPrim()
    if not root:
        kids = [p for p in stage.GetPseudoRoot().GetChildren() if p.IsActive()]
        if len(kids) != 1:
            raise RuntimeError(f"{usd_path} has no defaultPrim (and {len(kids)} root prims); set a defaultPrim")
        root = kids[0]
    info = SceneInfo(usd_path=usd_path, root=str(root.GetPath()),
                     meters_per_unit=UsdGeom.GetStageMetersPerUnit(stage), up_axis=str(UsdGeom.GetStageUpAxis(stage)))
    xc = UsdGeom.XformCache()
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render", "proxy", "guide"], useExtentsHint=True)
    bc_vis = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"], useExtentsHint=True)

    art_roots, rigid = [], []
    for prim in Usd.PrimRange(root, Usd.TraverseInstanceProxies()):
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            art_roots.append(str(prim.GetPath()))
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            rigid.append(prim)
        if prim.IsA(UsdPhysics.Scene):
            info.physics_scenes.append(str(prim.GetPath()))
        if prim.IsA(UsdGeom.Camera):
            info.cameras.append(str(prim.GetPath()))
        if prim.HasAPI(UsdLux.LightAPI) or "Light" in prim.GetTypeName():
            info.lights += 1
        name = prim.GetName()
        if name in ("RobotBase", "TargetZone"):
            r = bc.ComputeWorldBound(prim).ComputeAlignedRange()
            t = xc.GetLocalToWorldTransform(prim).ExtractTranslation()
            box = Box(tuple(r.GetMin()), tuple(r.GetMax())) if not r.IsEmpty() else Box(tuple(t), tuple(t))
            info.markers[name] = (box, yaw_of(prim, xc))
    info.articulations = art_roots

    for prim in rigid:
        p = str(prim.GetPath())
        box = bc_vis.ComputeWorldBound(prim).ComputeAlignedRange()
        if box.IsEmpty():
            box = bc.ComputeWorldBound(prim).ComputeAlignedRange()
        mass = UsdPhysics.MassAPI(prim).GetMassAttr().Get() if prim.HasAPI(UsdPhysics.MassAPI) else None
        info.bodies.append(Body(
            path=p, box=Box(tuple(box.GetMin()), tuple(box.GetMax())), mass=mass or None,
            kinematic=bool(prim.GetAttribute("physics:kinematicEnabled").Get()),
            in_articulation=any(p == a or p.startswith(a + "/") for a in art_roots)))

    for prim in Usd.PrimRange(root, Usd.TraverseInstanceProxies()):
        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        en = prim.GetAttribute("physics:collisionEnabled").Get()
        if en is False:
            continue
        box = _world_box(prim, xc, bc)
        if box is None:
            continue
        p = str(prim.GetPath())
        body = info.body_of(p)
        dyn = body is not None and not body.kinematic and not body.in_articulation
        in_art = any(p.startswith(a + "/") or p == a for a in art_roots)
        tris = _world_triangles(prim, xc) if prim.IsA(UsdGeom.Mesh) else None
        info.colliders.append(Collider(path=p, box=box, body=body.path if body else (p if in_art else None), dynamic=dyn, tris=tris))
        if tris is not None:
            low = (tris[:, :, 2] <= FLOOR_MAX_Z).all(axis=1) & (tris[:, :, 2] >= -0.1).all(axis=1)
            t = tris[low]
            area = 0.5 * np.abs(np.cross(t[:, 1, :2] - t[:, 0, :2], t[:, 2, :2] - t[:, 0, :2])).sum() if len(t) else 0.0
            if area >= 4.0:
                info.has_floor = True
        elif box.hi[2] <= FLOOR_MAX_Z and box.size[0] * box.size[1] >= 4.0:
            info.has_floor = True

    r = bc_vis.ComputeWorldBound(root).ComputeAlignedRange()
    if not r.IsEmpty():
        info.bounds = Box(tuple(r.GetMin()), tuple(r.GetMax()))
    return info


def report(info: SceneInfo) -> str:
    lines = [f"USD: {info.usd_path}",
             f"  default prim {info.root}, metersPerUnit {info.meters_per_unit}, upAxis {info.up_axis}, bounds {info.bounds}",
             f"  {len(info.colliders)} colliders, {len(info.bodies)} rigid bodies, {len(info.articulations)} articulations, "
             f"{info.lights} lights, floor collider: {'yes' if info.has_floor else 'NO (a ground plane will be added)'}"]
    if info.physics_scenes:
        lines.append(f"  physics scenes (deactivated in the task): {', '.join(info.physics_scenes)}")
    if info.cameras:
        lines.append(f"  cameras (kept, unused): {len(info.cameras)}")
    g = info.graspables()
    lines.append(f"  graspable objects (free rigid body, <= {GRASPABLE_MAX_MASS} kg, <= {GRASPABLE_MAX_SIZE} m): {len(g)}")
    for b in g:
        lines.append(f"    {info.rel(b.path):45s} mass {b.mass if b.mass is not None else '?'} kg  {b.box}")
    heavy = [b for b in info.bodies if not b.graspable and not b.in_articulation and not b.kinematic]
    if heavy:
        lines.append(f"  other free rigid bodies (furniture etc.): {len(heavy)}")
        for b in heavy[:12]:
            lines.append(f"    {info.rel(b.path):45s} mass {b.mass if b.mass is not None else '?'} kg  {b.box}")
    for name, (box, yaw) in info.markers.items():
        lines.append(f"  marker {name}: {box}, yaw {math.degrees(yaw):.0f} deg")
    return "\n".join(lines)
