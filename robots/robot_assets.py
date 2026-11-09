# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""
Unified robot asset loading: a robot can be described by a USD file or a URDF file.

``robot_spawn_cfg(path, ...)`` returns the matching Isaac Lab spawner config, so robot configs
(``robots/unitree.py``) and everything above them do not care which format the asset is in:

* ``.usd / .usda / .usdc`` -> :class:`isaaclab.sim.UsdFileCfg`
* ``.urdf``                -> :class:`isaaclab.sim.UrdfFileCfg`
  The URDF is first normalized (mesh paths resolved to absolute files, see below) into
  ``assets/robots/.urdf_cache/<name>/`` and converted to USD there on first use. Isaac Lab's
  converter caches by content hash, so later runs reuse the generated USD and a modified
  URDF / mesh / config is re-converted automatically.

Inline URDF materials that share a name but not a color get unique names (see _unique_material_names).

Mesh path normalization handles ``package://<pkg>/...``, relative paths and meshes that are
not where the URDF says (e.g. ``meshes/x.STL`` while the file is in ``meshes/meshes/x.STL``):
a mesh is looked up by its path first and by its file name anywhere below the URDF folder second.
"""

from __future__ import annotations

import hashlib
import os
import xml.etree.ElementTree as ET

import isaaclab.sim as sim_utils

PROJECT_ROOT = os.environ.get("PROJECT_ROOT") or os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
URDF_CACHE_DIR = os.path.join(PROJECT_ROOT, "assets", "robots", ".urdf_cache")

USD_SUFFIXES = (".usd", ".usda", ".usdc")


def _resolve_mesh(filename: str, urdf_dir: str, by_name: dict[str, str]) -> str:
    path = filename
    if path.startswith("package://"):
        path = path[len("package://"):].split("/", 1)[-1]  # drop the package name
    elif path.startswith("file://"):
        path = path[len("file://"):]
    candidates = [path] if os.path.isabs(path) else [os.path.join(urdf_dir, path)]
    for c in candidates:
        if os.path.isfile(c):
            return os.path.abspath(c)
    hit = by_name.get(os.path.basename(path).lower())
    if hit is None:
        raise FileNotFoundError(f"URDF mesh '{filename}' not found below {urdf_dir}")
    return hit


def _unique_material_names(root) -> None:
    """Give every inline material color its own name.

    URDF materials are global by name: exported URDFs often reuse one name (or "") for different
    inline colors - e.g. "silvery" for the light-grey arms and the black Dex1 gripper - and the importer
    then uses the first definition for all of them. Inline materials are renamed after their color, so
    equal colors still share one material.
    """
    for mat in root.iter("material"):
        color = mat.find("color")
        if color is None or not color.get("rgba"):
            continue  # reference to a named (top-level) material, or a texture
        rgba = [float(v) for v in color.get("rgba").split()]
        base = (mat.get("name") or "material").replace(" ", "_")
        mat.set("name", f"{base}_" + "_".join(f"{int(round(v * 1000)):04d}" for v in rgba))


def _override_link_colors(root, link_colors: dict[str, tuple] | None) -> None:
    """Set the visual color of links whose name fully matches a regex key of ``link_colors`` (rgba)."""
    if not link_colors:
        return
    import re

    rules = [(re.compile(k), v) for k, v in link_colors.items()]
    for link in root.findall("link"):
        rgba = next((v for rx, v in rules if rx.fullmatch(link.get("name", ""))), None)
        if rgba is None:
            continue
        for visual in link.findall("visual"):
            mat = visual.find("material")
            if mat is None:
                mat = ET.SubElement(visual, "material", {"name": ""})
            for child in list(mat):
                mat.remove(child)
            ET.SubElement(mat, "color", {"rgba": " ".join(f"{float(c):g}" for c in rgba)})


def normalize_urdf(urdf_path: str, cache_dir: str | None = None, link_colors: dict[str, tuple] | None = None) -> str:
    """Write a copy of ``urdf_path`` with absolute, existing mesh paths (and optional link color
    overrides, ``{link-name regex: (r, g, b, a)}``) and return its path."""
    urdf_path = os.path.abspath(urdf_path)
    urdf_dir = os.path.dirname(urdf_path)
    name = os.path.splitext(os.path.basename(urdf_path))[0]
    cache_dir = cache_dir or os.path.join(URDF_CACHE_DIR, name)

    by_name: dict[str, str] = {}
    for root, _, files in os.walk(urdf_dir):
        for f in files:
            by_name.setdefault(f.lower(), os.path.join(root, f))

    tree = ET.parse(urdf_path)
    for mesh in tree.getroot().iter("mesh"):
        mesh.set("filename", _resolve_mesh(mesh.get("filename", ""), urdf_dir, by_name))
    _override_link_colors(tree.getroot(), link_colors)
    _unique_material_names(tree.getroot())
    content = ET.tostring(tree.getroot(), encoding="unicode")

    os.makedirs(cache_dir, exist_ok=True)
    out = os.path.join(cache_dir, f"{name}.urdf")
    old = open(out, encoding="utf-8").read() if os.path.isfile(out) else None
    if old != content:  # keep mtime stable when nothing changed
        with open(out, "w", encoding="utf-8") as f:
            f.write(content)
    return out


def robot_spawn_cfg(
    asset_path: str,
    *,
    rigid_props: sim_utils.RigidBodyPropertiesCfg | None = None,
    articulation_props: sim_utils.ArticulationRootPropertiesCfg | None = None,
    activate_contact_sensors: bool = True,
    fix_base: bool = True,
    merge_fixed_joints: bool = False,
    force_usd_conversion: bool = False,
    link_colors: dict[str, tuple] | None = None,
):
    """Spawner config for a robot given as USD or URDF (see module docstring).

    Args:
        asset_path: ``.usd/.usda/.usdc`` or ``.urdf`` file.
        fix_base: URDF only - add a fixed joint between the world and the root link.
            (For USD this is a property of the asset itself.)
        merge_fixed_joints: URDF only - keep False so that sensor / camera links
            (head_link, *_wrist_yaw_link, ...) remain separate prims cameras can attach to.
        force_usd_conversion: URDF only - re-convert even if the cache is up to date.
        link_colors: URDF only - ``{link-name regex: (r, g, b, a)}`` visual color overrides (sRGB).
    """
    common = dict(
        activate_contact_sensors=activate_contact_sensors,
        rigid_props=rigid_props,
        articulation_props=articulation_props,
    )
    suffix = os.path.splitext(asset_path)[1].lower()
    if suffix in USD_SUFFIXES:
        return sim_utils.UsdFileCfg(usd_path=asset_path, **common)
    if suffix == ".urdf":
        if not os.path.isfile(asset_path):
            raise FileNotFoundError(f"Robot URDF not found: {asset_path}")
        fixed = normalize_urdf(asset_path, link_colors=link_colors)
        cache_dir = os.path.dirname(fixed)
        return sim_utils.UrdfFileCfg(
            asset_path=fixed,
            usd_dir=cache_dir,
            usd_file_name=os.path.splitext(os.path.basename(fixed))[0] + ".usd",
            force_usd_conversion=force_usd_conversion,
            fix_base=fix_base,
            merge_fixed_joints=merge_fixed_joints,
            # gains are set per joint group by the actuator configs of the ArticulationCfg
            joint_drive=sim_utils.UrdfConverterCfg.JointDriveCfg(
                drive_type="force",
                target_type="position",
                gains=sim_utils.UrdfConverterCfg.JointDriveCfg.PDGainsCfg(stiffness=100.0, damping=1.0),
            ),
            **common,
        )
    raise ValueError(f"Unsupported robot asset type '{suffix}' ({asset_path}); expected .usd/.usda/.usdc or .urdf")


def _urdf_link_chain(urdf_path: str, link: str) -> list[str]:
    root = ET.parse(urdf_path).getroot()
    parent_of = {j.find("child").get("link"): j.find("parent").get("link") for j in root.findall("joint")}
    links = {l.get("name") for l in root.findall("link")}
    if link not in links:
        raise KeyError(f"link '{link}' not found in {urdf_path}")
    chain = [link]
    while chain[-1] in parent_of:
        chain.append(parent_of[chain[-1]])
    return chain[::-1]


def robot_link_prim_path(asset_path: str, link: str) -> str:
    """Path of a robot link prim relative to the robot prim (e.g. ``"head_link"``).

    Use it to attach sensors to a link independent of how the asset is laid out:

    * URDF: the Isaac Sim 6 URDF importer nests every link under its parent link below a
      ``Geometry`` scope, e.g. ``Geometry/AGV_link/.../torso_link/head_link``.
    * USD: the prim named ``link`` is searched in the file (flat G1 USDs give just ``link``).
    """
    suffix = os.path.splitext(asset_path)[1].lower()
    if suffix == ".urdf":
        return "/".join(["Geometry", *_urdf_link_chain(asset_path, link)])
    if suffix in USD_SUFFIXES:
        from pxr import Usd

        stage = Usd.Stage.Open(asset_path)
        top = stage.GetDefaultPrim()
        for prim in Usd.PrimRange(top):
            if prim.GetName() == link:
                return str(prim.GetPath().MakeRelativePath(top.GetPath()))
        raise KeyError(f"link '{link}' not found in {asset_path}")
    raise ValueError(f"Unsupported robot asset type '{suffix}' ({asset_path})")


def apply_material_finish(env, env_ids, asset_name: str = "robot", metal=(0.85, 0.28), dark=(0.45, 0.4), black=(0.0, 0.5)):
    """Event (mode "startup"): give the robot's URDF-converted materials a metal / plastic finish.

    The URDF importer only authors a diffuse color (metallic 0, roughness 0.5). The robot meshes are
    instanceable and all reference the converted ``materials.usda`` layer, so that layer is edited in
    memory (the file on disk is not changed). The finish (metallic, roughness) is chosen per material
    from its diffuse color (linear): light -> ``metal``, dark grey -> ``dark``, near black -> ``black``.
    """
    import omni.usd
    from pxr import Sdf

    stage = omni.usd.get_context().get_stage()
    root = env.scene[asset_name].cfg.spawn
    usd_dir = getattr(root, "usd_dir", None)
    if not usd_dir:
        return
    usd_dir = os.path.abspath(usd_dir)
    edited = 0
    for layer in stage.GetUsedLayers():
        path = os.path.abspath(layer.realPath or "")
        if not (path.startswith(usd_dir) and path.endswith("materials.usda")):
            continue
        scope = layer.GetPrimAtPath("/Materials")
        for spec in scope.nameChildren if scope else []:
            diffuse = spec.attributes.get("inputs:diffuseColor")
            if diffuse is None or diffuse.default is None:
                continue
            lum = sum(float(c) for c in diffuse.default) / 3.0
            metallic, roughness = metal if lum >= 0.3 else (black if lum <= 0.005 else dark)
            for name, value in (("inputs:metallic", metallic), ("inputs:roughness", roughness)):
                attr = spec.attributes.get(name) or Sdf.AttributeSpec(spec, name, Sdf.ValueTypeNames.Float)
                attr.default = float(value)
            edited += 1
    print(f"[robot_assets] material finish applied to {edited} materials of '{asset_name}'")
