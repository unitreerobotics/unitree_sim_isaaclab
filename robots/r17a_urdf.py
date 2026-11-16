# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""Build the R1-7A arm robots (arm(s) on a mount link) from the single-arm URDFs. All robots are one left arm.

Source: assets/urdf/r1_7a_description (unitree_ros). Each source URDF is one arm (``base_link``, ``Link1..7``,
joints ``J1..J7``) with an end effector. The generated robot has a root link ``mount_link`` (x forward, y left,
z up; it is placed on the table or on a column by the task) and per arm ``<side>_arm_base_link`` fixed to it:

* arm links / joints get the side prefix: ``<side>_Link1..7``, ``<side>_J1..J7``
* Dex1: ``<side>_dex1_base_link``, ``<side>_dex1_finger_joint_1/2``, ``<side>_dex1_camera_link`` (same names as
  the G1-D / R1-A7-D Dex1 grippers)
* Dex2-5 (``<side>_*`` already) and LinkerHand O6 (``lh_*`` / ``rh_*``) keep their names
* the Dex2-5 hand links are colored black (like the R1-A7-D Dex2-5 hands)
* zero joint limits of the sources are filled in (an effort / velocity limit of 0 would block the joint): the Dex1
  fingers get the limits of the same gripper on the R1-A7-D (20 N, 0.2 m/s), the LinkerHand distal (mimic) joints
  the effort limit of the joint driving them

Mesh paths are written absolute; the file is only rewritten when its content changes (the URDF -> USD
conversion cache is keyed on it). No simulator imports.
"""

from __future__ import annotations

import math
import os
import re
import xml.etree.ElementTree as ET

PROJECT_ROOT = os.environ.get("PROJECT_ROOT") or os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
R17A_SRC_DIR = os.path.join(PROJECT_ROOT, "assets", "urdf", "r1_7a_description")
R17A_BUILD_DIR = os.path.join(PROJECT_ROOT, "assets", "robots", ".urdf_cache", "_r17a_src")

R17A_SOURCES = {
    ("dex1", "left"): "R1_7a_dex1.urdf",
    ("dex2_5", "left"): "R1_7a_Dex2_5_left.urdf",
    ("dex2_5", "right"): "R1_7a_Dex2_5_right.urdf",
    ("o6", "left"): "R1_7a_linkerhandO6_left.urdf",
}
_ARM = re.compile(r"^(base_link|Link[1-7]|J[1-7])$")
_DEX1 = {"Dex1_base_link": "dex1_base_link", "Dex1_finger_link_1": "dex1_finger_link_1", "Dex1_finger_link_2": "dex1_finger_link_2",
         "Dex1_finger_joint_1": "dex1_finger_joint_1", "Dex1_finger_joint_2": "dex1_finger_joint_2", "Dex1_mount": "dex1_mount",
         "Camera": "dex1_camera_link"}
_BLACK = "0.03 0.03 0.03 1"
_DEX1_FINGER_LIMITS = {"effort": "20", "velocity": "0.2"}  # as in r1_d_with_dex1_1.urdf


def _rename(name: str, side: str, is_joint: bool) -> str:
    if name == "base_link":
        return f"{side}_arm_base_link"
    if _ARM.match(name):
        return f"{side}_{name}"
    if name in _DEX1:
        n = _DEX1[name]
        return f"{side}_{'dex1_camera_joint' if (is_joint and name == 'Camera') else n}"
    return name  # Dex2-5 (<side>_*) and LinkerHand (lh_* / rh_*) names are prefixed already


def _arm_elements(hand: str, side: str):
    path = os.path.join(R17A_SRC_DIR, R17A_SOURCES[(hand, side)])
    src_dir = os.path.dirname(path)
    root = ET.parse(path).getroot()
    out = []
    for el in root:
        if el.tag == "link":
            el.set("name", _rename(el.get("name"), side, False))
            for mesh in el.iter("mesh"):
                f = mesh.get("filename", "")
                f = f.split("://", 1)[-1].split("/", 1)[-1] if "://" in f else f
                mesh.set("filename", os.path.abspath(os.path.join(src_dir, f)))
            if hand == "dex2_5" and re.match(rf"^{side}_[0-5]+_link$", el.get("name")):
                for color in el.iter("color"):
                    color.set("rgba", _BLACK)
        elif el.tag == "joint":
            el.set("name", _rename(el.get("name"), side, True))
            for tag in ("parent", "child"):
                el.find(tag).set("link", _rename(el.find(tag).get("link"), side, False))
            mimic = el.find("mimic")
            if mimic is not None:
                mimic.set("joint", _rename(mimic.get("joint"), side, True))
        out.append(el)
    _fill_zero_limits(out)
    return out


def _fill_zero_limits(elements) -> None:
    joints = {el.get("name"): el for el in elements if el.tag == "joint"}
    for name, j in joints.items():
        lim = j.find("limit")
        if lim is None or j.get("type") == "fixed":
            continue
        if "dex1_finger_joint" in name:
            for k, v in _DEX1_FINGER_LIMITS.items():
                if float(lim.get(k) or 0) <= 0:
                    lim.set(k, v)
        mimic = j.find("mimic")
        if mimic is not None and float(lim.get("effort") or 0) <= 0:
            src = joints.get(mimic.get("joint"))
            src_lim = src.find("limit") if src is not None else None
            if src_lim is not None:
                lim.set("effort", src_lim.get("effort"))


def build_r17a_urdf(name: str, hand: str, arms: dict[str, tuple]) -> str:
    """Write ``<R17A_BUILD_DIR>/<name>.urdf`` and return its path.

    ``arms``: side -> (x, y, z, yaw) of ``<side>_arm_base_link`` in ``mount_link``.
    """
    robot = ET.Element("robot", {"name": name})
    mount = ET.SubElement(robot, "link", {"name": "mount_link"})
    inertial = ET.SubElement(mount, "inertial")
    ET.SubElement(inertial, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
    ET.SubElement(inertial, "mass", {"value": "1.0"})
    ET.SubElement(inertial, "inertia", {"ixx": "0.001", "ixy": "0", "ixz": "0", "iyy": "0.001", "iyz": "0", "izz": "0.001"})
    for side, (x, y, z, yaw) in arms.items():
        robot.extend(_arm_elements(hand, side))
        j = ET.SubElement(robot, "joint", {"name": f"{side}_arm_mount_joint", "type": "fixed"})
        ET.SubElement(j, "origin", {"xyz": f"{x:g} {y:g} {z:g}", "rpy": f"0 0 {yaw:g}"})
        ET.SubElement(j, "parent", {"link": "mount_link"})
        ET.SubElement(j, "child", {"link": f"{side}_arm_base_link"})
    ET.indent(robot)
    content = ET.tostring(robot, encoding="unicode") + "\n"
    os.makedirs(R17A_BUILD_DIR, exist_ok=True)
    out = os.path.join(R17A_BUILD_DIR, f"{name}.urdf")
    old = open(out, encoding="utf-8").read() if os.path.isfile(out) else None
    if old != content:
        with open(out, "w", encoding="utf-8") as f:
            f.write(content)
    return out


# ---- robots (mount_link: x forward, y left, z up)
R17A_ARM_YAW = -math.pi / 2  # arm base yaw on mount_link
R17A_ARMS = {
    "dex1": {"left": (0.0, 0.0)},
    "dex2_5": {"left": (0.0, 0.0)},
    "o6": {"left": (0.0, 0.0)},
}

# default arm pose (inverse kinematics): tool point 0.30 m in front of / 0.15 m above the arm base, fingers
# pointing forward and 30 deg down, palm (Dex2-5, O6) facing down, Dex1 fingers opening sideways
R17A_HOME = {
    "dex1": {"left": (-0.169, 2.006, 2.983, -0.560, 1.631, -0.056, -0.039)},
    "dex2_5": {"left": (-0.299, 1.737, 2.852, -0.845, -1.521, 0.036, 0.025)},
    "o6": {"left": (-0.151, 1.844, 2.999, -0.758, -1.528, 0.031, 0.001)},
}
# tool point per hand (link, offset in that link): between the Dex1 fingers / in the palm of Dex2-5 and O6
R17A_TCP = {
    "dex1": {"left": ("left_dex1_base_link", (0.11, 0.0, 0.0))},
    "dex2_5": {"left": ("left_0_link", (0.05, -0.037, 0.0))},
    "o6": {"left": ("lh_hand_base_link", (0.03, 0.0, 0.09))},
}


def r17a_arm_joints(side: str) -> list[str]:
    return [f"{side}_J{i}" for i in range(1, 8)]


def r17a_asset(hand: str) -> str:
    """Build (if needed) and return the URDF of the ``hand`` robot ("dex1", "dex2_5", "o6")."""
    arms = {side: (x, y, 0.0, R17A_ARM_YAW) for side, (x, y) in R17A_ARMS[hand].items()}
    return build_r17a_urdf(f"r1_7a_{hand}", hand, arms)
