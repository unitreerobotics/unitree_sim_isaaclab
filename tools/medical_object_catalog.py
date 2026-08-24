"""Shared catalog and launcher-local role selection for hospital table objects."""

from __future__ import annotations

import json
from dataclasses import dataclass


OBJECT_ROLES_ENV = "HOSPITAL_OBJECT_ROLES"
ROLE_EXCLUDED = "excluded"
ROLE_IMPORTANT = "important"
ROLE_DISTRACTOR = "distractor"
VALID_ROLES = (ROLE_EXCLUDED, ROLE_IMPORTANT, ROLE_DISTRACTOR)

ISAAC_ASSET_ROOT = (
    "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.1/"
    "Isaac/Environments"
)


@dataclass(frozen=True)
class MedicalObjectSpec:
    prim_name: str
    scene_name: str
    asset_url: str
    bbox_half_xy: tuple[float, float]
    center_offset: tuple[float, float, float]
    min_z: float
    mass: float
    default_role: str = ROLE_EXCLUDED


def _asset_url(category: str, filename: str) -> str:
    return f"{ISAAC_ASSET_ROOT}/{category}/Props/{filename}"


MEDICAL_OBJECT_SPECS = (
    MedicalObjectSpec(
        "SM_CoffeeToGo",
        "coffee_cup",
        _asset_url("Office", "SM_CoffeeToGo.usd"),
        (0.043416, 0.042756),
        (0.0, 0.000330, 0.061590),
        0.000017,
        0.08,
    ),
    MedicalObjectSpec(
        "SM_PillBottle_01t",
        "pill_bottle_t",
        _asset_url("Hospital", "SM_PillBottle_01t.usd"),
        (0.014611, 0.014611),
        (0.0, 0.0, 0.025304),
        0.0,
        0.03,
        ROLE_IMPORTANT,
    ),
    MedicalObjectSpec(
        "SM_PillBottle_01v",
        "pill_bottle_v",
        _asset_url("Hospital", "SM_PillBottle_01v.usd"),
        (0.013665, 0.013665),
        (0.0, 0.0, 0.020902),
        0.0,
        0.03,
        ROLE_IMPORTANT,
    ),
    MedicalObjectSpec(
        "SM_Cup_half_full",
        "cup_half_full",
        _asset_url("Office", "SM_Cup_half_full.usd"),
        (0.045000, 0.059930),
        (0.0, 0.0, 0.050000),
        0.0,
        0.08,
    ),
    MedicalObjectSpec(
        "SM_BottleA",
        "medical_bottle_a",
        _asset_url("Hospital", "SM_BottleA.usd"),
        (0.031000, 0.031000),
        (0.0, 0.0, 0.124744),
        0.0,
        0.15,
        ROLE_DISTRACTOR,
    ),
    MedicalObjectSpec(
        "SM_PlasticCup",
        "plastic_cup",
        _asset_url("Office", "SM_PlasticCup.usd"),
        (0.035000, 0.035000),
        (0.0, 0.0, 0.049972),
        -0.000028,
        0.05,
    ),
    MedicalObjectSpec(
        "SM_BottleF",
        "medical_bottle_f",
        _asset_url("Hospital", "SM_BottleF.usd"),
        (0.030050, 0.030050),
        (0.0, 0.0, 0.117414),
        0.0,
        0.15,
        ROLE_DISTRACTOR,
    ),
    MedicalObjectSpec(
        "SM_MarkerBlue",
        "marker_blue",
        _asset_url("Office", "SM_MarkerBlue.usd"),
        (0.058500, 0.012140),
        (0.0, 0.0, 0.004500),
        0.000130,
        0.02,
        ROLE_DISTRACTOR,
    ),
    MedicalObjectSpec(
        "SM_MarkerYellow",
        "marker_yellow",
        _asset_url("Office", "SM_MarkerYellow.usd"),
        (0.058500, 0.012140),
        (0.0, 0.0, 0.004500),
        0.000130,
        0.02,
        ROLE_DISTRACTOR,
    ),
    MedicalObjectSpec(
        "SM_Felt__Pen_Pink",
        "felt_pen_pink",
        _asset_url("Office", "SM_Felt__Pen_Pink.usd"),
        (0.059126, 0.007000),
        (0.0, 0.0, 0.007000),
        0.0,
        0.02,
    ),
)

SPEC_BY_PRIM_NAME = {spec.prim_name: spec for spec in MEDICAL_OBJECT_SPECS}
SPEC_BY_SCENE_NAME = {spec.scene_name: spec for spec in MEDICAL_OBJECT_SPECS}


def default_roles() -> dict[str, str]:
    return {spec.scene_name: spec.default_role for spec in MEDICAL_OBJECT_SPECS}


def parse_roles(raw: str | None) -> dict[str, str]:
    """Parse role JSON, falling back to the pre-GUI six-object task layout."""
    if not raw:
        roles = default_roles()
    else:
        try:
            payload = json.loads(raw)
        except json.JSONDecodeError as exc:
            raise ValueError(f"invalid {OBJECT_ROLES_ENV} JSON: {exc.msg}") from exc
        if not isinstance(payload, dict):
            raise ValueError(f"{OBJECT_ROLES_ENV} must be a JSON object")
        unknown_keys = set(payload).difference((ROLE_IMPORTANT, ROLE_DISTRACTOR))
        if unknown_keys:
            raise ValueError(f"unknown role keys: {sorted(unknown_keys)}")
        roles = {name: ROLE_EXCLUDED for name in SPEC_BY_SCENE_NAME}
        assigned: set[str] = set()
        for role in (ROLE_IMPORTANT, ROLE_DISTRACTOR):
            names = payload.get(role, [])
            if not isinstance(names, list) or not all(isinstance(name, str) for name in names):
                raise ValueError(f"role {role!r} must contain a JSON list of scene names")
            unknown_names = set(names).difference(SPEC_BY_SCENE_NAME)
            if unknown_names:
                raise ValueError(f"unknown medical object names: {sorted(unknown_names)}")
            duplicates = assigned.intersection(names)
            if duplicates:
                raise ValueError(f"objects cannot have multiple roles: {sorted(duplicates)}")
            for name in names:
                roles[name] = role
            assigned.update(names)

    if not any(role == ROLE_IMPORTANT for role in roles.values()):
        raise ValueError("select at least one important medical object")
    return roles


def roles_json(roles: dict[str, str]) -> str:
    """Encode only active roles in stable catalog order for the launcher."""
    payload = {
        role: [
            spec.scene_name
            for spec in MEDICAL_OBJECT_SPECS
            if roles.get(spec.scene_name) == role
        ]
        for role in (ROLE_IMPORTANT, ROLE_DISTRACTOR)
    }
    return json.dumps(payload, separators=(",", ":"))
