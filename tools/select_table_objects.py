#!/usr/bin/env python3
"""Assign important, distractor, or excluded roles to MedicalObjects assets."""

from __future__ import annotations

import argparse
import re
import sys
from dataclasses import dataclass
from pathlib import Path

try:
    from tools.medical_object_catalog import (
        ROLE_DISTRACTOR,
        ROLE_EXCLUDED,
        ROLE_IMPORTANT,
        SPEC_BY_PRIM_NAME,
        default_roles,
        roles_json,
    )
except ModuleNotFoundError:
    # Direct execution adds tools/, rather than the repository root, to
    # sys.path when the launcher is invoked from another working directory.
    from medical_object_catalog import (
        ROLE_DISTRACTOR,
        ROLE_EXCLUDED,
        ROLE_IMPORTANT,
        SPEC_BY_PRIM_NAME,
        default_roles,
        roles_json,
    )


MEDICAL_OBJECTS_SCOPE = "MedicalObjects"


@dataclass(frozen=True)
class TableObject:
    """One supported direct child of the authored MedicalObjects scope."""

    prim_name: str
    scene_name: str
    asset_name: str
    asset_url: str
    default_role: str

    @property
    def label(self) -> str:
        source = self.prim_name.removeprefix("SM_").replace("__", "_")
        words = re.sub(r"(?<!^)(?=[A-Z])", " ", source).replace("_", " ")
        return f"{words}  ({self.asset_name})"


def _brace_delta(line: str) -> int:
    without_comment = line.split("#", 1)[0]
    without_strings = re.sub(r'"(?:[^"\\]|\\.)*"', '""', without_comment)
    return without_strings.count("{") - without_strings.count("}")


def medical_objects_from_usda(path: Path) -> list[TableObject]:
    """Read every direct asset child of ``/World/MedicalObjects``."""
    lines = path.read_text(encoding="utf-8").splitlines()
    prim_pattern = re.compile(r'^\s*def(?:\s+\w+)?\s+"([^"]+)"')
    payload_pattern = re.compile(r"(?:payload|reference)\s*=\s*@([^@]+\.usd)@")

    depth = 0
    scope_body_depth: int | None = None
    scope_header_pending = False
    scope_found = False
    discovered: list[TableObject] = []

    for line_number, line in enumerate(lines):
        match = prim_pattern.match(line)
        if match:
            prim_name = match.group(1)
            if prim_name == MEDICAL_OBJECTS_SCOPE and not scope_found:
                scope_header_pending = True
                scope_found = True
            elif scope_body_depth is not None and depth == scope_body_depth:
                declaration = "\n".join(lines[line_number : line_number + 5])
                payload = payload_pattern.search(declaration)
                if payload is None:
                    raise ValueError(
                        f"MedicalObjects child {prim_name!r} has no USD payload/reference"
                    )
                spec = SPEC_BY_PRIM_NAME.get(prim_name)
                if spec is None:
                    raise ValueError(
                        f"MedicalObjects child {prim_name!r} needs physics metadata in "
                        "tools/medical_object_catalog.py"
                    )
                asset_url = payload.group(1)
                if asset_url != spec.asset_url:
                    raise ValueError(
                        f"catalog URL mismatch for {prim_name}: {asset_url!r} != {spec.asset_url!r}"
                    )
                discovered.append(
                    TableObject(
                        prim_name=prim_name,
                        scene_name=spec.scene_name,
                        asset_name=Path(asset_url).name,
                        asset_url=asset_url,
                        default_role=spec.default_role,
                    )
                )

        next_depth = depth + _brace_delta(line)
        if scope_header_pending and next_depth > depth:
            scope_body_depth = next_depth
            scope_header_pending = False
        elif scope_body_depth is not None and next_depth < scope_body_depth:
            scope_body_depth = None
        depth = next_depth

    if not scope_found:
        raise ValueError(f'authored scope "{MEDICAL_OBJECTS_SCOPE}" was not found in {path}')
    if not discovered:
        raise ValueError(f"no USD assets were found beneath {MEDICAL_OBJECTS_SCOPE} in {path}")

    discovered_names = {item.prim_name for item in discovered}
    missing = set(SPEC_BY_PRIM_NAME).difference(discovered_names)
    if missing:
        raise ValueError(f"catalog objects missing from MedicalObjects: {sorted(missing)}")
    return discovered


def show_selector(objects: list[TableObject]) -> dict[str, str] | None:
    """Show the modal role selector; return None when launch is cancelled."""
    import tkinter as tk
    from tkinter import messagebox, ttk

    root = tk.Tk()
    root.title("Hospital medical objects")
    root.resizable(False, False)

    result: dict[str, str] | None = None
    initial_roles = default_roles()
    variables = {
        item.scene_name: tk.StringVar(value=initial_roles[item.scene_name])
        for item in objects
    }

    frame = ttk.Frame(root, padding=18)
    frame.grid(sticky="nsew")
    ttk.Label(
        frame,
        text="Choose table objects and their task roles",
        font=("TkDefaultFont", 11, "bold"),
    ).grid(row=0, column=0, columnspan=4, sticky="w", pady=(0, 4))
    ttk.Label(
        frame,
        text="Important objects must be placed in the Ridgeback crate.\n"
        "Distractors appear on the table but do not affect task success.",
        justify="left",
    ).grid(row=1, column=0, columnspan=4, sticky="w", pady=(0, 12))

    headers = ("MedicalObjects asset", "Excluded", "Important", "Distractor")
    for column, heading in enumerate(headers):
        ttk.Label(frame, text=heading).grid(row=2, column=column, padx=6, sticky="w")

    for row, item in enumerate(objects, start=3):
        ttk.Label(frame, text=item.label).grid(row=row, column=0, sticky="w", pady=2)
        for column, role in enumerate(
            (ROLE_EXCLUDED, ROLE_IMPORTANT, ROLE_DISTRACTOR), start=1
        ):
            ttk.Radiobutton(
                frame,
                variable=variables[item.scene_name],
                value=role,
            ).grid(row=row, column=column, padx=12)

    button_row = ttk.Frame(frame)
    button_row.grid(row=len(objects) + 3, column=0, columnspan=4, sticky="e", pady=(16, 0))

    def restore_defaults() -> None:
        roles = default_roles()
        for name, variable in variables.items():
            variable.set(roles[name])

    def clear_all() -> None:
        for variable in variables.values():
            variable.set(ROLE_EXCLUDED)

    def start() -> None:
        nonlocal result
        selected = {name: variable.get() for name, variable in variables.items()}
        if ROLE_IMPORTANT not in selected.values():
            messagebox.showerror(
                "Important object required",
                "Select at least one object as Important before starting.",
                parent=root,
            )
            return
        result = selected
        root.destroy()

    def cancel() -> None:
        root.destroy()

    ttk.Button(button_row, text="Restore defaults", command=restore_defaults).grid(
        row=0, column=0, padx=(0, 8)
    )
    ttk.Button(button_row, text="Clear all", command=clear_all).grid(
        row=0, column=1, padx=(0, 8)
    )
    ttk.Button(button_row, text="Cancel", command=cancel).grid(
        row=0, column=2, padx=(0, 8)
    )
    ttk.Button(button_row, text="Start", command=start).grid(row=0, column=3)

    root.protocol("WM_DELETE_WINDOW", cancel)
    root.bind("<Escape>", lambda _event: cancel())
    root.update_idletasks()
    root.geometry(
        f"+{max(0, (root.winfo_screenwidth() - root.winfo_width()) // 2)}+"
        f"{max(0, (root.winfo_screenheight() - root.winfo_height()) // 2)}"
    )
    root.mainloop()
    return result


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--usd", type=Path, required=True, help="USDA containing MedicalObjects")
    parser.add_argument(
        "--list",
        action="store_true",
        help="print scene name, authored prim, and default role without opening the GUI",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        objects = medical_objects_from_usda(args.usd)
        if args.list:
            for item in objects:
                print(f"{item.scene_name}\t{item.prim_name}\t{item.default_role}")
            return 0
        selected = show_selector(objects)
    except (OSError, ValueError, RuntimeError) as exc:
        print(f"Medical-object selector error: {exc}", file=sys.stderr)
        return 1

    if selected is None:
        print("Launch cancelled in the medical-object selector.", file=sys.stderr)
        return 130
    print(roles_json(selected))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
