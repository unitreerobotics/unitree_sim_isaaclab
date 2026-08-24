"""Live viewport overlay for the room randomizer's 2D oriented boxes."""

from __future__ import annotations

from isaacsim.util.debug_draw import _debug_draw

from .constants import OBB_PLACEMENT_MARGIN
from .placement_utils import obb_corners


_CATEGORY_COLORS = {
    "wall": [0.10, 0.55, 1.00, 1.00],
    "static_wall": [1.00, 0.10, 0.10, 1.00],
    "table_group": [0.10, 1.00, 0.20, 1.00],
    "robot": [0.75, 0.20, 1.00, 1.00],
    "table_reserved": [1.00, 1.00, 1.00, 1.00],
    "tabletop": [1.00, 0.45, 0.05, 1.00],
}
_MARGIN_COLOR = [1.00, 0.90, 0.05, 0.55]


class ObbVisualizer:
    """Draw accepted collision footprints in the Isaac Sim viewport."""

    def __init__(self, env, show_margins: bool = False):
        self._env = env
        self._show_margins = show_margins
        self._draw = _debug_draw.acquire_debug_draw_interface()

    @staticmethod
    def _append_rectangle(
        box,
        origin,
        z,
        color,
        thickness,
        start_points,
        end_points,
        colors,
        thicknesses,
    ) -> None:
        corners = obb_corners(*box)
        points = [
            [x + origin[0], y + origin[1], z + origin[2]]
            for x, y in corners
        ]
        for index in range(4):
            start_points.append(points[index])
            end_points.append(points[(index + 1) % 4])
            colors.append(color)
            thicknesses.append(thickness)

    def draw(self) -> int:
        """Clear the previous overlay and draw the latest accepted OBBs."""
        self._draw.clear_lines()
        records_by_env = getattr(self._env, "_room_randomizer_debug_obbs", {})

        start_points = []
        end_points = []
        colors = []
        thicknesses = []
        box_count = 0

        for env_id, records in records_by_env.items():
            origin = self._env.scene.env_origins[env_id].detach().cpu().tolist()
            for record in records:
                box = record["box"]
                color = _CATEGORY_COLORS.get(record["category"], [1.0, 1.0, 1.0, 1.0])
                self._append_rectangle(
                    box,
                    origin,
                    record["z"],
                    color,
                    5.0,
                    start_points,
                    end_points,
                    colors,
                    thicknesses,
                )
                if self._show_margins and record["category"] != "tabletop":
                    inflated_box = (
                        box[0],
                        box[1],
                        box[2] + OBB_PLACEMENT_MARGIN,
                        box[3] + OBB_PLACEMENT_MARGIN,
                        box[4],
                    )
                    self._append_rectangle(
                        inflated_box,
                        origin,
                        record["z"] + 0.01,
                        _MARGIN_COLOR,
                        2.0,
                        start_points,
                        end_points,
                        colors,
                        thicknesses,
                    )
                box_count += 1

        if start_points:
            self._draw.draw_lines(start_points, end_points, colors, thicknesses)
        print(f"[INFO] Drew {box_count} room-randomizer OBB footprints.", flush=True)
        return box_count

    def clear(self) -> None:
        """Remove the overlay from the viewport."""
        self._draw.clear_lines()
