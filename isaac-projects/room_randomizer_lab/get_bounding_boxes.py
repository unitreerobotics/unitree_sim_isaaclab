import sys
import math
from pxr import Usd, UsdGeom

try:
    from room_randomizer_lab.constants import ASSET_PATHS
except ImportError:
    # Fallback to local import if run directly in the folder
    from constants import ASSET_PATHS

def compute_asset_bounds():
    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ['default', 'proxy', 'render'])

    print(f"{'Asset Name':<25} | {'Width (X)':<10} | {'Depth (Y)':<10} | {'Height (Z)':<10} | {'half_w':<8} | {'half_d':<8}")
    print("-" * 85)

    for name, path in ASSET_PATHS.items():
        stage = Usd.Stage.Open(path)
        if not stage:
            print(f"{name:<25} | Failed to open: {path}")
            continue

        default_prim = stage.GetDefaultPrim()
        if not default_prim:
            # Fallback to first prim under pseudo root
            children = list(stage.GetPseudoRoot().GetChildren())
            if children:
                default_prim = children[0]
            else:
                print(f"{name:<25} | No prims found")
                continue

        # Compute bounding box
        bound = bbox_cache.ComputeWorldBound(default_prim)
        b_range = bound.ComputeAlignedRange()

        min_pt = b_range.GetMin()
        max_pt = b_range.GetMax()

        width = max_pt[0] - min_pt[0]
        depth = max_pt[1] - min_pt[1]
        height = max_pt[2] - min_pt[2]

        half_w = width / 2.0
        half_d = depth / 2.0

        print(f"{name:<25} | {width:<10.3f} | {depth:<10.3f} | {height:<10.3f} | {half_w:<8.3f} | {half_d:<8.3f}")

if __name__ == '__main__':
    compute_asset_bounds()
