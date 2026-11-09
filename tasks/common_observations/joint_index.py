# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""
Name-based joint lookup for the observation / DDS state functions.

The state functions publish joints in a fixed (DDS motor) order given by joint *names*. The
simulator's joint order depends on how the asset was built (USD vs URDF import, robot variant),
so indices are resolved from the names once per robot. Joints the robot does not have (e.g. the
legs of the wheeled G1-D) are read as 0 via an extra zero column. Used by the G1-D state functions.
"""

from __future__ import annotations

import torch

class JointGather:
    """Gathers a fixed list of joints (by name) from ``[batch, num_joints]`` tensors."""

    def __init__(self, sim_joint_names: list[str], wanted: list[str], device):
        index = {n: i for i, n in enumerate(sim_joint_names)}
        num = len(sim_joint_names)
        ids, self.missing = [], []
        for name in wanted:
            i = index.get(name)
            if i is None:
                self.missing.append(name)
                i = num  # zero pad column
            ids.append(i)
        self.indices = ids
        self.pad = bool(self.missing)
        self.idx = torch.tensor(ids, dtype=torch.long, device=device)
        self.n = len(ids)
        self.device = device

    def gather(self, src: torch.Tensor, out: torch.Tensor | None = None) -> torch.Tensor:
        if self.pad:
            src = torch.nn.functional.pad(src, (0, 1))
        idx = self.idx.unsqueeze(0).expand(src.shape[0], self.n)
        if out is not None:
            torch.gather(src, 1, idx, out=out)
            return out
        return torch.gather(src, 1, idx)

