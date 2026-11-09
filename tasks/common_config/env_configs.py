"""Environment configuration shared by Unitree tasks."""

from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.sim import SimulationCfg
from isaaclab.utils import configclass
from isaaclab_physx.physics import PhysxCfg


@configclass
class UnitreeManagerBasedRLEnvCfg(ManagerBasedRLEnvCfg):
    """Manager-based environment with an explicit Isaac Sim PhysX backend."""

    # Isaac Lab 3.0 changed the default of ``enable_external_forces_every_iteration`` to True
    # (it was False in 2.x). With the CPU TGS pipeline this makes the fixed-base G1 articulation
    # explode within a few physics steps under gravity, so keep the 2.x behaviour.
    sim: SimulationCfg = SimulationCfg(physics=PhysxCfg(enable_external_forces_every_iteration=False))
