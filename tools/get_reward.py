#
# 实用函数：如何获取奖励值
##
import torch

def get_step_reward_value(env) -> torch.Tensor:
    """
    快速获取当前环境的总奖励值
    
    Args:
        env: ManagerBasedRLEnv 环境实例
        
    Returns:
        torch.Tensor: 当前的总奖励值，如果失败返回零张量
    """
    try:
        if hasattr(env, 'reward_manager'):
            return env.reward_manager.get_active_iterable_terms(0)[0][1][0]#.compute(dt=dt)
        else:
            return torch.zeros(env.num_envs, device=str(env.device))
    except Exception as e:
        print(f"获取奖励值时出错: {e}")
        return torch.zeros(env.num_envs, device=str(env.device))


def get_current_rewards(env) -> dict:
    """
    获取当前环境的奖励值
    
    Args:
        env: ManagerBasedRLEnv 环境实例
        
    Returns:
        dict: 包含奖励信息的字典
    """
    
    try:
        # 方法1: 通过奖励管理器计算当前奖励
        if hasattr(env, 'reward_manager'):
            # Isaac Lab 的奖励管理器需要 dt 参数
            dt = env.physics_dt if hasattr(env, 'physics_dt') else 0.02  # 默认使用 0.02 秒
            current_rewards = env.reward_manager.compute(dt=dt)
            return current_rewards
            
            # print(f"current_rewards: {current_rewards}")
            
    except Exception as e:
        print(f"获取奖励值时出错: {e}")
        return torch.zeros(env.num_envs, device=str(env.device))


