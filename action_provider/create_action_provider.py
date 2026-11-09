from action_provider.action_provider_dds import DDSActionProvider


from action_provider.action_provider_replay import FileActionProviderReplay

from action_provider.action_provider_wh_dds import DDSRLActionProvider
from pathlib import Path


def create_action_provider(env,args):
    """create action provider based on parameters"""
    task = getattr(args, "task", "")
    for rt, tag in (("g1d", "-G1D-"), ("r1d", "-R1D-")):
        if (tag in task) != (args.robot_type == rt):
            print(f"robot_type '{args.robot_type}' does not match task '{task}': "
                  f"use --robot_type {rt} for {tag.strip('-')} tasks (and only for them)")
            return None
    if args.action_source == "dds" and args.robot_type == "g1d":
        # Unitree G1-D: separate provider, same DDS interface as the G1
        from action_provider.action_provider_g1d_dds import G1DDDSActionProvider
        return G1DDDSActionProvider(env=env, args_cli=args)
    if args.action_source == "dds" and args.robot_type == "r1d":
        from action_provider.action_provider_r1d_dds import R1DDDSActionProvider
        return R1DDDSActionProvider(env=env, args_cli=args)
    if args.action_source == "dds":
        return DDSActionProvider(
            env=env,
            args_cli=args
        )
    elif args.action_source == "dds_wholebody":
        return DDSRLActionProvider(
            env=env,
            args_cli=args
        )
    elif args.action_source == "replay":
        return FileActionProviderReplay(env=env,args_cli=args)
    else:
        print(f"unknown action source: {args.action_source}")
        return None