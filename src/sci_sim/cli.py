import argparse
from dataclasses import replace
from importlib.resources import files
import json
from pathlib import Path
import sys

from .config import LabConfig, ROBOTS, TASK_IDS


def make_env(task, config):
    import gymnasium as gym
    from . import env as _env  # noqa: F401
    return gym.make(TASK_IDS[task], lab_config=config.to_dict(), render_mode="rgb_array",
                    max_episode_steps=config.max_steps)


def main(argv=None):
    parser = argparse.ArgumentParser(description="Sci-Sim laboratory benchmark (Linux)")
    commands = parser.add_subparsers(dest="command", required=True)
    commands.add_parser("catalog", help="List tasks, robots, instruments and source URLs")
    assets = commands.add_parser("prepare-assets", help="Download and assemble official arm assets")
    assets.add_argument("--robots", nargs="+", choices=ROBOTS, default=list(ROBOTS))
    instruments = commands.add_parser('import-instruments', help='Reconstruct local AtomBridge meshes')
    instruments.add_argument('--source', type=Path, default=Path('../atombridge'))
    for command in ("view", "render", "collect", "demo"):
        p = commands.add_parser(command)
        p.add_argument("--task", choices=TASK_IDS, default="mix_transfer")
        p.add_argument("--robot", choices=ROBOTS, default="ur7e")
        p.add_argument("--width", type=int, default=640)
        p.add_argument("--height", type=int, default=480)
        p.add_argument("--seed", type=int, default=0)
        p.add_argument("--no-randomize", action="store_true")
        p.add_argument('--instruments', choices=('atombridge', 'procedural'), default='atombridge')
        if command not in ('collect', 'demo'):
            p.add_argument("--layout", choices=("single", "lab"), default="lab")
        if command == 'demo':
            p.add_argument('--render', action='store_true', help='Show the live SAPIEN viewer')
        if command == "render":
            p.add_argument("--output", type=Path, default=Path("outputs/overview.png"))
        if command == "collect":
            p.add_argument("--root", type=Path, required=True)
            p.add_argument("--repo-id", default="local/sci-sim")
            p.add_argument("--episodes", type=int, default=10)
            p.add_argument("--max-attempts", type=int)
            p.add_argument("--split", choices=("train", "validation", "test"), default="train")
            p.add_argument("--render", action="store_true")
    args = parser.parse_args(argv)
    if args.command == "catalog":
        print(json.dumps({"tasks": TASK_IDS, "robots": ROBOTS,
                          "instruments": json.loads(files("sci_sim").joinpath("catalog.json").read_text())},
                         indent=2, ensure_ascii=False))
        return 0
    if args.command == "prepare-assets":
        from .assets import prepare_assets
        print(json.dumps(prepare_assets(args.robots), indent=2))
        return 0
    if args.command == 'import-instruments':
        from .instrument_assets import reconstruct
        from .labware_assets import reconstruct_labware
        print(json.dumps(reconstruct(args.source), indent=2))
        print(json.dumps(reconstruct_labware(args.source), indent=2))
        return 0
    config = replace(LabConfig(), robot=args.robot, width=args.width, height=args.height,
                     randomize=not args.no_randomize, layout=getattr(args, "layout", "single"),
                     instruments=args.instruments)
    if args.command == "collect":
        if sys.version_info < (3, 12):
            parser.error("LeRobot 0.7.0 requires Python 3.12; use the documented Linux environment")
        from .collect import collect
        summary = collect(args.task, args.root, args.repo_id, args.episodes, args.max_attempts,
                          args.seed, args.split, config, args.render)
        print(json.dumps(summary, indent=2))
        return 0 if summary["complete"] else 2
    env = make_env(args.task, config)
    try:
        obs, _ = env.reset(seed=args.seed)
        if args.command == 'demo':
            from .planning import DualArmExpert, EpisodeComplete, PlanningFailure
            expert = DualArmExpert(env, obs, render=args.render, seed=args.seed)
            try:
                expert.run()
            except EpisodeComplete:
                pass
            except PlanningFailure as error:
                print(str(error), file=sys.stderr)
            print(json.dumps(env.unwrapped.task_state.to_dict(), indent=2))
            return 0 if env.unwrapped.task_state.success else 2
        elif args.command == "render":
            import imageio.v3 as iio
            from .env import array
            args.output.parent.mkdir(parents=True, exist_ok=True)
            frame = array(env.render())
            iio.imwrite(args.output, frame[0] if frame.ndim == 4 else frame)
            print(args.output.resolve())
        else:
            viewer = env.unwrapped.render_human()
            print("Mouse: orbit/pan/zoom. Close the viewer to exit. This command does not collect data.")
            while not viewer.closed:
                env.unwrapped.render_human()
    finally:
        env.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
