"""Exercise real scene resets, stable physics steps, RGB-D outputs and export evidence."""
import argparse
from dataclasses import asdict
import json
from pathlib import Path
import platform

import gymnasium as gym
import imageio.v3 as iio
import numpy as np

from sci_sim import env as registered_envs
from sci_sim.config import CAMERAS, LabConfig, TASK_IDS
from sci_sim.data import metric_depth_mm


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--physics-only", action="store_true")
    parser.add_argument("--robot", default="ur7e")
    parser.add_argument("--steps", type=int, default=100)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    results = []
    for task, uid in TASK_IDS.items():
        options = {"obs_mode": "state_dict", "render_backend": "none"} if args.physics_only else {}
        env = gym.make(uid, lab_config=asdict(LabConfig(robot=args.robot, width=320, height=240)),
                       render_mode="rgb_array", **options)
        try:
            obs, _ = env.reset(seed=42)
            base = env.unwrapped
            initial = {name: registered_envs.xyz(obj).copy() for name, obj in base.station.objects.items()}
            action = base.hold_action()
            for _ in range(args.steps):
                obs, _, _, _, info = env.step(action)
            result = {"task": task, "robot": args.robot, "platform": platform.platform(),
                      "steps": args.steps, "task_state": base.task_state.to_dict(),
                      "object_drift_m": {name: float(np.linalg.norm(registered_envs.xyz(obj) - initial[name]))
                                         for name, obj in base.station.objects.items()}, "cameras": {}}
            assert not base.task_state.failed, result
            assert max(result["object_drift_m"].values()) < .025, result
            if not args.physics_only:
                frame = registered_envs.array(env.render())
                iio.imwrite(args.output / f"{task}.png", frame[0] if frame.ndim == 4 else frame)
                raw = {}
                for camera in CAMERAS:
                    sensor = obs["sensor_data"][camera]
                    rgb = registered_envs.array(sensor["rgb"])[0]
                    depth = metric_depth_mm(registered_envs.array(sensor["depth"])[0],
                                            2 if "wrist" in camera else 4)
                    valid_fraction = float((depth > 0).mean())
                    assert valid_fraction > .01, (task, camera, valid_fraction)
                    assert float(rgb.std()) > 1, (task, camera, "blank RGB")
                    iio.imwrite(args.output / f"{task}_{camera}.png", rgb)
                    raw[f"{camera}_depth_mm"] = depth
                    result["cameras"][camera] = {"rgb_shape": list(rgb.shape),
                                                   "depth_dtype": str(depth.dtype),
                                                   "valid_depth_fraction": valid_fraction}
                np.savez_compressed(args.output / f"{task}_depth.npz", **raw)
            results.append(result)
            (args.output / "report.json").write_text(json.dumps(results, indent=2))
            print(json.dumps(result), flush=True)
        finally:
            env.close()


if __name__ == "__main__":
    main()
