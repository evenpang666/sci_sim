from dataclasses import asdict
from importlib.metadata import version
from pathlib import Path
import json
import time

from .config import (LabConfig, TASK_IDS, ARM_BASES, STAND_TUBE_DIAMETER, STAND_CROSSBAR_Z,
                     arm_mount_quaternion, arm_mount_position, asset_root, episode_seed)


def collect(task, root, repo_id, episodes=10, max_attempts=None, seed=0, split="train",
            config=None, render=False):
    import gymnasium as gym
    from . import env as _env  # noqa: F401
    from .data import LeRobotRecorder, PHASES
    from .planning import DualArmExpert, EpisodeComplete, PlanningFailure

    config = config or LabConfig()
    if config.layout != "single":
        raise ValueError("Collection requires --layout single")
    if episodes <= 0:
        raise ValueError("episodes must be positive")
    max_attempts = max_attempts or episodes * 10
    if max_attempts < episodes:
        raise ValueError("max_attempts cannot be less than episodes")
    Path(root).parent.mkdir(parents=True, exist_ok=True)
    env = gym.make(TASK_IDS[task], lab_config=asdict(config), render_mode="rgb_array",
                   max_episode_steps=config.max_steps)
    writer = None
    try:
        writer = LeRobotRecorder(env, root, repo_id)
        base = env.unwrapped
        manifest = {
            "task": task, "env_id": TASK_IDS[task], "config": asdict(config),
            "split": split, "base_seed": seed, "requested_episodes": episodes,
            "versions": {name: version(name) for name in ("sci-sim", "mani_skill", "sapien", "mplib")},
            "dataset_writer": {"lerobot": "0.7.0", "isolated_process": True},
            "asset_manifest": json.loads((asset_root() / "manifest.json").read_text()),
            "instrument_model": base.station.asset_metadata or {"source": "procedural"},
            "labware_assets": (json.loads((asset_root()/'labware/manifest.json').read_text())
                                if (asset_root()/'labware/manifest.json').exists() else None),
            "task_protocol": {"lid_operating_limit_rad": -1.65,
                              "thermal_knob_tightened_threshold_rad": 1.4,
                              "gripper_actuator_force_limit": 1.0},
            "expert_observation": "Privileged simulator object poses and contacts for geometric planning and insertion feedback; not a vision-only policy",
            "instrument_assets": (json.loads((asset_root() / 'instruments/manifest.json').read_text())
                                  if config.instruments == 'atombridge' else None),
            "mount": {"design": "cabinet_cylindrical_t", "axis_outward_down_deg": 45,
                      "end_faces": "45_degree_cuts", "crossbar_center_z_m": STAND_CROSSBAR_Z,
                      "tube_diameter_m": STAND_TUBE_DIAMETER,
                      "mount_face_centers_m": ARM_BASES,
                      "base_positions_m": [arm_mount_position(i, config.robot).tolist()
                                           for i in (0, 1)],
                      "base_quaternions_wxyz": [arm_mount_quaternion(i, config.robot).tolist()
                                                 for i in (0, 1)]},
            "depth": {"unit": "mm", "dtype": "uint16", "invalid": 0, "storage": "lossless image"},
            "camera_convention": "extrinsic_cv: world to OpenCV camera (X right, Y down, Z forward), row-major 3x4; intrinsic_cv row-major 3x3",
            "action": "absolute arm joint targets and 2F-85 actuator angle in radians; 0=open, 0.80=closed",
            "alignment": "obs[t], action[t], next.reward/done/success/phase after action[t]",
            "phases": list(PHASES),
        }
        meta = Path(root) / "meta"
        (meta / "scisim_manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
        attempted = 0
        for attempt in range(max_attempts):
            if writer.episodes >= episodes:
                break
            current_seed = episode_seed(seed, attempt, split)
            attempted += 1
            observation, _ = env.reset(seed=current_seed)
            metadata = {"seed": current_seed, "layout_seed": base.layout_seed,
                        "layout": base.initial_layout, "attempt": attempt, "split": split,
                        "initial_object_poses": {name: obj.pose.raw_pose.cpu().numpy().tolist()
                                                 for name, obj in base.station.objects.items()}}
            expert = DualArmExpert(env, observation, writer, render, current_seed)
            started = time.monotonic()
            failure = None
            try:
                expert.run()
            except EpisodeComplete:
                pass
            except PlanningFailure as error:
                failure = str(error)
            # Programming errors and IO failures are deliberately not swallowed as bad demos.
            success = base.task_state.success and failure is None
            if success:
                writer.save({**metadata, "task_state": base.task_state.to_dict()})
            else:
                writer.discard()
            report = {**metadata, "success": success, "failure": failure,
                      "steps": expert.steps, "wall_seconds": time.monotonic() - started,
                      "task_state": base.task_state.to_dict()}
            with (meta / "scisim_attempts.jsonl").open("a", encoding="utf-8") as stream:
                stream.write(json.dumps(report) + "\n")
            print(f"attempt={attempt + 1} seed={current_seed} success={success} "
                  f"kept={writer.episodes}/{episodes} reason={failure}", flush=True)
        summary = {"attempts": attempted, "successes": writer.episodes,
                   "success_rate": writer.episodes / attempted if attempted else 0,
                   "complete": writer.episodes == episodes}
        (meta / "scisim_summary.json").write_text(json.dumps(summary, indent=2))
        return summary
    finally:
        try:
            if writer:
                writer.close()
        finally:
            env.close()
