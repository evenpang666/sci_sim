"""LeRobot v3 writer. RGB and metric depth are lossless images in the dataset.

This adapter targets lerobot==0.7.0 (Python 3.12). No Hub upload is performed.
Every row stores obs[t], action[t], and the outcome of that action, not obs[t+1].
"""
from pathlib import Path
import json
import numpy as np

from .config import CAMERAS, INSTRUCTIONS

PHASES = ("approach", "grasp", "mix", "place", "open", "load", "close", "start", "run", "complete", "tighten",
          "load_beaker", "load_cleaner", "clean", "return_beaker", "remove_rack", "aspirate", "dispense", "return_pipette", "stop_spin", "reopen_spin", "open_flash",
          "transfer_flash", "close_flash", "flash_start", "flash_run", "open_dryer", "load_dryer", "close_dryer")


def numpy(value):
    if hasattr(value, "detach"):
        return value.detach().cpu().numpy()
    return np.asarray(value)


def metric_depth_mm(value, far_m):
    """ManiSkill rgbd depth uses integer millimetres (minimal shader).

    Reject floats instead of guessing whether they represent metres or millimetres.
    Invalid/non-positive/far-plane measurements are represented by 0.
    """
    depth = numpy(value)
    if not np.issubdtype(depth.dtype, np.integer):
        raise ValueError("Expected ManiSkill integer millimetre depth; shader/format changed")
    valid = (depth > 0) & (depth < far_m * 1000) & (depth <= 65535)
    return np.where(valid, depth, 0).astype(np.uint16)


def feature_schema(state_names, action_names, height, width):
    features = {
        "observation.state": {"dtype": "float32", "shape": (len(state_names),), "names": state_names},
        "action": {"dtype": "float32", "shape": (len(action_names),), "names": action_names},
        "next.reward": {"dtype": "float32", "shape": (1,), "names": None},
        "next.done": {"dtype": "bool", "shape": (1,), "names": None},
        "next.success": {"dtype": "bool", "shape": (1,), "names": None},
        "next.phase": {"dtype": "int64", "shape": (1,), "names": ["phase_index"]},
    }
    for camera in CAMERAS:
        features[f"observation.images.{camera}"] = {
            "dtype": "image", "shape": (height, width, 3), "names": ["height", "width", "channels"]}
        features[f"observation.depth.{camera}"] = {
            "dtype": "image", "shape": (height, width, 1), "names": ["height", "width", "channels"],
            "info": {"is_depth_map": True, "depth_unit": "mm"}}
        features[f"observation.camera.{camera}.intrinsic_cv"] = {
            "dtype": "float32", "shape": (9,), "names": None}
        features[f"observation.camera.{camera}.extrinsic_cv"] = {
            "dtype": "float32", "shape": (12,), "names": None}
    return features


class LeRobotRecorder:
    def __init__(self, env, root, repo_id):
        from .dataset_client import DatasetClient

        self.env = env.unwrapped
        if self.env.lab_config.layout != "single":
            raise ValueError("Collect single-table tasks; the gallery has six arms and twelve cameras")
        self.root = Path(root)
        if self.root.exists():
            raise FileExistsError(f"Choose a new dataset directory: {self.root}")
        self.keys = list(self.env.agent.agents_dict)
        self.state_names, self.action_names = [], []
        for side, agent in zip(("left", "right"), self.env.active_agents):
            joints = [j.name for j in agent.robot.get_active_joints()]
            self.state_names.extend(f"{side}.qpos.{name}" for name in joints)
            self.state_names.extend(f"{side}.qvel.{name}" for name in joints)
            self.action_names.extend(f"{side}.target_rad.{name}" for name in agent.arm_joint_names)
            self.action_names.append(f"{side}.gripper_target_rad")
        cfg = self.env.lab_config
        self.dataset = DatasetClient(
            repo_id=repo_id, root=self.root, fps=cfg.fps,
            robot_type=f"dual_{cfg.robot}_robotiq_2f85",
            features=feature_schema(self.state_names, self.action_names, cfg.height, cfg.width),
            use_videos=False, image_writer_threads=4)
        self.frames = 0
        self.episodes = 0

    def __call__(self, observation, action, reward, terminated, truncated, info):
        state = []
        for key in self.keys:
            proprio = observation["agent"][key]
            state.extend([numpy(proprio["qpos"])[0].ravel(), numpy(proprio["qvel"])[0].ravel()])
        frame = {
            "observation.state": np.concatenate(state).astype(np.float32),
            "action": np.concatenate([numpy(action[k]).ravel() for k in self.keys]).astype(np.float32),
            "next.reward": numpy(reward).astype(np.float32).reshape(1),
            "next.done": np.array([bool(numpy(terminated).any() or numpy(truncated).any())]),
            "next.success": np.array([bool(numpy(info["success"]).any())]),
            "next.phase": np.array([PHASES.index(info["phase"])], dtype=np.int64),
            "task": INSTRUCTIONS[self.env.task_name],
        }
        for name in CAMERAS:
            sensor = observation["sensor_data"][name]
            rgb = numpy(sensor["rgb"])[0, ..., :3]
            if rgb.dtype != np.uint8:
                raise ValueError("Expected uint8 ManiSkill RGB")
            frame[f"observation.images.{name}"] = rgb
            frame[f"observation.depth.{name}"] = metric_depth_mm(
                numpy(sensor["depth"])[0], 2.0 if "wrist" in name else 4.0)
            params = observation["sensor_param"][name]
            for kind, size in (("intrinsic_cv", 9), ("extrinsic_cv", 12)):
                values = numpy(params[kind])[0].astype(np.float32).ravel()
                if len(values) != size:
                    raise ValueError(f"Unexpected {name}/{kind} shape")
                frame[f"observation.camera.{name}.{kind}"] = values
        self.dataset.add_frame(frame)
        self.frames += 1

    def save(self, metadata):
        if not self.env.task_state.success or self.frames == 0:
            raise RuntimeError("Only nonempty, physically successful episodes may be saved")
        self.dataset.save_episode()
        record = {"episode_index": self.episodes, "frames": self.frames, **metadata}
        with (self.root / "meta/scisim_episodes.jsonl").open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(record, ensure_ascii=False) + "\n")
        self.episodes += 1
        self.frames = 0

    def discard(self):
        self.dataset.clear_episode_buffer()
        self.frames = 0

    def close(self):
        self.dataset.finalize()
