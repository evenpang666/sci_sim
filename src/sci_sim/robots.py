"""Four robot embodiments sharing ManiSkill's Robotiq linkage and grasp detector."""
import numpy as np
import json
from importlib.resources import files
import sapien
from mani_skill.agents.registration import register_agent
from mani_skill.agents.robots.xarm6.xarm6_robotiq import XArm6Robotiq

from .config import ARM_JOINTS, ROBOTS, asset_root

MOUNT_HOMES = json.loads(files('sci_sim').joinpath('resources/mount_homes.json').read_text())


class LabRobotiqArm(XArm6Robotiq):
    keyframes = {}
    gripper_force_limit = 1.0
    gripper_stiffness = 1000.0
    gripper_damping = 80.0
    arm_stiffness = 2000.0
    arm_damping = 200.0
    ee_link_name = "eef"

    @property
    def _sensor_configs(self):
        # Environment registers wrist cameras with stable names and configured resolution.
        return []

    def _after_loading_articulation(self):
        # Same four-bar closure as ManiSkill's XArm6Robotiq, without its xarm-only link5.
        points = {
            "right": ([-1.6048949e-08, .037600022, .043000020],
                      [1.3578170e-09, -.017901104, .0065159947]),
            "left": ([-1.8080145e-08, .037600014, .042999994],
                     [-1.4041154e-08, -.017901093, .0065159872]),
        }
        for side, (a, b) in points.items():
            finger = self.robot.active_joints_map[f"{side}_inner_finger_joint"].get_child_link()
            knuckle = self.robot.active_joints_map[f"{side}_inner_knuckle_joint"].get_child_link()
            drive = self.scene.create_drive(knuckle, sapien.Pose(a), finger, sapien.Pose(b))
            drive.set_limit_x(0, 0)
            drive.set_limit_y(0, 0)
            drive.set_limit_z(0, 0)
        # Disable only within this robot's gripper. Pairwise collision between robots remains on:
        # use a different bit for each arm, unlike the upstream single-arm hard-coded bit.
        index = int(self._agent_idx or 0)
        for name, link in self.robot.links_map.items():
            if name.startswith(("left_", "right_", "robotiq_")):
                link.set_collision_group_bit(group=2, bit_idx=20 + index, bit=1)

    def reset_home(self):
        qpos = np.zeros(len(self.robot.get_active_joints()), dtype=np.float32)
        indices = {j.name: i for i, j in enumerate(self.robot.get_active_joints())}
        home = MOUNT_HOMES[self.model][int(self._agent_idx or 0) % 2]
        for name, angle in zip(self.arm_joint_names, home):
            qpos[indices[name]] = angle
        self.reset(qpos)


def register_robots():
    for model in ROBOTS:
        uid = f"scisim_{model}_2f85"
        cls = type(f"Lab_{model}", (LabRobotiqArm,), {
            "uid": uid, "model": model,
            "urdf_path": str(asset_root() / "robots" / model / "robot.urdf"),
            "arm_joint_names": list(ARM_JOINTS[model]),
        })
        register_agent()(cls)


register_robots()
