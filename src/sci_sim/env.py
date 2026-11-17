"""ManiSkill environments: independent tasks, optionally in a shared multi-table lab."""
import math
import platform
import numpy as np
import sapien
import torch

from mani_skill.envs.sapien_env import BaseEnv
from mani_skill.sensors.camera import CameraConfig
from mani_skill.utils import sapien_utils
from mani_skill.utils.registration import register_env
from mani_skill.utils.building.ground import build_ground
from mani_skill.utils.structs.types import SimConfig

from . import robots as _robots  # noqa: F401 - registration
from .config import (STATION_SPACING, TABLE_TOP, TASK_IDS, arm_mount_quaternion, arm_mount_position,
                     LabConfig, asset_root, sample_layout)
from .scene import RACK_SLOTS, build_station
from .task_state import Facts, TaskState


def array(value):
    if isinstance(value, torch.Tensor):
        return value.detach().cpu().numpy()
    return np.asarray(value)


def xyz(actor):
    return array(actor.pose.p)[0]


class LabEnv(BaseEnv):
    SUPPORTED_ROBOTS = None  # the constructor validates our four registered embodiments
    task_name = "mix_transfer"

    def __init__(self, *args, lab_config=None, **kwargs):
        self.lab_config = (LabConfig(**lab_config) if isinstance(lab_config, dict)
                           else lab_config or LabConfig())
        if kwargs.get("num_envs", 1) != 1:
            raise ValueError("v0 uses one CPU physics scene per process. Scale with separate workers.")
        if kwargs.get("sim_backend", "cpu") not in ("auto", "cpu", "physx_cpu"):
            raise ValueError("v0 contact-based task predicates require CPU physics")
        path = asset_root() / "robots" / self.lab_config.robot / "robot.urdf"
        if not path.is_file():
            raise FileNotFoundError(f"Missing {path}; first run sci-sim prepare-assets")
        self.station_tasks = list(TASK_IDS) if self.lab_config.layout == "lab" else [self.task_name]
        self.origins = [np.array([i * STATION_SPACING, 0, 0]) for i in range(len(self.station_tasks))]
        self.active_index = self.station_tasks.index(self.task_name)
        uid = f"scisim_{self.lab_config.robot}_2f85"
        self.task_state = TaskState(self.task_name)
        kwargs.setdefault("obs_mode", "rgbd")
        kwargs.setdefault("control_mode", "pd_joint_pos")
        kwargs.setdefault("reward_mode", "sparse")
        # Windows SAPIEN supports Vulkan rendering but not its CUDA tensor export.
        # Host readback still renders on the graphics device and works with CPU PyTorch.
        if platform.system() == "Windows" or not torch.cuda.is_available():
            kwargs.setdefault("render_backend", "cpu")
        kwargs["sim_backend"] = "cpu"
        kwargs["num_envs"] = 1
        # The minimal shader renders alpha glass/water dark; the default raster
        # shader supports transparency for this task and for the combined lab.
        if self.task_name in ('slide_cleaning', 'spin_coating') or self.lab_config.layout == 'lab':
            for name in ('sensor_configs', 'human_render_camera_configs'):
                configs = dict(kwargs.get(name) or {})
                configs.setdefault('shader_pack', 'default')
                kwargs[name] = configs
        super().__init__(*args, robot_uids=tuple([uid] * (2 * len(self.station_tasks))), **kwargs)

    @property
    def _default_sim_config(self):
        return SimConfig(sim_freq=self.lab_config.sim_freq, control_freq=self.lab_config.fps)

    def _load_agent(self, options):
        poses = []
        for origin in self.origins:
            for side in (0, 1):
                base = arm_mount_position(side, self.lab_config.robot)
                q = arm_mount_quaternion(side, self.lab_config.robot)
                poses.append(sapien.Pose(origin + base, q))
        super()._load_agent(options, poses)

    def _load_scene(self, options):
        # build_ground's floor_length spans world X. Include every station
        # in the visible floor as well as in its infinite collision plane.
        lab_span = (len(self.station_tasks) - 1) * STATION_SPACING
        floor_length = max(8, math.ceil(lab_span + 4))
        floor_center = (lab_span / 2, 0) if self.lab_config.layout == 'lab' else (2, 0)
        build_ground(self.scene, floor_width=12, floor_length=floor_length, xy_origin=floor_center,
                     texture_square_len=1)
        self.stations = [build_station(self.scene, task, origin, f"station{i}_", self.lab_config.instruments)
                         for i, (task, origin) in enumerate(zip(self.station_tasks, self.origins))]
        self.station = self.stations[self.active_index]
    def _load_lighting(self, options):
        self.scene.set_ambient_light([.45, .45, .45])
        self.scene.add_directional_light([1, 1, -2], [2, 2, 2], shadow=True)

    @property
    def active_agents(self):
        i = self.active_index * 2
        return self.agent.agents[i:i + 2]

    def hold_action(self):
        """Hold all arms at the current joint positions with grippers open."""
        action = {}
        for key, agent in self.agent.agents_dict.items():
            names = [joint.name for joint in agent.robot.get_active_joints()]
            indices = [names.index(name) for name in agent.arm_joint_names]
            action[key] = np.r_[array(agent.robot.get_qpos())[0, indices], 0].astype(np.float32)
        return action

    @property
    def _default_sensor_configs(self):
        configs = []
        cfg = self.lab_config
        for i, origin in enumerate(self.origins):
            prefix = f"station{i}_" if cfg.layout == "lab" else ""
            for name, eye in (("frontcam", [0, -.48, 1.85]), ("sidecam", [1.15, -.05, 1.45])):
                pose = sapien_utils.look_at(origin + eye, origin + [0, 0, 0.91])
                configs.append(CameraConfig(prefix + name, pose, cfg.width, cfg.height,
                                            math.radians(65), .02, 4.0))
            for j, side in enumerate(("left", "right")):
                configs.append(CameraConfig(
                    prefix + side + "_wristcam",
                    sapien_utils.look_at([.055, 0, -.075], [0, 0, .08]),
                    cfg.width, cfg.height, math.radians(75), .015, 2.0,
                    mount=self.agent.agents[2 * i + j].tcp))
        return configs

    @property
    def _default_human_render_camera_configs(self):
        n = len(self.origins)
        target = [(n - 1) * STATION_SPACING / 2, -.05, .65]
        eye = [target[0] + 1.65, -2.55 - (n - 1) * .9, 2.65 + (n - 1) * .30]
        return CameraConfig("overview", sapien_utils.look_at(eye, target), 1600, 1000,
                            math.radians(65), .02, 20)

    def _initialize_episode(self, env_idx, options):
        self.task_state = TaskState(self.task_name, self.lab_config.mix_seconds,
                                   self.lab_config.run_seconds,
                                   flash_required=self.lab_config.flash_seconds)
        for agent in self.agent.agents:
            agent.reset_home()
        # ManiSkill seeds this generator in reset(seed=...). Use a local seed in the manifest.
        self.layout_seed = int(self._episode_rng.randint(0, 2**31 - 1))
        rng = np.random.default_rng(self.layout_seed)
        for station in self.stations:
            station.set_layout(sample_layout(rng, station.task, self.lab_config.randomize,
                                             station.instrument_size))
        self.initial_layout = {s.prefix: s.layout for s in self.stations}

    def _get_obs_extra(self, info):
        return {"tcp_poses": torch.cat([a.tcp.pose.raw_pose for a in self.active_agents], dim=-1)}

    def contact(self, a, b, threshold=.02):
        force = self.scene.get_pairwise_contact_forces(a, b)
        return bool((torch.linalg.norm(force, dim=-1) > threshold).any().item())

    def is_held(self, obj):
        return any(bool(a.is_grasping(obj, min_force=.02).item()) for a in self.active_agents)

    def close_to(self, obj, target, xy=.010, z=.010):
        delta = xyz(obj) - target
        upright = float(array(obj.pose.to_transformation_matrix())[0, 2, 2]) > .97
        return np.linalg.norm(delta[:2]) < xy and abs(delta[2]) < z and upright

    def target_position(self, name):
        s = self.station
        if self.task_name == 'spin_coating' and name == 'pipette':
            from .spin_coating import PIPETTE_HOME
            return s.world(np.array(s.layout['target_rack']) + PIPETTE_HOME)
        if self.task_name == 'slide_cleaning':
            from .slide_drying import LOAD_LOCAL
            from .slide_cleaning import BEAKER_FLOOR
            target = s.dryer.instrument_point(LOAD_LOCAL)
            return target + ([0, 0, BEAKER_FLOOR] if name == 'slide_rack' else [0, 0, 0])
        if name in s.target_local:
            return s.instrument_point(s.target_local[name])
        if self.task_name == "mix_transfer":
            return s.world(np.asarray(s.layout["target_rack"]) + RACK_SLOTS[0])
        if self.task_name == "thermal_cycler":
            return s.instrument_point([0, 0, .175])
        angle = math.pi / 24 + (0 if name == "sample" else math.pi)
        return s.instrument_point([.066 * math.cos(angle), .066 * math.sin(angle), .165])

    def measure_facts(self):
        s = self.station
        objs = list(s.objects.values())
        held = [self.is_held(obj) for obj in objs]
        stable = all(np.linalg.norm(array(obj.linear_velocity)) < .025 and
                     np.linalg.norm(array(obj.angular_velocity)) < .25 for obj in objs)
        # All active robot links are included, not merely the end-effectors.
        left, right = self.active_agents
        left_entities = {link._bodies[0].entity for link in left.robot.get_links()}
        right_entities = {link._bodies[0].entity for link in right.robot.get_links()}
        collision = False
        # CPU PhysX returns all contacts in one query. Preserve the same pairwise
        # force criterion without re-reading every contact for every link pair.
        impulses = {}
        for contact in self.scene.get_contacts():
            a, b = (body.entity for body in contact.bodies)
            sign = 1
            if a in right_entities and b in left_entities:
                a, b = b, a
                sign = -1
            elif not (a in left_entities and b in right_entities):
                continue
            impulses.setdefault((a, b), np.zeros(3))
            impulses[(a, b)] += sign * sum((p.impulse for p in contact.points), np.zeros(3))
        collision = any(np.linalg.norm(v) / self.scene.px.timestep > 2.0
                        for v in impulses.values())
        facts = Facts(grasped=any(held), released=not any(held), stable=stable,
                      dropped=any(xyz(obj)[2] < TABLE_TOP - .04 for obj in objs),
                      robot_collision=collision)
        if self.task_name == 'spin_coating':
            from .spin_coating import measure_spin_facts
            measure_spin_facts(self, facts)
        elif self.task_name == 'slide_cleaning':
            from .slide_cleaning import measure_cleaning_facts
            measure_cleaning_facts(self, facts)
        elif self.task_name == "mix_transfer":
            sample = s.objects["sample"]
            mix_point = s.mixer_local if s.mixer_local is not None else [0, 0, .189]
            facts.on_mixer = (self.close_to(sample, s.instrument_point(mix_point), .012, .009)
                              and self.contact(sample, s.machine))
            facts.placed = self.close_to(sample, self.target_position("sample"))
        else:
            facts.lid_open = s.lid_angle < -1.25
            facts.lid_closed = s.lid_angle > -.07
            facts.loaded = all(self.close_to(obj, self.target_position(name), xy=.009)
                               for name, obj in s.objects.items())
            if self.task_name == "thermal_cycler":
                facts.knob_tightened = s.knob_angle >= 1.4
                # Plate yaw must align with the bed, not simply have an upright normal.
                matrix = array(objs[0].pose.to_transformation_matrix())[0]
                facts.loaded = bool(facts.loaded and matrix[0, 0] > .98)
            facts.balanced = facts.loaded and self.task_name == "centrifuge"
            facts.button_contact = any(self.contact(link, s.button)
                                       for a in self.active_agents
                                       for link in (a.finger1_link, a.finger2_link))
        return facts

    def _after_control_step(self):
        self.task_state.update(self.measure_facts(), self.control_timestep)
        if self.task_name == "spin_coating":
            from .spin_coating import update_visuals
            self.station.spin_joint.set_drive_velocity_target(
                6.0 if self.task_state.running and not self.task_state.failed else 0.0)
            update_visuals(self.station, self.task_state)

    def evaluate(self):
        state = self.task_state
        return {"success": torch.tensor([state.success], device=self.device),
                "fail": torch.tensor([state.failed], device=self.device),
                "phase": state.phase, "failure_reason": state.failure_reason}

    def compute_dense_reward(self, obs, action, info):
        raise NotImplementedError("v0 benchmark defines sparse success rewards only")


@register_env(TASK_IDS["mix_transfer"], max_episode_steps=5000)
class MixTransferEnv(LabEnv):
    task_name = "mix_transfer"


@register_env(TASK_IDS["thermal_cycler"], max_episode_steps=5000)
class ThermalCyclerEnv(LabEnv):
    task_name = "thermal_cycler"


@register_env(TASK_IDS["centrifuge"], max_episode_steps=5000)
class CentrifugeEnv(LabEnv):
    task_name = "centrifuge"


@register_env(TASK_IDS['slide_cleaning'], max_episode_steps=8000)
class SlideCleaningEnv(LabEnv):
    task_name = 'slide_cleaning'


@register_env(TASK_IDS["spin_coating"], max_episode_steps=8000)
class SpinCoatingEnv(LabEnv):
    task_name = "spin_coating"
