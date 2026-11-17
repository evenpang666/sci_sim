from dataclasses import asdict, dataclass
from pathlib import Path
import os
import numpy as np

TASK_IDS = {
    "mix_transfer": "SciSim-MixTransfer-v0",
    "thermal_cycler": "SciSim-ThermalCycler-v0",
    "centrifuge": "SciSim-Centrifuge-v0",
    "slide_cleaning": "SciSim-SlideCleaning-v0",
    "spin_coating": "SciSim-SpinCoating-v0",
}
INSTRUCTIONS = {
    "spin_coating": "Open the spin coater, load a glass slide, aspirate brown solution with the pipette, dispense one drop, return the pipette, close the lid and spin; after stopping, reopen the coater, open the flash evaporator, transfer the slide, close the evaporator and start flash evaporation.",
    "mix_transfer": "Pick the sample tube, vortex mix it, then place it in the destination rack.",
    "thermal_cycler": "Open the thermal cycler, insert the PCR plate, close the lid, tighten the knob and press start on the screen.",
    "centrifuge": "Open the centrifuge, load sample and balance tubes into opposite slots, close and start.",
    "slide_cleaning": "Put the slide rack into the water-filled beaker, place the beaker in the ultrasonic bath, press start, return the beaker to the table, then open the drying cabinet, place the cleaned beaker with its slide rack inside and close the cabinet.",
}
ROBOTS = ("ur7e", "ur5e", "franka", "xarm6")
CAMERAS = ("frontcam", "sidecam", "left_wristcam", "right_wristcam")
TABLE_TOP = 0.76
TABLE_SIZE = (1.40, 0.90, 0.045)
STATION_SPACING = 2.25
ARM_BASES = ((-0.245, -0.74, 1.43), (0.245, -0.74, 1.43))
STAND_TUBE_DIAMETER = .18
STAND_CROSSBAR_Z = ARM_BASES[0][2]
# The supplied collision meshes extend below their nominal local-Z=0 base plane.
# Compensate along the mount normal (measured STL/convex-mesh lower bounds).
MOUNT_MESH_OFFSETS = {'ur7e': 0., 'ur5e': 0., 'franka': .0000335, 'xarm6': .000533}


def arm_mount_position(side, robot='ur7e'):
    normal = np.array([-1 if side == 0 else 1, 0, -1]) / np.sqrt(2)
    return np.asarray(ARM_BASES[side]) + normal * MOUNT_MESH_OFFSETS[robot]


def arm_mount_quaternion(side, robot='ur7e'):
    """Base +Z points outward and 45 degrees below horizontal (wxyz)."""
    angle = (-1 if side == 0 else 1) * 3 * np.pi / 4
    tilt = np.array([np.cos(angle / 2), 0, np.sin(angle / 2), 0])
    if robot.startswith('ur'):
        return tilt
    # Panda/xArm's neutral arm heading is +X; rotate it toward the table (+Y).
    w, x, y, z = tilt
    c = np.sqrt(.5)
    return np.array([w*c-z*c, x*c+y*c, y*c-x*c, z*c+w*c])
UR_JOINTS = (
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
)
ARM_JOINTS = {
    "ur7e": UR_JOINTS, "ur5e": UR_JOINTS,
    "franka": tuple(f"panda_joint{i}" for i in range(1, 8)),
    "xarm6": tuple(f"joint{i}" for i in range(1, 7)),
}
HOME = {
    "ur7e": (1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 0),
    "ur5e": (1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 0),
    "franka": (0, -0.5, 0, -2.3, 0, 1.8, 0.7854),
    "xarm6": (0, 0.22, -1.23, 0, 1.01, 0),
}


def asset_root() -> Path:
    return Path(os.environ.get("SCI_SIM_ASSET_DIR", "assets")).resolve()


@dataclass(frozen=True)
class LabConfig:
    robot: str = "ur7e"
    layout: str = "single"
    width: int = 640
    height: int = 480
    fps: int = 20
    sim_freq: int = 240
    randomize: bool = True
    max_steps: int = 8000
    mix_seconds: float = 2.0
    run_seconds: float = 2.0
    flash_seconds: float = 2.0
    instruments: str = 'atombridge'

    def __post_init__(self):
        if self.robot not in ROBOTS:
            raise ValueError(f"robot must be one of {ROBOTS}")
        if self.layout not in ("single", "lab"):
            raise ValueError("layout must be single or lab")
        if self.instruments not in ('atombridge', 'procedural'):
            raise ValueError('instruments must be atombridge or procedural')
        if min(self.width, self.height, self.fps, self.sim_freq, self.max_steps) <= 0:
            raise ValueError("image sizes, frequencies and horizon must be positive")
        if self.sim_freq % self.fps:
            raise ValueError("sim_freq must be divisible by fps")
        if min(self.mix_seconds, self.run_seconds, self.flash_seconds) <= 0:
            raise ValueError("task dwell times must be positive")

    def to_dict(self):
        return asdict(self)


def episode_seed(seed: int, attempt: int, split: str) -> int:
    """Disjoint integer ranges, independent of Python's randomized hash()."""
    if not 0 <= seed < 1_000_000 or not 0 <= attempt < 1_000_000:
        raise ValueError("seed and attempt must lie in [0, 1000000)")
    offsets = {"train": 0, "validation": 1_000_000_000, "test": 2_000_000_000}
    if split not in offsets:
        raise ValueError(f"invalid split {split}")
    return offsets[split] + (seed + attempt) % 1_000_000_000


def sample_layout(rng: np.random.Generator, task: str, randomize: bool = True,
                  instrument_size=None) -> dict:
    """Bounded fixture jitter with rejection of overlapping footprints.

    Coordinates are station-local, metres; objects inherit their fixture's transform.
    Rotations are kept at zero in v0 to retain tube/plate insertion clearance.
    """
    nominal = {
        "source_rack": (-0.33, -0.14), "target_rack": (0.33, -0.14),
        "instrument": (0.00, -.22 if task == 'centrifuge' else -.18), "beaker": (0.52, 0.24),
    }
    sizes = {"source_rack": (0.16, 0.10), "target_rack": (0.16, 0.10),
             "instrument": {"mix_transfer": (0.122, 0.165),
                            "thermal_cycler": (0.26, 0.47),
                            "centrifuge": (0.24, 0.39),
                            "slide_cleaning": (0.28, 0.28), "spin_coating": (.30, .32)}[task], "beaker": (0.08, 0.08)}
    if task == 'slide_cleaning':
        nominal = {'source_rack': (-.38, -.18), 'target_rack': (.0, -.37),
                   'beaker': (-.16, -.18), 'instrument': (.08, -.16), 'dryer': (.45, .10)}
        sizes.update(source_rack=(.09, .09), target_rack=(.15, .10), beaker=(.15, .10), dryer=(.38, .36))
    if task == 'spin_coating':
        nominal = {'source_rack': (-.34, -.23), 'target_rack': (-.34, .13),
                   'beaker': (-.34, -.055), 'instrument': (.08, -.15), 'flash': (.46, -.15)}
        sizes.update(source_rack=(.10, .07), target_rack=(.09, .12), flash=(.28, .28))
    if instrument_size is not None:
        sizes['instrument'] = instrument_size[:2]
    # Front of a T100 is at y=-0.115; side racks stay clear of its body.
    for _ in range(500):
        poses = {k: np.array(v) + (rng.uniform(-0.018, 0.018, 2) if randomize else 0)
                 for k, v in nominal.items()}
        valid = True
        keys = list(poses)
        for i, a in enumerate(keys):
            half = np.array(sizes[a]) / 2
            if np.any(np.abs(poses[a]) + half > np.array(TABLE_SIZE[:2]) / 2 - 0.015):
                valid = False
            for b in keys[i + 1:]:
                if np.all(np.abs(poses[a] - poses[b]) <
                          (np.array(sizes[a]) + sizes[b]) / 2 + 0.012):
                    valid = False
        if valid:
            return {k: [float(x), float(y), TABLE_TOP] for k, (x, y) in poses.items()}
    raise RuntimeError("Could not sample a non-overlapping scene")
