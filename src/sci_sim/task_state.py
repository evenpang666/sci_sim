"""Task progression is driven solely by measured scene facts, once per control step."""
from dataclasses import dataclass, asdict


@dataclass
class Facts:
    grasped: bool = False
    on_mixer: bool = False
    placed: bool = False
    lid_open: bool = False
    lid_closed: bool = False
    loaded: bool = False
    balanced: bool = False
    button_contact: bool = False
    dropped: bool = False
    robot_collision: bool = False
    released: bool = False
    stable: bool = False
    knob_tightened: bool = False


@dataclass
class TaskState:
    task: str
    mix_required: float = 2
    run_required: float = 2
    mix_dwell: float = 0
    has_mixed: bool = False
    has_opened: bool = False
    has_loaded: bool = False
    running: bool = False
    run_dwell: float = 0
    settle_dwell: float = 0
    success: bool = False
    failed: bool = False
    failure_reason: str = ""
    phase: str = "approach"

    def update(self, facts: Facts, dt: float):
        if self.failed or self.success:
            return
        if facts.dropped or facts.robot_collision:
            self.failed = True
            self.failure_reason = "object_dropped" if facts.dropped else "inter_arm_collision"
            return
        if self.task == "mix_transfer":
            self.mix_dwell = self.mix_dwell + dt if facts.grasped and facts.on_mixer else 0
            self.has_mixed |= self.mix_dwell >= self.mix_required
            self.phase = "place" if self.has_mixed else "mix" if facts.grasped else "grasp"
            complete = self.has_mixed and facts.placed and facts.released and facts.stable
        else:
            self.has_opened |= facts.lid_open
            self.has_loaded |= self.has_opened and facts.loaded and facts.released
            valid_load = facts.loaded and (self.task != "centrifuge" or facts.balanced)
            closed_loaded = self.has_loaded and valid_load and facts.released and facts.lid_closed
            ready = closed_loaded and (self.task != 'thermal_cycler' or facts.knob_tightened)
            if self.running and not ready:
                self.failed, self.failure_reason = True, "interlock_opened_during_run"
                return
            if ready and facts.button_contact:
                self.running = True
            if self.running:
                self.run_dwell += dt
            self.phase = ("run" if self.running else "start" if ready else
                          "tighten" if closed_loaded else
                          "close" if self.has_loaded and valid_load else
                          "load" if self.has_opened else "open")
            complete = self.running and self.run_dwell >= self.run_required and facts.stable
        self.settle_dwell = self.settle_dwell + dt if complete else 0
        self.success = self.settle_dwell >= .3
        if self.success:
            self.phase = "complete"

    def to_dict(self):
        return asdict(self)
