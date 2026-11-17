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
    rack_in_beaker: bool = False
    beaker_in_cleaner: bool = False
    beaker_on_table: bool = False
    rack_on_table: bool = False
    water_ready: bool = False
    tip_immersed: bool = False
    tip_over_slide: bool = False
    pipette_returned: bool = False
    spinning: bool = False
    dryer_open: bool = False
    dryer_closed: bool = False
    beaker_in_dryer: bool = False
    rotor_stopped: bool = False
    flash_lid_open: bool = False
    flash_lid_closed: bool = False
    flash_loaded: bool = False
    flash_button_contact: bool = False


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
    has_rack_loaded: bool = False
    has_beaker_loaded: bool = False
    has_cleaned: bool = False
    has_beaker_returned: bool = False
    has_aspirated: bool = False
    has_dispensed: bool = False
    has_pipette_returned: bool = False
    aspirate_dwell: float = 0
    dispense_dwell: float = 0
    drop_age: float = 0
    dose_count: int = 0
    has_dryer_opened: bool = False
    has_dryer_loaded: bool = False
    has_spun: bool = False
    has_spin_reopened: bool = False
    has_flash_opened: bool = False
    has_flash_loaded: bool = False
    flash_running: bool = False
    flash_dwell: float = 0
    flash_required: float = 2

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
        elif self.task == 'slide_cleaning':
            self.has_rack_loaded |= (facts.rack_in_beaker and facts.beaker_on_table
                                     and facts.released and facts.stable)
            ready = (self.has_rack_loaded and facts.rack_in_beaker
                     and facts.beaker_in_cleaner and facts.released and facts.water_ready)
            self.has_beaker_loaded |= ready and facts.stable
            if self.running and not self.has_cleaned:
                if not ready:
                    self.failed, self.failure_reason = True, 'cleaning_load_removed_during_run'
                    return
                self.run_dwell += dt
                if self.run_dwell >= self.run_required:
                    self.has_cleaned = True
                    self.running = False
            elif not self.has_cleaned and self.has_beaker_loaded and ready and facts.button_contact:
                self.running = True
            self.has_beaker_returned |= (self.has_cleaned and facts.beaker_on_table
                                         and facts.rack_in_beaker and facts.released and facts.stable)
            self.has_dryer_opened |= self.has_beaker_returned and facts.dryer_open
            self.has_dryer_loaded |= (self.has_dryer_opened and facts.dryer_open
                                      and facts.beaker_in_dryer and facts.rack_in_beaker
                                      and facts.released and facts.stable)
            self.phase = ('close_dryer' if self.has_dryer_loaded else
                          'load_dryer' if self.has_dryer_opened else
                          'open_dryer' if self.has_beaker_returned else
                          'return_beaker' if self.has_cleaned else
                          'clean' if self.running else
                          'start' if self.has_beaker_loaded and ready else
                          'load_cleaner' if self.has_rack_loaded else 'load_beaker')
            complete = (self.has_dryer_loaded and facts.beaker_in_dryer
                        and facts.rack_in_beaker and facts.dryer_closed
                        and facts.released and facts.stable)
        elif self.task == 'spin_coating':
            self.has_opened |= facts.lid_open
            self.has_loaded |= self.has_opened and facts.lid_open and facts.loaded and facts.released
            if not self.has_aspirated:
                eligible = self.has_loaded and facts.loaded and facts.lid_open and facts.tip_immersed
                self.aspirate_dwell = self.aspirate_dwell + dt if eligible else 0
                self.has_aspirated = self.aspirate_dwell >= .5
            if self.has_aspirated and not self.has_dispensed:
                self.dispense_dwell = self.dispense_dwell + dt if facts.tip_over_slide else 0
                if self.dispense_dwell >= .4:
                    self.has_dispensed = True
                    self.dose_count = 1
            if self.has_dispensed:
                self.drop_age += dt
            self.has_pipette_returned |= (self.has_dispensed and facts.lid_open
                                         and facts.pipette_returned and facts.stable)
            ready = (self.has_loaded and self.has_dispensed and self.has_pipette_returned
                     and facts.loaded and facts.pipette_returned and facts.released and facts.lid_closed)
            if self.running and not ready:
                self.failed, self.failure_reason = True, 'spin_interlock_opened_during_run'
                return
            if not self.has_spun and ready and facts.button_contact:
                self.running = True
            if self.running and facts.spinning:
                self.run_dwell += dt
            self.phase = ('run' if self.running else 'start' if ready else
                          'close' if self.has_pipette_returned else
                          'return_pipette' if self.has_dispensed else
                          'dispense' if self.has_aspirated else
                          'aspirate' if self.has_loaded else
                          'load' if self.has_opened else 'open')
            if self.running and self.run_dwell >= self.run_required and facts.stable:
                self.has_spun, self.running = True, False
            complete = False
            if self.has_spun:
                if not facts.rotor_stopped and facts.lid_open:
                    self.failed, self.failure_reason = True, 'spin_opened_before_rotor_stopped'
                    return
                self.has_spin_reopened |= (facts.rotor_stopped and facts.lid_open
                                           and facts.loaded and facts.released and facts.stable)
                self.has_flash_opened |= (self.has_spin_reopened and facts.lid_open
                                          and facts.rotor_stopped and facts.flash_lid_open)
                self.has_flash_loaded |= (self.has_flash_opened and facts.flash_lid_open
                                          and facts.flash_loaded and facts.released and facts.stable)
                flash_ready = (self.has_flash_loaded and facts.flash_loaded
                               and facts.flash_lid_closed and facts.released
                               and facts.pipette_returned and facts.rotor_stopped)
                if self.flash_running and not flash_ready:
                    self.failed, self.failure_reason = True, 'flash_interlock_opened_during_run'
                    return
                if flash_ready and facts.flash_button_contact:
                    self.flash_running = True
                if self.flash_running:
                    self.flash_dwell += dt
                self.phase = ('flash_run' if self.flash_running else
                              'flash_start' if flash_ready else
                              'close_flash' if self.has_flash_loaded else
                              'transfer_flash' if self.has_flash_opened else
                              'open_flash' if self.has_spin_reopened else
                              'reopen_spin' if facts.rotor_stopped else 'stop_spin')
                complete = (self.flash_running and self.flash_dwell >= self.flash_required
                            and facts.stable)
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
