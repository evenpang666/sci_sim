"""Ordered liquid transfer and safety interlocks for spin coating."""

from dataclasses import replace

import pytest

from sci_sim.data import PHASES
from sci_sim.task_state import Facts, TaskState


def loaded():
    return Facts(lid_open=True, loaded=True, released=True, stable=True)


def prepared():
    s = TaskState("spin_coating", run_required=0.2)
    s.update(loaded(), 0.1)
    s.update(replace(loaded(), tip_immersed=True, released=False), 0.6)
    s.update(replace(loaded(), tip_over_slide=True, released=False), 0.5)
    s.update(replace(loaded(), pipette_returned=True), 0.1)
    return s


def ready():
    return Facts(loaded=True, lid_closed=True, released=True, stable=True, pipette_returned=True)


def test_seven_steps_and_motor_rotation_are_required():
    s = TaskState("spin_coating", run_required=0.2)
    s.update(replace(ready(), button_contact=True, spinning=True), 10)
    assert not s.running and not s.success
    s.update(Facts(lid_open=True), 0.1)
    assert s.phase == "load"
    s.update(loaded(), 0.1)
    assert s.phase == "aspirate"
    s.update(replace(loaded(), tip_over_slide=True), 2)
    assert not s.has_dispensed
    s.update(replace(loaded(), tip_immersed=True), 0.3)
    s.update(loaded(), 0.1)  # Breaking immersion resets its dwell.
    s.update(replace(loaded(), tip_immersed=True), 0.3)
    assert not s.has_aspirated
    s.update(replace(loaded(), tip_immersed=True), 0.3)
    assert s.phase == "dispense"
    s.update(replace(loaded(), tip_over_slide=True), 0.5)
    assert s.phase == "return_pipette" and s.dose_count == 1
    s.update(replace(loaded(), tip_over_slide=True), 3)
    assert s.dose_count == 1
    s.update(replace(loaded(), pipette_returned=True), 0.1)
    assert s.phase == "close"
    s.update(ready(), 0.1)
    assert s.phase == "start"
    s.update(replace(ready(), button_contact=True), 0.1)
    s.update(ready(), 10)  # A stalled motor cannot complete the task.
    assert s.running and not s.success and s.run_dwell == 0
    s.update(replace(ready(), spinning=True), 0.5)
    assert s.has_spun and not s.running and not s.success


@pytest.mark.parametrize("missing", ["loaded", "lid_closed", "pipette_returned", "released"])
def test_start_and_running_interlocks(missing):
    s = prepared()
    invalid = replace(ready(), **{missing: False})
    s.update(replace(invalid, button_contact=True), 0.1)
    assert not s.running
    s.update(replace(ready(), button_contact=True), 0.1)
    s.update(invalid, 0.1)
    assert s.failed and s.failure_reason == "spin_interlock_opened_during_run"


def test_pipette_must_return_before_lid_closes():
    s = TaskState("spin_coating")
    s.update(loaded(), 0.1)
    s.update(replace(loaded(), tip_immersed=True), 0.6)
    s.update(replace(loaded(), tip_over_slide=True), 0.5)
    s.update(replace(ready(), button_contact=True), 2)
    assert not s.running and not s.has_pipette_returned


def test_new_phase_codes_append_without_changing_existing_codes():
    assert PHASES[16:19] == ("aspirate", "dispense", "return_pipette")
