"""The spin stage must finish, stop and transfer the same slide before flashing."""

from dataclasses import replace

import pytest

from sci_sim.task_state import Facts, TaskState


def spun():
    state = TaskState(
        "spin_coating",
        run_required=0.1,
        flash_required=0.2,
        has_opened=True,
        has_loaded=True,
        has_aspirated=True,
        has_dispensed=True,
        has_pipette_returned=True,
        dose_count=1,
    )
    state.update(
        Facts(
            loaded=True,
            lid_closed=True,
            released=True,
            stable=True,
            pipette_returned=True,
            button_contact=True,
            spinning=True,
        ),
        0.2,
    )
    assert state.has_spun and not state.success and not state.running
    return state


def reopen():
    return Facts(
        lid_open=True,
        loaded=True,
        released=True,
        stable=True,
        pipette_returned=True,
        rotor_stopped=True,
    )


def placed():
    return replace(reopen(), loaded=False, flash_lid_open=True, flash_loaded=True)


def ready():
    return replace(placed(), flash_lid_open=False, flash_lid_closed=True)


def test_flash_requires_ordered_transfer_and_own_button():
    s = spun()
    s.update(ready(), 0.1)
    assert not s.has_flash_loaded and not s.success
    s.update(reopen(), 0.1)
    assert s.phase == "open_flash"
    s.update(replace(reopen(), flash_lid_open=True), 0.1)
    assert s.phase == "transfer_flash"
    s.update(placed(), 0.1)
    assert s.phase == "close_flash"
    s.update(replace(ready(), button_contact=True), 1)
    assert not s.flash_running and s.phase == "flash_start"
    s.update(replace(ready(), flash_button_contact=True), 0.1)
    assert s.flash_running and not s.success
    s.update(ready(), 0.5)
    assert s.success and s.phase == "complete"


def test_flash_button_cannot_skip_spin():
    s = TaskState("spin_coating")
    s.update(replace(ready(), flash_button_contact=True), 20)
    assert not s.flash_running and not s.success


def test_opening_while_rotor_coasts_fails():
    s = spun()
    s.update(replace(reopen(), rotor_stopped=False), 0.1)
    assert s.failed and s.failure_reason == "spin_opened_before_rotor_stopped"


@pytest.mark.parametrize(
    "missing", ["flash_loaded", "flash_lid_closed", "released", "pipette_returned", "rotor_stopped"]
)
def test_flash_start_and_run_interlocks(missing):
    s = spun()
    s.update(replace(reopen(), flash_lid_open=True), 0.1)
    s.update(placed(), 0.1)
    invalid = replace(ready(), **{missing: False})
    # Keep the source lid shut when testing a spinning rotor so this specifically
    # exercises the flash interlock rather than the source-lid safety check.
    invalid = replace(invalid, lid_open=False)
    s.update(replace(invalid, flash_button_contact=True), 0.1)
    assert not s.flash_running
    s.update(replace(ready(), flash_button_contact=True), 0.1)
    s.update(invalid, 0.1)
    assert s.failed and s.failure_reason == "flash_interlock_opened_during_run"
