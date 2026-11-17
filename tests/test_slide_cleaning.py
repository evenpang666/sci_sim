"""Sequence/interlock regressions for the nested slide-cleaning task."""

import json
from dataclasses import replace

import pytest

from sci_sim.data import PHASES
from sci_sim.task_state import Facts, TaskState


def loaded_on_table():
    return Facts(
        rack_in_beaker=True, beaker_on_table=True, released=True, stable=True, water_ready=True
    )


def loaded_in_bath():
    return replace(loaded_on_table(), beaker_on_table=False, beaker_in_cleaner=True)


def test_cleaning_and_drying_require_ordered_nested_load():
    state = TaskState("slide_cleaning", run_required=0.2)
    # Pressing start with the final placement already satisfied must not succeed.
    state.update(
        Facts(
            button_contact=True,
            beaker_on_table=True,
            rack_on_table=True,
            released=True,
            stable=True,
            water_ready=True,
        ),
        10,
    )
    assert not state.running and not state.success
    state.update(loaded_on_table(), 0.1)
    assert state.phase == "load_cleaner"
    state.update(loaded_in_bath(), 0.1)
    assert state.phase == "start" and not state.has_cleaned
    state.update(replace(loaded_in_bath(), button_contact=True), 0.1)
    assert state.running
    state.update(loaded_in_bath(), 0.25)
    assert state.has_cleaned and not state.running and state.phase == "return_beaker"
    state.update(Facts(beaker_on_table=True, rack_on_table=True, released=True, stable=True), 1)
    assert not state.success  # Rack cannot bypass returning inside the beaker.
    state.update(loaded_on_table(), 0.1)
    assert state.has_beaker_returned and state.phase == "open_dryer"
    state.update(Facts(beaker_on_table=True, rack_on_table=True, released=True, stable=True), 0.4)
    assert not state.success  # Removing the rack is no longer the terminal step.
    state.update(replace(loaded_on_table(), dryer_open=True), 0.1)
    assert state.phase == "load_dryer"
    in_dryer = replace(
        loaded_on_table(), beaker_on_table=False, beaker_in_dryer=True, dryer_open=True
    )
    state.update(in_dryer, 0.1)
    assert state.has_dryer_loaded and state.phase == "close_dryer"
    state.update(replace(in_dryer, dryer_open=False, dryer_closed=True), 0.4)
    assert state.success and state.phase == "complete"
    json.dumps(state.to_dict())


def test_button_cannot_start_before_rack_was_loaded_on_table():
    state = TaskState("slide_cleaning")
    state.update(replace(loaded_in_bath(), button_contact=True), 10)
    assert not state.running and not state.has_beaker_loaded and not state.has_cleaned


@pytest.mark.parametrize(
    "missing", ["rack_in_beaker", "beaker_in_cleaner", "released", "water_ready"]
)
def test_cleaning_refuses_invalid_load_and_aborts_early_removal(missing):
    state = TaskState("slide_cleaning", run_required=2)
    state.update(loaded_on_table(), 0.1)
    state.update(replace(loaded_in_bath(), button_contact=True, **{missing: False}), 0.1)
    assert not state.running
    state.update(replace(loaded_in_bath(), button_contact=True), 0.1)
    assert state.running
    state.update(replace(loaded_in_bath(), **{missing: False}), 0.1)
    assert state.failed and not state.success
    assert state.failure_reason == "cleaning_load_removed_during_run"


def test_cleaning_phases_have_stable_dataset_encodings():
    assert PHASES[:11] == (
        "approach",
        "grasp",
        "mix",
        "place",
        "open",
        "load",
        "close",
        "start",
        "run",
        "complete",
        "tighten",
    )
    assert {"load_beaker", "load_cleaner", "clean", "return_beaker", "remove_rack"} <= set(PHASES)


def test_dryer_cannot_skip_cleaning_or_door_opening():
    final = Facts(
        beaker_in_dryer=True, rack_in_beaker=True, dryer_closed=True, released=True, stable=True
    )
    state = TaskState("slide_cleaning")
    state.update(final, 10)
    assert not state.success and not state.has_dryer_loaded
    state.has_cleaned = state.has_beaker_returned = True
    state.update(final, 10)
    assert not state.success and not state.has_dryer_loaded


@pytest.mark.parametrize(
    "missing", ["beaker_in_dryer", "rack_in_beaker", "dryer_closed", "released", "stable"]
)
def test_drying_completion_requires_nested_stable_load_and_closed_door(missing):
    state = TaskState(
        "slide_cleaning",
        has_cleaned=True,
        has_beaker_returned=True,
        has_dryer_opened=True,
        has_dryer_loaded=True,
    )
    final = Facts(
        beaker_in_dryer=True, rack_in_beaker=True, dryer_closed=True, released=True, stable=True
    )
    state.update(replace(final, **{missing: False}), 1)
    assert not state.success
    state.update(final, 0.4)
    assert state.success
