import json
import numpy as np
import pytest

from sci_sim.config import LabConfig, TASK_IDS, episode_seed, sample_layout
from sci_sim.config import ROBOTS, ARM_BASES, arm_mount_quaternion
from sci_sim.data import feature_schema, metric_depth_mm
from sci_sim.task_state import Facts, TaskState


@pytest.mark.parametrize("task", TASK_IDS)
def test_layout_reproducible_and_varies(task):
    a = sample_layout(np.random.default_rng(42), task)
    b = sample_layout(np.random.default_rng(42), task)
    c = sample_layout(np.random.default_rng(43), task)
    assert a == b
    assert a != c
    for seed in range(100):
        result = sample_layout(np.random.default_rng(seed), task)
        assert set(result) == ({"source_rack", "target_rack", "instrument", "beaker"}
                               | ({"flash"} if task == "spin_coating" else set())
                               | ({"dryer"} if task == "slide_cleaning" else set()))
        assert all(abs(p[0]) < .7 and abs(p[1]) < .4 and p[2] == .76
                   for p in result.values())


def test_config_rejects_invalid_frequency_and_robot():
    with pytest.raises(ValueError):
        LabConfig(fps=19)
    with pytest.raises(ValueError):
        LabConfig(robot="ur7e_is_really_ur5e")


def test_splits_do_not_share_seeds():
    sets = [{episode_seed(42, i, split) for i in range(100)}
            for split in ("train", "validation", "test")]
    assert len(set.union(*sets)) == 300


def test_mix_requires_continuous_contact_and_release():
    state = TaskState("mix_transfer", mix_required=.2)
    state.update(Facts(grasped=True, on_mixer=True), .1)
    state.update(Facts(grasped=True, on_mixer=False), .1)
    assert state.mix_dwell == 0
    state.update(Facts(grasped=True, on_mixer=True), .2)
    state.update(Facts(placed=True, released=False, stable=True), 1)
    assert not state.success
    state.update(Facts(placed=True, released=True, stable=True), .4)
    assert state.success


def test_plate_cannot_succeed_by_start_signal_alone():
    state = TaskState("thermal_cycler", run_required=.2)
    state.update(Facts(button_contact=True, lid_closed=True), 10)
    assert not state.running and not state.success
    state.update(Facts(lid_open=True), .1)
    state.update(Facts(lid_open=True, loaded=True, released=True), .1)
    state.update(Facts(lid_closed=True, loaded=True, released=True,
                       button_contact=True, stable=True), .5)
    assert not state.success and not state.running and state.phase == 'tighten'
    state.update(Facts(lid_closed=True, loaded=True, released=True,
                       knob_tightened=True, button_contact=True, stable=True), .5)
    assert state.success
    json.dumps(state.to_dict())


def test_centrifuge_requires_balance_and_aborts_on_open_lid():
    state = TaskState("centrifuge")
    state.update(Facts(lid_open=True, loaded=True, released=True), .1)
    state.update(Facts(lid_closed=True, loaded=True, released=True, button_contact=True), 1)
    assert not state.running
    state.update(Facts(lid_closed=True, loaded=True, balanced=True,
                       released=True, button_contact=True), .1)
    assert state.running
    state.update(Facts(lid_open=True, loaded=True, balanced=True, released=True), .1)
    assert state.failed and not state.success


@pytest.mark.parametrize("reason", ["dropped", "robot_collision"])
def test_failure_is_latched(reason):
    state = TaskState("mix_transfer")
    state.update(Facts(**{reason: True}), .1)
    state.update(Facts(grasped=True, on_mixer=True), 10)
    state.update(Facts(placed=True, released=True, stable=True), 10)
    assert state.failed and not state.success


def test_depth_preserves_metric_values_and_invalid_mask():
    values = np.array([-1, 0, 512, 1200, 4000, 90000], dtype=np.int32)
    actual = metric_depth_mm(values, 4)
    np.testing.assert_array_equal(actual, np.array([0, 0, 512, 1200, 0, 0], dtype=np.uint16))
    with pytest.raises(ValueError):
        metric_depth_mm(np.array([1.2], dtype=np.float32), 4)


def test_four_rgb_depth_pairs_and_calibration():
    schema = feature_schema(["s"], ["a"], 480, 640)
    assert sum(k.startswith("observation.images.") for k in schema) == 4
    assert sum(k.startswith("observation.depth.") for k in schema) == 4
    assert sum(k.endswith("extrinsic_cv") for k in schema) == 4
    assert schema["observation.depth.frontcam"]["info"]["is_depth_map"]


@pytest.mark.parametrize('robot', ROBOTS)
def test_mount_axes_are_outward_and_45_degrees_down(robot):
    for side in (0, 1):
        w, x, y, z = arm_mount_quaternion(side, robot)
        axis = np.array([2*(x*z+w*y), 2*(y*z-w*x), 1-2*(x*x+y*y)])
        np.testing.assert_allclose(axis, [(-1 if side == 0 else 1)/np.sqrt(2), 0,
                                          -1/np.sqrt(2)], atol=1e-7)
        assert ARM_BASES[side][1] < -.4
        assert ARM_BASES[side][2] > 1.3
