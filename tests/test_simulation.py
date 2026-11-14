"""Opt-in Linux integration tests: SCI_SIM_INTEGRATION=1 pytest -m integration."""
import os
import pytest

pytestmark = [pytest.mark.integration,
              pytest.mark.skipif(os.getenv("SCI_SIM_INTEGRATION") != "1", reason="opt-in GPU/render test")]


@pytest.mark.parametrize("task", ["mix_transfer", "thermal_cycler", "centrifuge"])
def test_scene_reset_step_rgbd(task):
    import numpy as np
    from sci_sim.cli import make_env
    from sci_sim.config import CAMERAS, LabConfig

    env = make_env(task, LabConfig(width=160, height=120))
    try:
        observation, _ = env.reset(seed=42)
        assert len(env.unwrapped.agent.agents) == 2
        for side, agent in enumerate(env.unwrapped.active_agents):
            rotation = np.asarray(agent.robot.pose.to_transformation_matrix())[0, :3, :3]
            np.testing.assert_allclose(rotation[:, 2],
                [(-1 if side == 0 else 1)/np.sqrt(2), 0, -1/np.sqrt(2)], atol=1e-6)
        assert env.unwrapped.station.asset_metadata['source'] == 'AtomBridge local reconstruction'
        assert set(observation["sensor_data"]) == set(CAMERAS)
        action = env.unwrapped.hold_action()
        for camera in CAMERAS:
            assert tuple(observation["sensor_data"][camera]["rgb"].shape) == (1, 120, 160, 3)
            assert tuple(observation["sensor_data"][camera]["depth"].shape) == (1, 120, 160, 1)
        _, _, _, _, info = env.step(action)
        assert not bool(np.asarray(info["success"]).any())
    finally:
        env.close()
