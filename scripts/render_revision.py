"""Render scene geometry and reconstructed instruments for visual inspection."""
from pathlib import Path
import gymnasium as gym
import imageio.v3 as iio
from mani_skill.utils import sapien_utils
from sci_sim import env as registered
from sci_sim.config import TASK_IDS

output = Path('outputs/revision2')
output.mkdir(parents=True, exist_ok=True)
for task, uid in TASK_IDS.items():
    env = gym.make(uid, lab_config={'width': 160, 'height': 120}, render_mode='rgb_array',
                   human_render_camera_configs={'overview': {'width': 1200, 'height': 900,
                     'pose': sapien_utils.look_at([1.7, -2.2, 2.0], [0, -.45, .95])}})
    env.reset(seed=42)
    image = registered.array(env.render())[0]
    iio.imwrite(output / f'{task}_stand.png', image)
    env.close()
    for opened in (False, True) if task != 'mix_transfer' else (False,):
        env = gym.make(uid, lab_config={'width': 160, 'height': 120}, render_mode='rgb_array',
            human_render_camera_configs={'overview': {'width': 1000, 'height': 800,
                'pose': sapien_utils.look_at([.14, -.55, 1.50], [0, .09, 1.0]), 'fov': 1.0}})
        env.reset(seed=42)
        if opened:
            # Geometry preview only: this is not an executed expert trajectory.
            env.unwrapped.station.machine.set_qpos([[-1.5]])
        image = registered.array(env.render())[0]
        iio.imwrite(output / f'{task}_instrument_{"open" if opened else "closed"}.png', image)
        env.close()
