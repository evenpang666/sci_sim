"""Render saved physical states (no planning or task success is inferred here)."""
import argparse
import pickle
import numpy as np
import imageio.v2 as iio
from pathlib import Path
from PIL import Image, ImageDraw
import gymnasium as gym
from sci_sim.config import TASK_IDS
from sci_sim.env import array
from mani_skill.utils import sapien_utils

parser = argparse.ArgumentParser()
parser.add_argument('trace', type=Path)
args = parser.parse_args()
with args.trace.open('rb') as f:
    trace = pickle.load(f)  # Only load snapshots generated locally by validate_expert.py.
env = gym.make(TASK_IDS[trace['task']], obs_mode='state_dict', render_mode='rgb_array',
    human_render_camera_configs={'overview': {'width': 960, 'height': 720,
        'pose': sapien_utils.look_at([.90, .55, 1.80], [0, -.12, 1.05])}})
env.reset(seed=trace['seed'])
base = env.unwrapped
writer = iio.get_writer(args.trace.with_suffix('.mp4'), fps=5, quality=7,
                        macro_block_size=2) if len(trace['frames']) > 1 else None
key_indices = set(np.linspace(0, len(trace['frames'])-1, 9, dtype=int))
thumbnails = []
for index, frame in enumerate(trace['frames']):
    base.set_state_dict(frame['state'])
    rgb = array(env.render())
    rgb = rgb[0] if rgb.ndim == 4 else rgb
    canvas = Image.fromarray(rgb)
    draw = ImageDraw.Draw(canvas)
    draw.rectangle((0, 0, 960, 30), fill=(20, 25, 30))
    draw.text((12, 8), f"{trace['task']} | {frame['phase']} | step {frame['step']} | seed {trace['seed']}", fill='white')
    rgb = np.array(canvas)
    if index in key_indices:
        thumbnails.append(canvas.resize((480, 360)))
    if index == len(trace['frames'])-1:
        iio.imwrite(args.trace.with_suffix('.png'), rgb)
    if writer:
        writer.append_data(rgb)
if writer:
    writer.close()
sheet = Image.new('RGB', (1440, 360*((len(thumbnails)+2)//3)))
for i, thumbnail in enumerate(thumbnails):
    sheet.paste(thumbnail, ((i%3)*480, (i//3)*360))
sheet.save(args.trace.with_name(args.trace.stem+'_stages.png'))
env.close()
