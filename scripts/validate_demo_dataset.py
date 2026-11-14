"""Read successful local demonstrations through LeRobot (use the data venv)."""
import argparse
import json
from pathlib import Path
import numpy as np
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from sci_sim.config import CAMERAS

p = argparse.ArgumentParser()
p.add_argument('--root', type=Path, required=True)
p.add_argument('--repo-id', required=True)
args = p.parse_args()
dataset = LeRobotDataset(args.repo_id, root=args.root)
manifest = json.loads((args.root/'meta/scisim_manifest.json').read_text())
episodes = [json.loads(line) for line in (args.root/'meta/scisim_episodes.jsonl').read_text().splitlines()]
assert episodes and all(item['task_state']['success'] for item in episodes)
height, width = manifest['config']['height'], manifest['config']['width']
checked = []
for index in sorted({0, len(dataset)//2, len(dataset)-1}):
    row = dataset[index]
    assert np.isfinite(row['observation.state'].numpy()).all()
    assert row['action'].numel() == (16 if manifest['config']['robot'] == 'franka' else 14)
    for camera in CAMERAS:
        rgb = row[f'observation.images.{camera}'].numpy()
        depth = row[f'observation.depth.{camera}'].numpy()
        assert rgb.shape == (3, height, width) and np.isfinite(rgb).all()
        assert depth.shape == (1, height, width)
        assert (depth >= 0).all() and (depth < 4000).all() and (depth > 0).any()
        assert np.equal(depth, np.round(depth)).all(), 'Depth must retain integer millimetres'
        assert row[f'observation.camera.{camera}.intrinsic_cv'].numel() == 9
        assert row[f'observation.camera.{camera}.extrinsic_cv'].numel() == 12
    checked.append(index)
assert bool(dataset[len(dataset)-1]['next.done'].item())
assert bool(dataset[len(dataset)-1]['next.success'].item())
assert not bool(dataset[0]['next.success'].item())
result = {'frames': len(dataset), 'episodes': len(episodes), 'checked_rows': checked,
          'cameras': len(CAMERAS), 'resolution': [width, height],
          'terminal_success': True, 'depth_unit': 'mm',
          'scope': 'LeRobot loader and sampled RGB-D/action/calibration/terminal checks'}
(args.root/'loader_validation.json').write_text(json.dumps(result, indent=2))
print(json.dumps(result))
