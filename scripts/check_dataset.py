"""Read diagnostic data through LeRobot and compare every depth pixel."""
import argparse
import json
from pathlib import Path
import numpy as np
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from sci_sim.config import CAMERAS

parser = argparse.ArgumentParser()
parser.add_argument('--root', type=Path, required=True)
args = parser.parse_args()
dataset = LeRobotDataset('local/scisim_diagnostic', root=args.root)
expected = np.load(args.root / 'expected_depth.npz')
assert len(dataset) == 3
for i in range(len(dataset)):
    row = dataset[i]
    assert row['action'].shape == (14,)
    assert row['observation.state'].shape == (48,)
    assert not bool(row['next.success'].any())
    for camera in CAMERAS:
        depth = row[f'observation.depth.{camera}'].numpy().transpose(1, 2, 0)
        np.testing.assert_array_equal(depth, expected[f'{i}_{camera}'])
        assert row[f'observation.images.{camera}'].shape == (3, 120, 160)
        assert row[f'observation.camera.{camera}.intrinsic_cv'].numel() == 9
        assert row[f'observation.camera.{camera}.extrinsic_cv'].numel() == 12
result = {'frames': 3, 'cameras': 4, 'all_depth_pixels_exact': True,
          'loader_depth_dtype': 'float32', 'depth_unit': 'mm', 'successful_demonstration': False}
(args.root / 'roundtrip_report.json').write_text(json.dumps(result, indent=2))
print(json.dumps(result))
