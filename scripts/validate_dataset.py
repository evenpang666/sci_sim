"""Diagnostic RGB-D round trip, explicitly not a successful task demonstration."""
from copy import deepcopy
from pathlib import Path
import argparse
import json
import gymnasium as gym
import numpy as np
from sci_sim import env as _registered
from sci_sim.data import LeRobotRecorder, metric_depth_mm


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--root', type=Path, required=True)
    args = parser.parse_args()
    env = gym.make('SciSim-MixTransfer-v0', lab_config={'width': 160, 'height': 120})
    obs, _ = env.reset(seed=42)
    recorder = LeRobotRecorder(env, args.root, 'local/scisim_diagnostic')
    expected = {}
    try:
        for i in range(3):
            before = deepcopy(obs)
            action = env.unwrapped.hold_action()
            obs, reward, done, truncated, info = env.step(action)
            recorder(before, action, reward, done, truncated, info)
            for cam, sensor in before['sensor_data'].items():
                expected[f'{i}_{cam}'] = metric_depth_mm(sensor['depth'][0], 2 if 'wrist' in cam else 4)
        # Deliberately bypass the success-only benchmark recorder for this labeled I/O test.
        recorder.dataset.save_episode()
    finally:
        recorder.close()
        env.close()
    np.savez_compressed(args.root / 'expected_depth.npz', **expected)
    (args.root / 'DIAGNOSTIC_ONLY.json').write_text(json.dumps({
        'purpose': 'Three stationary frames for writer round-trip verification only',
        'successful_demonstration': False, 'seed': 42}))
    print('DIAGNOSTIC_WRITE_OK', flush=True)


if __name__ == '__main__':
    main()
