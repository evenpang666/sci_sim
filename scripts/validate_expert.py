"""Run actual contact-based experts and retain failures as validation evidence."""
import argparse
import json
from pathlib import Path
import traceback
import pickle
from copy import deepcopy
from dataclasses import asdict

import gymnasium as gym
from sci_sim import env as _registered
from sci_sim.config import TASK_IDS
from sci_sim.planning import DualArmExpert, EpisodeComplete


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--task', choices=TASK_IDS, default='mix_transfer')
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--trace', action='store_true', help='Save physical state snapshots for rendering')
    args = parser.parse_args()
    env = gym.make(TASK_IDS[args.task], obs_mode='state_dict', render_backend='none')
    obs, _ = env.reset(seed=args.seed)
    expert = DualArmExpert(env, obs, seed=args.seed)
    snapshots = []
    phases = []
    def capture(*unused):
        phase = env.unwrapped.task_state.phase
        if not phases or phases[-1]['phase'] != phase:
            phases.append({'phase': phase, 'step': expert.steps})
        if expert.steps % 4 == 0:
            snapshots.append({'step': expert.steps, 'phase': env.unwrapped.task_state.phase,
                              'state': deepcopy(env.unwrapped.get_state_dict())})
    if args.trace:
        expert.record = capture
    result = {'task': args.task, 'seed': args.seed}
    try:
        expert.run()
    except EpisodeComplete:
        pass
    except Exception:
        result['error'] = traceback.format_exc()
    finally:
        result.update(steps=expert.steps, task_state=env.unwrapped.task_state.to_dict())
        result['phases'] = phases
        base = env.unwrapped
        result['facts'] = {key: bool(value) for key, value in asdict(base.measure_facts()).items()}
        result['objects'] = {k: _registered.xyz(v).tolist() for k, v in base.station.objects.items()}
        result['object_quaternions'] = {k: _registered.array(v.pose.q).tolist() for k, v in base.station.objects.items()}
        result['instrument_origin'] = base.station.instrument_point([0, 0, 0]).tolist()
        if base.task_name != 'mix_transfer':
            result['lid_angle'] = base.station.lid_angle
            result['knob_angle'] = base.station.knob_angle
        args.output.parent.mkdir(parents=True, exist_ok=True)
        snapshots.append({'step': expert.steps, 'phase': base.task_state.phase,
                          'state': deepcopy(base.get_state_dict())})
        with args.output.with_suffix('.pkl').open('wb') as f:
            pickle.dump({'task': args.task, 'seed': args.seed, 'frames': snapshots}, f)
        args.output.write_text(json.dumps(result, indent=2))
        print(json.dumps(result), flush=True)
        env.close()
    return 0 if result['task_state']['success'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
