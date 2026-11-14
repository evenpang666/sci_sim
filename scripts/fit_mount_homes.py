"""Offline collision-aware IK for the 45-degree stand's initial arm configurations."""
import json
from pathlib import Path
import gymnasium as gym
import mplib
import numpy as np
from sci_sim import env as registered
from sci_sim.config import ROBOTS
from sci_sim.planning import DualArmExpert, downward, mp, sp

result = {}
for model in ROBOTS:
    env = gym.make('SciSim-MixTransfer-v0', lab_config={'robot': model},
                   obs_mode='state_dict', render_backend='none')
    obs, _ = env.reset(seed=42)
    expert = DualArmExpert(env, obs)
    result[model] = []
    for side, agent in enumerate(env.unwrapped.active_agents):
        planner = expert.planner(side)
        qpos = registered.array(agent.robot.get_qpos())[0]
        mask = np.ones(len(qpos), dtype=bool)
        mask[expert.indices[side]] = False
        goal = downward([-.38 if side == 0 else .38, -.20, 1.14])
        target = sp(agent.robot.pose).inv() * goal
        mplib.set_global_seed(12)
        status, solution = planner.IK(mp(target), qpos, mask=mask, n_init_qpos=50, return_closest=True)
        if status != 'Success':
            raise RuntimeError((model, side, status))
        result[model].append(np.asarray(solution)[expert.indices[side]].tolist())
        agent.reset(np.asarray(solution, dtype=np.float32))
    print(model, result[model], flush=True)
    env.close()
Path('outputs/revision2/mount_homes.json').write_text(json.dumps(result, indent=2))
