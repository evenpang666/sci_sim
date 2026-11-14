"""Check every actual robot collision link against the complete stand, without exclusions."""
import json
from pathlib import Path
import gymnasium as gym
from mplib.collision_detection import fcl
from sci_sim import env as _registered
from sci_sim.collision import collision_object
from sci_sim.config import ROBOTS

output = Path('outputs/revision4')
output.mkdir(parents=True, exist_ok=True)
report = []
for model in ROBOTS:
    env = gym.make('SciSim-MixTransfer-v0', lab_config={'robot': model},
                   obs_mode='state_dict', render_backend='none')
    env.reset(seed=42)
    base = env.unwrapped
    for step in range(21):
        if step:
            env.step(action)
        else:
            action = base.hold_action()
        if step not in (0, 20):
            continue
        stand = collision_object(base.station.furniture[1]._bodies[0], 'stand')
        contacts = []
        for side, agent in enumerate(base.active_agents):
            for link in agent.robot.get_links():
                obj = collision_object(link._bodies[0], f'arm{side}_{link.name}')
                if obj is None:
                    continue
                result = fcl.collide(obj, stand, fcl.CollisionRequest(enable_contact=True,
                                                                     num_max_contacts=100))
                for contact in result.get_contacts():
                    contacts.append({'arm': side, 'link': link.name,
                                     'penetration_m': float(contact.penetration_depth)})
        worst = max((c['penetration_m'] for c in contacts), default=0.)
        record = {'robot': model, 'step': step, 'contacts': contacts,
                  'max_penetration_m': worst, 'within_10um_numerical_tolerance': worst <= 1e-5}
        report.append(record)
        (output/'clearance.json').write_text(json.dumps(report, indent=2))
        print(json.dumps(record), flush=True)
    env.close()

assert all(r['within_10um_numerical_tolerance'] for r in report), report
