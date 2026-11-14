"""SAPIEN instruments reconstructed from local AtomBridge visual assemblies."""
import json
import math
import numpy as np
import sapien

from .config import asset_root
from .instrument_assets import MODELS


def add_collision(builder, record, local=None):
    pose = sapien.Pose(np.asarray(record['matrix'], dtype=np.float32))
    if local is not None:
        pose = local * pose
    kind, size = record['kind'], record['size']
    if kind == 'box':
        builder.add_box_collision(pose=pose, half_size=size, density=120)
    elif kind == 'sphere':
        builder.add_sphere_collision(pose=pose, radius=size[0], density=120)
    elif kind == 'cylinder':
        pose = pose * sapien.Pose(q=[math.sqrt(.5), 0, math.sqrt(.5), 0])
        builder.add_cylinder_collision(pose=pose, radius=size[0], half_length=size[1], density=120)
    else:
        raise ValueError(f'Unsupported source primitive: {kind}')


def add_thermal_body_collision(builder, record, center):
    """Keep the source shell, with an explicit open rectangular plate cavity.

    Source boxes can cover the visible reaction-block opening. Clip their AABBs
    around the cavity; this remains a documented conservative shell approximation.
    """
    if record['kind'] != 'box':
        return add_collision(builder, record)
    matrix = np.asarray(record['matrix'])
    extent = np.abs(matrix[:3, :3]) @ np.asarray(record['size'])
    low, high = matrix[:3, 3]-extent, matrix[:3, 3]+extent
    cut_low = np.array([center[0]-.061, center[1]-.042, .178])
    cut_high = np.array([center[0]+.061, center[1]+.042, .40])
    overlap_low, overlap_high = np.maximum(low, cut_low), np.minimum(high, cut_high)
    if np.any(overlap_low >= overlap_high):
        return add_collision(builder, record)
    work_low, work_high = low.copy(), high.copy()
    for axis in range(3):
        for lower in (True, False):
            a, b = work_low.copy(), work_high.copy()
            if lower:
                b[axis] = overlap_low[axis]
                work_low[axis] = overlap_low[axis]
            else:
                a[axis] = overlap_high[axis]
                work_high[axis] = overlap_high[axis]
            if np.all(b-a > 1e-6):
                builder.add_box_collision(half_size=(b-a)/2, pose=sapien.Pose((a+b)/2), density=120)


def build_instrument(scene, station, initial):
    from .scene import actor, box, material, DARK
    s = station
    folder = asset_root() / 'instruments' / MODELS[s.task]
    if not (folder / 'model.json').is_file():
        raise FileNotFoundError('Run sci-sim import-instruments --source ../atombridge '
                                'or explicitly select --instruments procedural')
    data = json.loads((folder / 'model.json').read_text())
    s.asset_metadata = {'model': data['model'], 'source': 'AtomBridge local reconstruction'}
    s.instrument_size = tuple(data['size_xyz_m'])
    if s.task == 'mix_transfer':
        def vortex(b):
            b.add_visual_from_file(str(folder / 'body.glb'))
            low, high = np.array(data['body_bounds']['body'])
            b.add_box_collision(half_size=(high-low)/2, pose=sapien.Pose((high+low)/2))
            for record in data['collisions']:
                add_collision(b, record)
        s.machine = actor(scene, s.prefix + data['model'], vortex, initial)
        s.mixer_local = np.array(data['sites']['platform-function']) + [0, 0, .024]
        return s

    w, d, h = s.instrument_size
    s.lid_depth, s.lid_height, s.machine_height = d, h, h
    builder = scene.create_articulation_builder()
    root = builder.create_link_builder()
    root.set_name(s.prefix + 'instrument_body')
    root.add_visual_from_file(str(folder / 'body.glb'))
    hinge = data['hinge']
    # Imported models have a horizontal X hinge; retain the measured pivot.
    if not np.allclose(hinge['axis'], [1, 0, 0], atol=1e-5):
        raise ValueError('Instrument hinge requires an explicit axis conversion')
    s.hinge_frame = sapien.Pose(hinge['pivot'])
    lid = builder.create_link_builder(root)
    lid.set_name(s.prefix + 'lid')
    lid.set_joint_name(s.prefix + 'lid_hinge')
    # Open beyond vertical so gravity holds the lid against its operating stop.
    lid.set_joint_properties('revolute', limits=[[-1.65, 0]],
                            pose_in_parent=s.hinge_frame, pose_in_child=sapien.Pose(),
                            friction=1.0, damping=.6)
    lid.add_visual_from_file(str(folder / 'lid.glb'), pose=s.hinge_frame.inv())
    knob = None
    if s.task == 'thermal_cycler' and 'knob' in data:
        info = data['knob']
        axis = np.asarray(info['axis'])
        axis /= np.linalg.norm(axis)
        tangent = np.cross(axis, [1, 0, 0])
        tangent /= np.linalg.norm(tangent)
        matrix = np.eye(4)
        matrix[:3, :3] = np.c_[axis, tangent, np.cross(axis, tangent)]
        matrix[:3, 3] = info['pivot']
        s.knob_frame = sapien.Pose(matrix)
        knob = builder.create_link_builder(lid)
        knob.set_name(s.prefix + 'knob')
        knob.set_joint_name(s.prefix + 'knob_rotation')
        knob.set_joint_properties('revolute', limits=[info['limits']],
            pose_in_parent=s.hinge_frame.inv()*s.knob_frame,
            pose_in_child=sapien.Pose(), friction=.006, damping=.002)
        knob.add_visual_from_file(str(folder/'knob.glb'), pose=s.knob_frame.inv())
    for record in data['collisions']:
        group = record['group']
        target, local = ((root, None) if group == 'body' else
                         (knob, s.knob_frame.inv()) if group == 'knob' else
                         (lid, s.hinge_frame.inv()))
        if s.task == 'thermal_cycler' and group == 'body':
            block_center = np.mean(data['body_bounds']['reaction-block'], axis=0)
            add_thermal_body_collision(target, record, block_center)
        else:
            add_collision(target, record, local)
    # An explicit narrow grasp handle supplements the source geometry for 2F-85.
    handle_point = np.array(data['sites'].get('lid-lever', data['sites'].get('lid')))
    handle_point[0] += .05
    if s.task == 'centrifuge':
        handle_point[0] = .10
        handle_point[1] = -.020
        handle_point[2] = data['body_bounds']['lid'][1][2] + .070
    s.handle_local = sapien.Pose(handle_point)
    hp = (s.hinge_frame.inv() * s.handle_local).p
    box(lid, (.026, .009, .009), hp, DARK, density=120)
    if s.task == 'centrifuge':
        stem = hp - np.array([0, 0, .035])
        box(lid, (.006, .006, .035), stem, DARK, density=120)
    if s.task == 'thermal_cycler':
        low, high = np.array(data['body_bounds']['reaction-block'])
        center = (low+high)/2
        # The plate seats into the block; resting it on the top of the visible
        # well rim would obstruct the closed heated lid by about 12 mm.
        bed_top = float(high[2])-.012
        root.add_box_collision(half_size=[.076, .050, .004],
                               pose=sapien.Pose([center[0], center[1], bed_top-.004]))
        s.target_local['plate'] = np.array([center[0], center[1], bed_top+.005])
        for y in (-.041, .041):
            box(root, (.061, .0015, .005), center+[0, y, bed_top-center[2]+.004], DARK)
        for x in (-.060, .060):
            box(root, (.0015, .041, .005), center+[x, 0, bed_top-center[2]+.004], DARK)
        s.button_rotation = np.array(data.get('button', {}).get('rotation', np.eye(3)))
        normal = s.button_rotation[:, 2]
        s.button_local = np.array(data['sites']['button']) + .012*normal
        plate = np.eye(4)
        plate[:3, :3] = s.button_rotation
        plate[:3, 3] = np.array(data['sites']['button']) + .004*normal
        root.add_box_visual(pose=sapien.Pose(plate), half_size=[.012, .011, .006],
                            material=material(DARK))
        root.add_box_collision(pose=sapien.Pose(plate), half_size=[.012, .011, .006])
    else:
        low, high = np.array(data['body_bounds']['rotor'])
        center = (low+high)/2
        # Task proxy: thirty vertical slots aligned around the visual rotor centre.
        # The manufacturer's tilted wells and spinning rotor are not simulated.
        bottom, radius = float(low[2]), .076
        root.add_cylinder_collision(radius=.105, half_length=.003,
            pose=sapien.Pose([center[0], center[1], bottom], [math.sqrt(.5), 0, math.sqrt(.5), 0]))
        for i in range(30):
            a = 2*math.pi*i/30
            p = center[:2] + radius*np.array([math.cos(a), math.sin(a)])
            root.add_box_collision(half_size=[.012, .001, .014],
                pose=sapien.Pose([p[0], p[1], bottom+.017], [math.cos(a/2), 0, 0, math.sin(a/2)]))
        for i, name in enumerate(('sample', 'balance')):
            a = math.pi/30 + i*math.pi
            s.target_local[name] = np.array([center[0]+radius*math.cos(a),
                                            center[1]+radius*math.sin(a), bottom+.027])
            # Close-fitting insertion guides for the two occupied, opposed wells.
            p = s.target_local[name]
            for j in range(16):
                theta = j*2*math.pi/16
                box(root, (.001, .0018, .010),
                    (p[0]+.0085*math.cos(theta), p[1]+.0085*math.sin(theta), bottom+.015),
                    DARK, q=(math.cos(theta/2), 0, 0, math.sin(theta/2)))
        s.button_local = [.10, -d/2+.035, .222]
        box(root, (.012, .011, .015), np.array(s.button_local)-[0, 0, .015], DARK)
    builder.initial_pose = initial
    s.machine = builder.build(name=s.prefix + data['model'], fix_root_link=True)
    s.lid = s.machine.links_map[s.prefix + 'lid']
    if knob is not None:
        s.knob = s.machine.links_map[s.prefix + 'knob']
    s.button = actor(scene, s.prefix + 'start_button',
                     lambda b: box(b, (.010, .009, .004), color=(.12, .68, .38, 1)), initial)
    return s
