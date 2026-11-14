"""Reconstruct visual assemblies from local AtomBridge MJCF data, without executing it.

Only mesh/material/body/frame transforms are read. MuJoCo plugins, actuators,
scripts and source instructions are not executed. SAPIEN physics is built separately.
"""
from hashlib import sha256
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import trimesh
from PIL import Image
from scipy.spatial.transform import Rotation

from .config import asset_root

MODELS = {
    'mix_transfer': 'vortex_mixer_genie_2',
    'thermal_cycler': 'thermal_cycler_biorad_c1000',
    'centrifuge': 'centrifuge_eppendorf_5430',
}


def transform(node):
    matrix = np.eye(4)
    matrix[:3, 3] = np.fromstring(node.get('pos', '0 0 0'), sep=' ')
    if node.get('quat'):
        w, x, y, z = np.fromstring(node.get('quat'), sep=' ')
        matrix[:3, :3] = Rotation.from_quat([x, y, z, w]).as_matrix()
    elif node.get('euler'):
        matrix[:3, :3] = Rotation.from_euler('xyz', np.fromstring(node.get('euler'), sep=' ')).as_matrix()
    return matrix


def reconstruct(source, output=None):
    source = Path(source).resolve()
    output = output or asset_root() / 'instruments'
    output.mkdir(parents=True, exist_ok=True)
    manifest = {'source': 'User-provided local AtomBridge assets', 'models': {}}
    for task, name in MODELS.items():
        xml_path = source / 'model/instrument' / (name + '.xml')
        root = ET.parse(xml_path).getroot()
        meshes = {n.get('name'): n for n in root.findall('asset/mesh')}
        materials = {n.get('name'): n for n in root.findall('asset/material')}
        textures = {n.get('name'): n for n in root.findall('asset/texture')}
        hashes = {str(xml_path.relative_to(source)): sha256(xml_path.read_bytes()).hexdigest()}
        parts, sites, joint_info, knob_info, button_info, body_bounds, collisions = [], {}, {}, {}, {}, {}, []

        def walk(node, parent, group='body', body_name='body'):
            if node.tag == 'replicate':
                # Replicates contain rotor collision pieces/sites, not visual meshes.
                return
            current = parent @ transform(node)
            if node.tag == 'body':
                body_name = node.get('name', body_name)
                if body_name == 'start-button':
                    sites['button'] = current[:3, 3].copy()
                    button_info['rotation'] = current[:3, :3].tolist()
                    return  # Rebuilt as a contact button in SAPIEN.
                if body_name == 'lid':
                    group = 'lid'
                    joint = node.find('joint')
                    pivot = np.fromstring(joint.get('pos', '0 0 0'), sep=' ')
                    axis = np.fromstring(joint.get('axis'), sep=' ')
                    axis /= np.linalg.norm(axis)
                    closed = float(joint.get('range').split()[0])
                    joint_info.update(pivot=(current @ np.r_[pivot, 1])[:3],
                                      axis=-(current[:3, :3] @ axis), travel=abs(closed))
                    rotation = np.eye(4)
                    rotation[:3, :3] = Rotation.from_rotvec(axis * closed).as_matrix()
                    rotation[:3, 3] = pivot - rotation[:3, :3] @ pivot
                    current = current @ rotation
                if body_name == 'lid-force-knob':
                    group = 'knob'
                    joint = node.find('joint')
                    pivot = np.fromstring(joint.get('pos', '0 0 0'), sep=' ')
                    axis = np.fromstring(joint.get('axis'), sep=' ')
                    knob_info.update(pivot=(current @ np.r_[pivot, 1])[:3],
                                     axis=current[:3, :3] @ axis,
                                     limits=np.fromstring(joint.get('range'), sep=' ').tolist())
            if node.tag == 'site':
                sites[node.get('name')] = current[:3, 3].copy()
            if node.tag == 'geom' and node.get('class') == 'collision' and not node.get('mesh'):
                collisions.append({'group': group, 'kind': node.get('type'),
                                   'size': np.fromstring(node.get('size'), sep=' ').tolist(),
                                   'matrix': current.copy()})
            if node.tag == 'geom' and node.get('class') == 'visual' and node.get('mesh'):
                mesh_node = meshes[node.get('mesh')]
                filename = source / 'assets' / mesh_node.get('file')
                hashes[str(filename.relative_to(source))] = sha256(filename.read_bytes()).hexdigest()
                mesh = trimesh.load(filename, force='mesh', process=False)
                mesh.apply_scale(np.fromstring(mesh_node.get('scale', '1 1 1'), sep=' '))
                mat = materials[node.get('material')]
                color = np.fromstring(mat.get('rgba', '1 1 1 1'), sep=' ')
                texture = None
                if mat.get('texture'):
                    file = source / 'assets' / textures[mat.get('texture')].get('file')
                    hashes[str(file.relative_to(source))] = sha256(file.read_bytes()).hexdigest()
                    texture = Image.open(file).convert('RGBA')
                uv = getattr(mesh.visual, 'uv', None)
                mesh.visual = trimesh.visual.TextureVisuals(
                    uv=uv, material=trimesh.visual.material.PBRMaterial(
                        baseColorFactor=(color * 255).astype(np.uint8), baseColorTexture=texture,
                        metallicFactor=float(mat.get('metallic', '0')),
                        roughnessFactor=float(mat.get('roughness', '1'))))
                mesh.apply_transform(current)
                parts.append((group, body_name, mesh))
            for child in node:
                if child.tag in ('body', 'frame', 'geom', 'site', 'replicate'):
                    walk(child, current, group, body_name)

        walk(root.find('worldbody'), np.eye(4))
        bounds = np.array([m.bounds for _, _, m in parts])
        low, high = bounds[:, 0].min(0), bounds[:, 1].max(0)
        offset = np.r_[(low[:2] + high[:2]) / 2, low[2]]
        for group, body, mesh in parts:
            mesh.apply_translation(-offset)
            body_bounds.setdefault(body, []).append(mesh.bounds)
        dimensions = high - low
        for collision in collisions:
            collision['matrix'][:3, 3] -= offset
            collision['matrix'] = collision['matrix'].tolist()
        folder = output / name
        folder.mkdir(parents=True, exist_ok=True)
        for group in ('body', 'lid', 'knob'):
            collection = [m for g, _, m in parts if g == group]
            if collection:
                trimesh.Scene(collection).export(folder / (group + '.glb'))
        record = {'model': name, 'size_xyz_m': dimensions.tolist(), 'source_hashes': hashes,
                  'physics': 'Reconstructed primitive collisions; main lid and C1000 pressure knob articulated; other secondary joints frozen',
                  'body_bounds': {k: [np.array(v)[:, 0].min(0).tolist(),
                                      np.array(v)[:, 1].max(0).tolist()] for k, v in body_bounds.items()},
                  'sites': {k: (v - offset).tolist() for k, v in sites.items()},
                  'collisions': collisions}
        if joint_info:
            record['hinge'] = {'pivot': (joint_info['pivot'] - offset).tolist(),
                               'axis': joint_info['axis'].tolist(), 'travel': joint_info['travel']}
        if knob_info:
            record['knob'] = {'pivot': (knob_info['pivot']-offset).tolist(),
                              'axis': knob_info['axis'].tolist(), 'limits': knob_info['limits']}
        if button_info:
            record['button'] = button_info
        (folder / 'model.json').write_text(json.dumps(record, indent=2))
        manifest['models'][task] = record
    (output / 'manifest.json').write_text(json.dumps(manifest, indent=2))
    return {task: {'model': r['model'], 'size_xyz_m': r['size_xyz_m'], 'hinge': r.get('hinge')}
            for task, r in manifest['models'].items()}
