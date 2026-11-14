"""Closed cylindrical crossbar with two real 45-degree mounting cuts."""
from hashlib import sha256
import numpy as np

from .config import ARM_BASES, STAND_TUBE_DIAMETER, STAND_CROSSBAR_Z, asset_root


def crossbar_geometry(segments=96):
    radius = STAND_TUBE_DIAMETER / 2
    theta = np.arange(segments) * (2 * np.pi / segments)
    y, z = radius * np.cos(theta), radius * np.sin(theta)
    # Plane normals are (-1,0,-1)/sqrt(2) and (1,0,-1)/sqrt(2).
    # Both planes pass through the actual robot mounting origins.
    left, right = np.array(ARM_BASES)
    left_x = left[0] - (STAND_CROSSBAR_Z + z - left[2])
    right_x = right[0] + (STAND_CROSSBAR_Z + z - right[2])
    vertices = np.vstack([np.c_[left_x, y, z], np.c_[right_x, y, z],
                          [left[0], 0, left[2]-STAND_CROSSBAR_Z],
                          [right[0], 0, right[2]-STAND_CROSSBAR_Z]])
    faces, normals = [], []
    # Each face corner gets its own normal index: smooth cylinder, flat end faces.
    normal_vectors = np.vstack([np.c_[np.zeros(segments), np.cos(theta), np.sin(theta)],
                                np.array([[-1, 0, -1], [1, 0, -1]]) / np.sqrt(2)])
    for i in range(segments):
        j = (i + 1) % segments
        faces.extend([[i, j, segments+j], [i, segments+j, segments+i],
                      [2*segments, j, i], [2*segments+1, segments+i, segments+j]])
        normals.extend([[i, j, j], [i, j, i], [segments]*3, [segments+1]*3])
    return vertices, np.asarray(faces), normal_vectors, np.asarray(normals)


def crossbar_mesh_file():
    vertices, faces, normals, indices = crossbar_geometry()
    lines = ['# Equal-diameter T support, closed cylinder with 45-degree end cuts.']
    lines.extend('v ' + ' '.join(f'{v:.10f}' for v in row) for row in vertices)
    lines.extend('vn ' + ' '.join(f'{v:.10f}' for v in row) for row in normals)
    lines.extend('f ' + ' '.join(f'{v+1}//{n+1}' for v, n in zip(face, normal))
                 for face, normal in zip(faces, indices))
    content = '\n'.join(lines) + '\n'
    path = asset_root() / 'generated' / ('crossbar_' + sha256(content.encode()).hexdigest()[:16] + '.obj')
    path.parent.mkdir(parents=True, exist_ok=True)
    if not path.exists():
        path.write_text(content, encoding='ascii')
    return str(path)
