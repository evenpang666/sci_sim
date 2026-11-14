"""Local AtomBridge labware meshes, fitted to the benchmark's collision dimensions."""
from hashlib import sha256
import json
from pathlib import Path
import numpy as np
import trimesh
from .config import asset_root


def reconstruct_labware(source):
    source = Path(source).resolve()
    output = asset_root()/'labware'
    output.mkdir(parents=True, exist_ok=True)
    records = {}
    tube_dir = source/'assets/container/centrifuge_1-5ml_screw_vis'
    inputs = {'tube_body': tube_dir/'centrifuge_1-5ml_screw_body.STL',
              'tube_cap': tube_dir/'centrifuge_1-5ml_screw_cap.STL',
              'plate': source/'assets/container/pcr_plate_96well_vis.obj'}
    for name, path in inputs.items():
        mesh = trimesh.load(path, force='mesh', process=False)
        low, high = mesh.bounds
        # Preserve source silhouettes/wells, adapt outer dimensions to the task proxy.
        size, center = (([.012, .012, .042], [0, 0, -.003]) if name == 'tube_body' else
                        ([.015, .015, .006], [0, 0, .021]) if name == 'tube_cap' else
                        ([.114, .076, .012], [0, 0, .001]))
        mesh.vertices = (mesh.vertices-(low+high)/2) * (np.array(size)/(high-low)) + center
        if name == 'tube_cap':
            # Source cap is authored upright; assembled screw cap faces downward.
            mesh.vertices[:, 2] = 2*center[2]-mesh.vertices[:, 2]
            mesh.faces = mesh.faces[:, ::-1]
        mesh.export(output/(name+'.obj'))
        records[name] = {'source': str(path.relative_to(source)),
                         'sha256': sha256(path.read_bytes()).hexdigest(),
                         'fitted_size_m': size, 'center_m': center}
    (output/'manifest.json').write_text(json.dumps(records, indent=2))
    return records
