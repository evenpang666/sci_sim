import numpy as np
import trimesh

from sci_sim.config import ARM_BASES, STAND_CROSSBAR_Z, STAND_TUBE_DIAMETER
from sci_sim.stand_geometry import crossbar_geometry


def test_closed_crossbar_cut_planes_coincide_with_mounts():
    vertices, faces, _, _ = crossbar_geometry()
    mesh = trimesh.Trimesh(vertices=vertices, faces=faces, process=False)
    assert mesh.is_watertight and mesh.is_winding_consistent
    assert mesh.volume > 0
    world = vertices + [0, -.74, STAND_CROSSBAR_Z]
    n = (len(vertices)-2)//2
    for side, base in enumerate(ARM_BASES):
        normal = np.array([-1 if side == 0 else 1, 0, -1])/np.sqrt(2)
        signed = (world-base) @ normal
        assert signed.max() < 1e-8  # No crossbar material beyond either mounting plane.
        np.testing.assert_allclose(signed[side*n:(side+1)*n], 0, atol=1e-8)
        # The ~151 mm UR7e mounting footprint lies inside the elliptical cut face.
        theta = np.linspace(0, 2*np.pi, 100)
        radial_y = .0756*np.cos(theta)
        radial_z = .0756*np.sin(theta)/np.sqrt(2)
        assert np.max(radial_y**2+radial_z**2) < (STAND_TUBE_DIAMETER/2)**2
