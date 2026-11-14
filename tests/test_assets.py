from pathlib import Path
import xml.etree.ElementTree as ET
import pytest

from sci_sim.config import ARM_JOINTS, ROBOTS, asset_root


@pytest.mark.parametrize("model", ROBOTS)
def test_prepared_urdf_graph_and_meshes(model):
    path = asset_root() / "robots" / model / "robot.urdf"
    if not path.exists():
        pytest.skip("run sci-sim prepare-assets first")
    root = ET.parse(path).getroot()
    materials = [node.get("name") for node in root.findall("material")]
    assert len(materials) == len(set(materials))
    links = [node.get("name") for node in root.findall("link")]
    assert len(links) == len(set(links))
    joints = {node.get("name"): node for node in root.findall("joint")}
    assert set(ARM_JOINTS[model]) <= joints.keys()
    assert {"eef", "left_inner_finger_pad", "right_inner_finger_pad"} <= set(links)
    children = []
    for joint in joints.values():
        assert joint.find("parent").get("link") in links
        child = joint.find("child").get("link")
        assert child in links
        children.append(child)
    assert len(children) == len(set(children))
    assert len(set(links) - set(children)) == 1
    for mesh in root.findall(".//mesh"):
        assert not Path(mesh.get("filename")).is_absolute()
        assert (path.parent / mesh.get("filename")).is_file()
    assert path.with_suffix(".srdf").is_file()
