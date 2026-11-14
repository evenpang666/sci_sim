"""Build local, relocatable arm + Robotiq assets from upstream robot descriptions."""
from copy import deepcopy
from hashlib import sha256
from importlib.resources import files
from pathlib import Path
from urllib.parse import urlparse
import io
import json
import shutil
import xml.etree.ElementTree as ET
import zipfile
from unittest.mock import patch

import requests

from .config import ARM_JOINTS, ROBOTS, asset_root

UR_REV = "0eb1a861f7772fceb38510d90fa2f98ec668070c"
SOURCES = {
    "ur_description": f"https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/archive/{UR_REV}.zip",
    "xarm6": "https://github.com/haosulab/ManiSkill-XArm6/archive/refs/tags/v0.1.1.zip",
}


def download_zip(url: str, destination: Path) -> dict:
    """Extract only regular files under one archive root, without path traversal."""
    if urlparse(url).scheme != "https":
        raise ValueError("Asset downloads require HTTPS")
    destination.mkdir(parents=True, exist_ok=True)
    response = requests.get(url, timeout=180)
    response.raise_for_status()
    data = response.content
    with zipfile.ZipFile(io.BytesIO(data)) as archive:
        for entry in archive.infolist():
            if entry.is_dir():
                continue
            # GitHub archives have one leading directory.
            parts = Path(entry.filename).parts[1:]
            if not parts or ".." in parts or Path(*parts).is_absolute():
                raise ValueError(f"Unsafe archive entry: {entry.filename}")
            target = destination.joinpath(*parts).resolve()
            if not target.is_relative_to(destination.resolve()):
                raise ValueError(f"Archive escaped destination: {entry.filename}")
            if (entry.external_attr >> 16) & 0o170000 == 0o120000:
                raise ValueError("Symlinks are not accepted in robot archives")
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(archive.read(entry))
    return {"url": url, "sha256": sha256(data).hexdigest()}


def descendants(root: ET.Element, link: str) -> set[str]:
    found = {link}
    while True:
        new = {j.find("child").get("link") for j in root.findall("joint")
               if j.find("parent").get("link") in found}
        if new <= found:
            return found
        found |= new


def absolute_meshes(root: ET.Element, directory: Path, packages: dict[str, Path]):
    for mesh in root.findall(".//mesh"):
        filename = mesh.get("filename")
        if filename.startswith("package://"):
            package, relative = filename[10:].split("/", 1)
            path = packages[package] / relative
        else:
            path = directory / filename
        if not path.is_file():
            raise FileNotFoundError(path)
        mesh.set("filename", path.resolve().as_posix())


def make_srdf(robot: ET.Element, filename: Path, arm_joints: tuple[str, ...]):
    srdf = ET.Element("robot", name=robot.get("name"))
    group = ET.SubElement(srdf, "group", name="arm")
    for name in arm_joints:
        ET.SubElement(group, "joint", name=name)
    # UR has intermediate fixed flange frames; adjacent rigid bodies can span
    # several fixed joints and still have intentional mating-surface contact.
    clusters = {link.get("name"): {link.get("name")} for link in robot.findall("link")}
    for joint in robot.findall("joint"):
        if joint.get("type") == "fixed":
            parent, child = joint.find("parent").get("link"), joint.find("child").get("link")
            union = clusters[parent] | clusters[child]
            for name in union:
                clusters[name] = union
    pairs = set()
    for joint in robot.findall("joint"):
        a, b = joint.find("parent").get("link"), joint.find("child").get("link")
        for left in clusters[a]:
            for right in clusters[b]:
                if left != right:
                    pairs.add(tuple(sorted((left, right))))
    for left, right in sorted(pairs):
        ET.SubElement(srdf, "disable_collisions", link1=left, link2=right, reason="Adjacent")
    # The 2F-85 four-bar linkage has intentional internal contacts.
    gripper = sorted(descendants(robot, "robotiq_arg2f_base_link"))
    for i, a in enumerate(gripper):
        for b in gripper[i + 1:]:
            ET.SubElement(srdf, "disable_collisions", link1=a, link2=b, reason="Gripper linkage")
    ET.indent(srdf)
    ET.ElementTree(srdf).write(filename, encoding="utf-8", xml_declaration=True)


def prepare_assets(robots=ROBOTS, root: Path | None = None):
    import xacro

    root = (root or asset_root()).resolve()
    upstream = root / "upstream"
    manifest_path = root / "manifest.json"
    manifest = (json.loads(manifest_path.read_text()) if manifest_path.exists()
                else {"robot_sources": {}, "models": {}, "ur_revision": UR_REV})
    for name, url in SOURCES.items():
        folder = upstream / name
        receipt = folder / ".download.json"
        if receipt.exists():
            manifest["robot_sources"][name] = json.loads(receipt.read_text())
        else:
            record = download_zip(url, folder)
            receipt.write_text(json.dumps(record, indent=2))
            manifest["robot_sources"][name] = record
    ur = upstream / "ur_description"
    # Resolve ROS package names locally without modifying the upstream xacro files.
    def find_package(name):
        if name != "ur_description":
            raise ValueError(f"Unexpected xacro package dependency: {name}")
        return ur.as_posix()
    xarm_dir = upstream / "xarm6"
    donor = ET.parse(xarm_dir / "xarm6_robotiq.urdf").getroot()
    absolute_meshes(donor, xarm_dir, {})
    gripper_links = descendants(donor, "robotiq_arg2f_base_link")
    for model in robots:
        if model not in ROBOTS:
            raise ValueError(model)
        if model.startswith("ur"):
            with patch("xacro.substitution_args._eval_find", find_package):
                xml = xacro.process_file(str(ur / "urdf/ur.urdf.xacro"),
                                         mappings={"name": model, "ur_type": model}).toxml()
            robot = ET.fromstring(xml)
            absolute_meshes(robot, ur, {"ur_description": ur})
            flange = "tool0"
        elif model == "franka":
            panda = Path(str(files("mani_skill").joinpath("assets/robots/panda")))
            local = upstream / "panda"
            shutil.copytree(panda, local, dirs_exist_ok=True)
            robot = ET.parse(local / "panda_v2.urdf").getroot()
            absolute_meshes(robot, local, {})
            remove = descendants(robot, "panda_hand")
            for item in list(robot):
                if ((item.tag == "link" and item.get("name") in remove) or
                    (item.tag == "joint" and item.find("child").get("link") in remove)):
                    robot.remove(item)
            flange = "panda_link8"
            manifest["robot_sources"]["panda"] = {"source": "mani_skill==3.0.1 packaged panda_v2"}
        else:
            robot = ET.parse(xarm_dir / "xarm6_nogripper.urdf").getroot()
            absolute_meshes(robot, xarm_dir, {})
            flange = "link6"
            # Remove any existing TCP/camera frames; a single donor TCP is authoritative.
            for name in ("eef", "camera_link"):
                for item in list(robot):
                    if ((item.tag == "link" and item.get("name") == name) or
                        (item.tag == "joint" and item.find("child").get("link") == name)):
                        robot.remove(item)
        for item in donor:
            if item.tag == "material" and any(
                existing.get("name") == item.get("name") for existing in robot.findall("material")
            ):
                continue
            if ((item.tag == "link" and item.get("name") in gripper_links) or
                (item.tag == "joint" and item.find("parent").get("link") in gripper_links) or
                item.tag == "material"):
                robot.append(deepcopy(item))
        joint = ET.SubElement(robot, "joint", name="lab_gripper_mount", type="fixed")
        ET.SubElement(joint, "parent", link=flange)
        ET.SubElement(joint, "child", link="robotiq_arg2f_base_link")
        ET.SubElement(joint, "origin", xyz="0 0 0", rpy="0 0 0")
        robot.set("name", f"{model}_robotiq_2f85")
        target = root / "robots" / model
        target.mkdir(parents=True, exist_ok=True)
        urdf = target / "robot.urdf"
        # Convert absolute upstream paths to relative paths so the project can move.
        import os
        for mesh in robot.findall(".//mesh"):
            mesh.set("filename", Path(os.path.relpath(mesh.get("filename"), target)).as_posix())
        ET.indent(robot)
        ET.ElementTree(robot).write(urdf, encoding="utf-8", xml_declaration=True)
        make_srdf(robot, target / "robot.srdf", ARM_JOINTS[model])
        manifest["models"][model] = {"sha256": sha256(urdf.read_bytes()).hexdigest(),
                                    "flange": flange, "tcp": "eef", "gripper": "Robotiq 2F-85"}
    (root / "manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    return manifest

