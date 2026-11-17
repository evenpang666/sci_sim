"""Contact-operated flash-evaporation chamber following the user's GIF.

The tray perforations are visual. Evaporation is a timed process with a status
lamp; this module does not simulate vacuum, heating or solvent transport.
"""

import math

import numpy as np
import sapien

from .scene import BLUE, DARK, WHITE, Station, actor, box, cylinder

TRAY_Z = 0.145


def build_flash_evaporator(scene, station, initial):
    s = Station("spin_coating", station.origin, station.prefix + "flash_")
    s.instrument_size = (0.28, 0.28, 0.19)
    s.lid_depth, s.lid_height = 0.28, 0.175
    s.objects = station.objects
    s.furniture = station.furniture
    builder = scene.create_articulation_builder()
    root = builder.create_link_builder()
    root.set_name(s.prefix + "body")
    box(root, (0.14, 0.14, 0.06), (0, 0, 0.06), (0.055, 0.19, 0.48, 1))
    box(root, (0.125, 0.125, 0.0125), (0, 0, 0.1325), (0.55, 0.62, 0.67, 1))
    # A grid of dark discs depicts the perforated removable metal tray. The
    # support collision is continuous so a 25 mm substrate cannot fall in a hole.
    for x in np.linspace(-0.108, 0.108, 13):
        for y in np.linspace(-0.108, 0.108, 13):
            cylinder(root, 0.0032, 0.0002, (x, y, TRAY_Z + 0.00025), DARK, collision=False)
    for x in (-0.132, 0.132):
        box(root, (0.007, 0.14, 0.012), (x, 0, 0.143), WHITE)
    for y in (-0.132, 0.132):
        box(root, (0.125, 0.007, 0.012), (0, y, 0.143), WHITE)
    box(root, (0.05, 0.025, 0.08), (0.080, -0.165, 0.08), BLUE)
    box(root, (0.053, 0.001, 0.025), (-0.038, -0.141, 0.070), DARK, collision=False)
    for x in (-0.066, -0.043, -0.020):
        box(root, (0.006, 0.001, 0.009), (x, -0.1425, 0.072), (0.6, 0.85, 0.93, 1), collision=False)
    lid = builder.create_link_builder(root)
    lid.set_name(s.prefix + "lid")
    lid.set_joint_name(s.prefix + "hinge")
    lid.set_joint_properties(
        "revolute",
        limits=[[-1.70, 0]],
        pose_in_parent=sapien.Pose([0, 0.128, 0.175]),
        pose_in_child=sapien.Pose(),
        friction=0.06,
        damping=0.15,
    )
    box(lid, (0.139, 0.128, 0.002), (0, -0.128, 0), (0.69, 0.85, 0.91, 0.22), density=180)
    for x in (-0.128, 0.128):
        box(lid, (0.011, 0.128, 0.006), (x, -0.128, 0), WHITE, density=120)
    for y in (-0.246, -0.010):
        box(lid, (0.117, 0.010, 0.006), (0, y, 0), WHITE, density=120)
    for angle in np.linspace(0, 2 * math.pi, 48, endpoint=False):
        box(
            lid,
            (0.002, 0.0074, 0.0015),
            (0.111 * math.cos(angle), -0.128 + 0.111 * math.sin(angle), 0.003),
            BLUE,
            collision=False,
            q=(math.cos(angle / 2), 0, 0, math.sin(angle / 2)),
        )
    box(lid, (0.028, 0.010, 0.010), (0, -0.236, 0.025), DARK, density=120)
    builder.initial_pose = initial
    s.machine = builder.build(name=s.prefix + "evaporator", fix_root_link=True)
    s.lid = s.machine.links_map[s.prefix + "lid"]
    s.button_local = np.array([0.100, -0.180, 0.164])
    # A half-turn about the vertical keeps the pressing wrist away from its limit
    # after the flash-lid arc; the closed fingertips retain the same contact face.
    s.button_rotation = np.diag([-1.0, -1.0, 1.0])
    s.button = actor(
        scene,
        s.prefix + "start",
        lambda b: box(b, (0.014, 0.010, 0.004), color=(0.12, 0.72, 0.35, 1)),
        initial,
    )
    s.status_light = actor(
        scene,
        s.prefix + "running_light",
        lambda b: cylinder(b, 0.007, 0.001, color=(1, 0.52, 0.04, 1), collision=False),
        initial,
    )
    s.asset_metadata = {
        "source": "Procedural reconstruction from user flash-evaporation GIF",
        "size_xyz_m": list(s.instrument_size),
        "tray": "Perforated metal visual with continuous support collision",
        "process": "Contact start and timed evaporation; no vacuum/thermal/solvent solver",
    }
    return s


def reset_flash(station):
    s = station.flash
    s.layout = {"instrument": station.layout["flash"]}
    s.machine.set_root_pose(sapien.Pose(s.instrument_point([0, 0, 0])))
    s.machine.set_qpos(np.zeros((1, 1), dtype=np.float32))
    s.machine.set_qvel(np.zeros((1, 1), dtype=np.float32))
    s.button.set_pose(sapien.Pose(s.button_point))
    s.status_light.set_pose(sapien.Pose(s.world([0, 0, -2])))


def measure_flash_facts(base, facts):
    from .env import array

    s, flash = base.station, base.station.flash
    slide = s.objects["slide"]
    facts.rotor_stopped = bool(abs(float(s.machine.get_qvel()[0, s.spin_joint_index])) < 0.05)
    facts.flash_lid_open = flash.lid_angle < -1.25
    facts.flash_lid_closed = flash.lid_angle > -0.07
    facts.flash_loaded = bool(
        base.close_to(slide, flash.instrument_point([0, 0, TRAY_Z]), 0.018, 0.005)
        and base.contact(slide, flash.machine.get_links()[0])
        and not base.is_held(slide)
    )
    facts.flash_button_contact = any(
        base.contact(link, flash.button)
        for a in base.active_agents
        for link in (a.finger1_link, a.finger2_link)
    )
    if base.task_state.has_spun:
        facts.stable = bool(
            facts.stable
            and np.linalg.norm(array(slide.linear_velocity)) < 0.025
            and np.linalg.norm(array(slide.angular_velocity)) < 0.25
        )


def update_flash_visual(station, state):
    flash = station.flash
    flash.status_light.set_pose(
        sapien.Pose(flash.instrument_point([0.075, -0.180, 0.164]))
        if state.flash_running
        else sapien.Pose(flash.world([0, 0, -2]))
    )
