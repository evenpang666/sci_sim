"""GIF-inspired spin coating: contact manipulation and a bounded liquid model.

Liquid transfer is one metered dose, gated by tip geometry and dwell, not CFD.
A motor drives the physical chuck; the slide moves through friction, not pose edits.
"""

import math

import numpy as np
import sapien

from .scene import BLUE, DARK, WHITE, actor, box, cylinder, ring

GLASS = (0.70, 0.87, 0.93, 0.24)
BROWN = (0.27, 0.105, 0.025, 1)
CHUCK_TOP = 0.115
SLIDE_GRASP = np.array([0, 0, 0.009])
PIPETTE_GRASP = np.array([0, 0, 0.225])
PIPETTE_HOME = np.array([0, 0, 0.025])
LIQUID_Z = 0.060


def build_spin_coating(scene, s, initial):
    s.instrument_size = (0.30, 0.32, 0.19)
    s.target_local["slide"] = [0, 0, CHUCK_TOP]
    s.lid_depth, s.lid_height = 0.30, 0.175
    s.asset_metadata = {
        "source": "Procedural reconstruction from user spin-coating GIF",
        "size_xyz_m": list(s.instrument_size),
        "physics": "Contact-operated lid, free slide and pipette, motor-driven friction chuck",
        "liquid_model": "Single dose; 0.5 s tip immersion aspirates, 0.4 s above slide dispenses; visual drop and coating only",
        "spin": "6 rad/s illustrative speed; no calibrated rpm, vacuum, film-thickness or fluid solver",
        "slide": "25 x 25 x 2 mm square; raised source nest and chuck allow gripper access",
    }
    builder = scene.create_articulation_builder()
    root = builder.create_link_builder()
    root.set_name(s.prefix + "spin_body")
    box(root, (0.15, 0.16, 0.025), (0, 0, 0.025), WHITE)
    ring(root, 0.132, 0.020, 0.12, 0.110, WHITE, segments=32)
    # Front instrument panel sits below the lid handle.
    box(root, (0.15, 0.025, 0.045), (0, -0.135, 0.095), BLUE)
    box(root, (0.053, 0.001, 0.025), (-0.045, -0.161, 0.096), DARK, collision=False)
    for x in (-0.07, -0.05, -0.03):
        box(root, (0.005, 0.001, 0.012), (x, -0.163, 0.098), (0.45, 0.83, 0.95, 1), collision=False)
    cylinder(root, 0.120, 0.001, (0, 0, 0.052), (0.64, 0.69, 0.73, 1), collision=False)
    lid = builder.create_link_builder(root)
    lid.set_name(s.prefix + "spin_lid")
    lid.set_joint_name(s.prefix + "spin_lid_hinge")
    lid.set_joint_properties(
        "revolute",
        limits=[[-1.70, 0]],
        pose_in_parent=sapien.Pose([0, 0.138, 0.175]),
        pose_in_child=sapien.Pose(),
        friction=0.06,
        damping=0.15,
    )
    cylinder(lid, 0.139, 0.002, (0, -0.138, 0), GLASS, density=180)
    # Sideways handle follows the same local hinge convention as other tasks.
    box(lid, (0.028, 0.010, 0.010), (0, -0.256, 0.025), DARK, density=120)
    for angle in np.linspace(0, 2 * math.pi, 48, endpoint=False):
        box(
            lid,
            (0.002, 0.0092, 0.0015),
            (0.138 * math.cos(angle), -0.138 + 0.138 * math.sin(angle), 0.002),
            (0.48, 0.61, 0.68, 0.80),
            collision=False,
            q=(math.cos(angle / 2), 0, 0, math.sin(angle / 2)),
        )
    # Two hinge blocks make the rear pivot visible.
    for x in (-0.07, 0.07):
        box(lid, (0.012, 0.010, 0.008), (x, 0, 0), DARK, density=120)
    rotor = builder.create_link_builder(root)
    rotor.set_name(s.prefix + "spin_chuck")
    rotor.set_joint_name(s.prefix + "spin_motor")
    rotor.set_joint_properties(
        "revolute_unwrapped",
        limits=[[-np.inf, np.inf]],
        pose_in_parent=sapien.Pose([0, 0, 0.055], [math.sqrt(0.5), 0, -math.sqrt(0.5), 0]),
        pose_in_child=sapien.Pose(q=[math.sqrt(0.5), 0, -math.sqrt(0.5), 0]),
        damping=0.01,
    )
    cylinder(rotor, 0.009, 0.030, (0, 0, 0.030), DARK, density=1000)
    box(rotor, (0.008, 0.001, 0.0003), (0, 0, 0.0603), BLUE, collision=False)
    builder.initial_pose = initial
    s.machine = builder.build(name=s.prefix + "spin_coater", fix_root_link=True)
    s.lid = s.machine.links_map[s.prefix + "spin_lid"]
    s.rotor = s.machine.links_map[s.prefix + "spin_chuck"]
    s.spin_joint = next(j for j in s.machine.get_active_joints() if j.name.endswith("spin_motor"))
    s.spin_joint_index = list(s.machine.get_active_joints()).index(s.spin_joint)
    s.spin_joint.set_drive_properties(stiffness=0, damping=0.02, force_limit=0.03)
    s.button_local = np.array([0.115, -0.145, 0.145])
    s.button = actor(
        scene,
        s.prefix + "spin_start",
        lambda b: box(b, (0.014, 0.012, 0.004), color=(0.12, 0.72, 0.35, 1)),
        initial,
    )

    def nest(b):
        box(b, (0.050, 0.035, 0.003), (0, 0, 0.003), BLUE)
        cylinder(b, 0.010, 0.020, (0, 0, 0.020), WHITE)

    def stand(b):
        box(b, (0.045, 0.045, 0.004), (0, 0.012, 0.004), BLUE)
        box(b, (0.008, 0.008, 0.085), (0, 0.045, 0.089), WHITE)
        box(b, (0.035, 0.008, 0.005), (0, 0.035, 0.160), WHITE)
        for x in (-0.024, 0.024):
            box(b, (0.010, 0.030, 0.005), (x, 0, 0.160), WHITE)

    def beaker(b):
        cylinder(b, 0.040, 0.002, (0, 0, 0.002), GLASS)
        ring(b, 0.039, 0.002, 0.085, 0.0465, GLASS, segments=24)
        cylinder(b, 0.037, 0.027, (0, 0, 0.033), (0.29, 0.12, 0.03, 0.82), collision=False)
        for z in (0.025, 0.045, 0.065):
            box(b, (0.010, 0.001, 0.0006), (0, -0.041, z), WHITE, collision=False)

    s.fixtures["source_rack"] = actor(scene, s.prefix + "slide_nest", nest, initial)
    s.fixtures["target_rack"] = actor(scene, s.prefix + "pipette_stand", stand, initial)
    s.fixtures["beaker"] = actor(scene, s.prefix + "brown_solution", beaker, initial)

    def slide(b):
        box(b, (0.0125, 0.0125, 0.001), (0, 0, 0.001), (0.62, 0.83, 0.88, 0.70), density=2500)

    def pipette(b):
        cylinder(b, 0.0018, 0.025, (0, 0, 0.025), (0.8, 0.85, 0.8, 0.8), density=600)
        cylinder(b, 0.0035, 0.040, (0, 0, 0.090), BLUE, density=500)
        box(b, (0.010, 0.010, 0.050), (0, 0, 0.180), WHITE, density=400)
        cylinder(b, 0.021, 0.003, (0, 0, 0.143), BLUE, density=400)
        cylinder(b, 0.006, 0.007, (0, 0, 0.238), BLUE, density=400)
        box(b, (0.006, 0.0006, 0.010), (0, -0.0105, 0.185), DARK, collision=False)

    s.objects["slide"] = actor(scene, s.prefix + "glass_slide", slide, initial, dynamic=True)
    s.objects["pipette"] = actor(scene, s.prefix + "pipette", pipette, initial, dynamic=True)
    s.drop_visual = actor(
        scene,
        s.prefix + "dispensed_drop",
        lambda b: cylinder(b, 0.003, 0.003, color=BROWN, collision=False),
        initial,
    )
    s.puddle_visual = actor(
        scene,
        s.prefix + "brown_puddle",
        lambda b: cylinder(b, 0.006, 0.0005, color=BROWN, collision=False),
        initial,
    )
    s.coating_visual = actor(
        scene,
        s.prefix + "brown_coating",
        lambda b: box(b, (0.009, 0.009, 0.0002), color=BROWN, collision=False),
        initial,
    )
    s.dose_visual = actor(
        scene,
        s.prefix + "aspirated_dose",
        lambda b: cylinder(b, 0.002, 0.012, color=BROWN, collision=False),
        initial,
    )
    from .flash_evaporation import build_flash_evaporator

    s.flash = build_flash_evaporator(scene, s, initial)
    s.asset_metadata["flash_evaporator"] = s.flash.asset_metadata
    return s


def reset_station(s, layout):
    for name, obj in s.fixtures.items():
        obj.set_pose(sapien.Pose(s.world(layout[name])))
    s.machine.set_root_pose(sapien.Pose(s.world(layout["instrument"])))
    zeros = np.zeros((1, len(s.machine.get_active_joints())), dtype=np.float32)
    s.machine.set_qpos(zeros)
    s.machine.set_qvel(zeros)
    s.spin_joint.set_drive_velocity_target(0)
    s.button.set_pose(sapien.Pose(s.button_point))
    for name, key, offset in [
        ("slide", "source_rack", [0, 0, 0.0405]),
        ("pipette", "target_rack", PIPETTE_HOME),
    ]:
        obj = s.objects[name]
        obj.set_pose(sapien.Pose(s.world(np.array(layout[key]) + offset)))
        obj.set_linear_velocity(np.zeros((1, 3)))
        obj.set_angular_velocity(np.zeros((1, 3)))
    from .flash_evaporation import reset_flash

    reset_flash(s)
    for obj in (s.drop_visual, s.coating_visual, s.dose_visual, s.puddle_visual):
        obj.set_pose(sapien.Pose(s.world([0, 0, -2])))


def measure_spin_facts(base, facts):
    from .env import array, xyz

    s = base.station
    slide, pipette = s.objects["slide"], s.objects["pipette"]
    facts.lid_open = s.lid_angle < -1.25
    facts.lid_closed = s.lid_angle > -0.07
    facts.loaded = bool(
        base.close_to(slide, s.instrument_point([0, 0, CHUCK_TOP]), 0.012, 0.008)
        and base.contact(slide, s.rotor)
        and not base.is_held(slide)
    )
    tip = xyz(pipette)
    upright = array(pipette.pose.to_transformation_matrix())[0, 2, 2] > 0.98
    cup = s.world(s.layout["beaker"])
    held = base.is_held(pipette)
    facts.tip_immersed = bool(
        held
        and upright
        and np.linalg.norm((tip - cup)[:2]) < 0.028
        and 0.018 < tip[2] - cup[2] < LIQUID_Z - 0.004
    )
    delta = tip - xyz(slide)
    facts.tip_over_slide = bool(
        held
        and upright
        and np.linalg.norm(delta[:2]) < 0.007
        and 0.015 < delta[2] < 0.045
        and facts.lid_open
        and facts.loaded
    )
    facts.pipette_returned = bool(
        not held
        and base.close_to(
            pipette, s.world(np.array(s.layout["target_rack"]) + PIPETTE_HOME), 0.012, 0.008
        )
        and base.contact(pipette, s.fixtures["target_rack"])
    )
    facts.spinning = bool(abs(float(s.machine.get_qvel()[0, s.spin_joint_index].item())) > 0.5)
    facts.button_contact = any(
        base.contact(link, s.button)
        for a in base.active_agents
        for link in (a.finger1_link, a.finger2_link)
    )
    # Rotation of the correctly seated slide is intentional; the parked pipette must be stable.
    facts.stable = bool(
        np.linalg.norm(array(pipette.linear_velocity)) < 0.025
        and np.linalg.norm(array(pipette.angular_velocity)) < 0.25
    )

    from .flash_evaporation import measure_flash_facts

    measure_flash_facts(base, facts)


def update_visuals(s, state):
    """Only decorative, non-colliding liquid markers follow the physical labware."""
    from .flash_evaporation import update_flash_visual

    update_flash_visual(s, state)
    hidden = sapien.Pose(s.world([0, 0, -2]))
    dose = s.objects["pipette"].pose.sp * sapien.Pose([0, 0, 0.025])
    s.dose_visual.set_pose(dose if state.has_aspirated and not state.has_dispensed else hidden)
    falling = state.has_dispensed and state.drop_age < 0.25
    if falling:
        z = 0.030 * (1 - state.drop_age / 0.25) + 0.003
        s.drop_visual.set_pose(s.objects["slide"].pose.sp * sapien.Pose([0, 0, z]))
    else:
        s.drop_visual.set_pose(hidden)
    s.puddle_visual.set_pose(
        s.objects["slide"].pose.sp * sapien.Pose([0, 0, 0.0026])
        if state.has_dispensed and not falling and state.run_dwell < 0.2
        else hidden
    )
    s.coating_visual.set_pose(
        s.objects["slide"].pose.sp * sapien.Pose([0, 0, 0.0025])
        if state.has_dispensed and not falling and state.run_dwell >= 0.2
        else hidden
    )


def run_spin_coating(expert):
    from .planning import PlanningFailure, downward, sp

    s, arm = expert.base.station, 0
    expert.move(0, downward(s.world([-0.45, -0.25, 1.20])))
    expert.operate_lid(True)
    expert.move(1, downward(s.world([0.45, -0.25, 1.20])))

    def pick(name, offset, support):
        obj = s.objects[name]
        target = sp(obj.pose) * downward(offset)
        expert.gripper(arm, False)
        expert.move(arm, sapien.Pose(target.p + [0, 0, 0.12], target.q))
        expert.move(arm, target, linear=True, touch=[obj])
        expert.gripper(arm, True)
        if not expert.base.is_held(obj):
            raise PlanningFailure(name + ": grasp_not_established")
        expert.payloads[arm] = obj
        expert.payload_contacts[arm] = [support]
        expert.move(arm, sapien.Pose(target.p + [0, 0, 0.14], target.q), linear=True)
        expert.payload_contacts[arm] = []
        return obj

    def transfer(obj, target, clearance=0.12, support=None, release=False):
        relative = sp(obj.pose).inv() * sp(expert.agents[arm].tcp.pose)
        goal = sapien.Pose(target) * relative
        expert.move(arm, sapien.Pose(goal.p + [0, 0, clearance], goal.q))
        relative = sp(obj.pose).inv() * sp(expert.agents[arm].tcp.pose)
        goal = sapien.Pose(target) * relative
        expert.move(arm, sapien.Pose(goal.p + [0, 0, 0.06], goal.q), linear=True)
        relative = sp(obj.pose).inv() * sp(expert.agents[arm].tcp.pose)
        goal = sapien.Pose(target) * relative
        expert.payload_contacts[arm] = [] if support is None else [support]
        expert.move(arm, goal, linear=True)
        if release:
            if obj is s.objects["slide"]:
                # Relax from the measured finger angle: snapping fully open can
                # flick the lightweight square substrate off the narrow chuck.
                agent = expert.agents[arm]
                names = [j.name for j in agent.robot.get_active_joints()]
                index = names.index("left_outer_knuckle_joint")
                measured = float(agent.robot.get_qpos()[0, index].item())
                for opening in np.linspace(measured, expert.OPEN, 21)[1:]:
                    expert.grippers[arm] = float(opening)
                    expert.hold(0.05)
                expert.hold(0.3)
            else:
                expert.gripper(arm, False)
            expert.payloads[arm] = None
            expert.payload_contacts[arm] = []
            expert.move(
                arm, sapien.Pose(goal.p + [0, 0, clearance], goal.q), linear=True, touch=[obj]
            )
            expert.hold(0.5)
        return goal

    slide = pick("slide", SLIDE_GRASP, s.fixtures["source_rack"])
    transfer(slide, s.instrument_point([0, 0, CHUCK_TOP + 0.001]), support=s.rotor, release=True)
    if not (expert.base.task_state.has_loaded and expert.base.measure_facts().loaded):
        raise PlanningFailure("slide_not_loaded")
    pipette = pick("pipette", PIPETTE_GRASP, s.fixtures["target_rack"])
    goal = transfer(pipette, s.world(np.array(s.layout["beaker"]) + [0, 0, 0.043]))
    expert.hold(0.7)
    if not expert.base.task_state.has_aspirated:
        raise PlanningFailure("aspiration_not_detected")
    expert.move(arm, sapien.Pose(goal.p + [0, 0, 0.14], goal.q), linear=True)
    goal = transfer(pipette, s.instrument_point([0, 0, CHUCK_TOP + 0.030]))
    expert.hold(0.7)
    if not expert.base.task_state.has_dispensed:
        raise PlanningFailure("drop_not_dispensed")
    expert.move(arm, sapien.Pose(goal.p + [0, 0, 0.14], goal.q), linear=True)
    transfer(
        pipette,
        s.world(np.array(s.layout["target_rack"]) + PIPETTE_HOME + [0, 0, 0.001]),
        support=s.fixtures["target_rack"],
        clearance=0.23,
        release=True,
    )
    if not expert.base.measure_facts().pipette_returned:
        raise PlanningFailure("pipette_not_returned")
    expert.move(0, downward(s.world([-0.45, -0.25, 1.20])))
    expert.operate_lid(False)
    expert.press_start()

    # Continue with the same coated square substrate; the spin stage is not a
    # terminal success. Wait for the measured rotor/slide motion to stop.
    for _ in range(100):
        if (
            expert.base.task_state.has_spun
            and expert.base.measure_facts().rotor_stopped
            and expert.base.measure_facts().stable
        ):
            break
        expert.hold(0.05)
    else:
        raise PlanningFailure("rotor_did_not_stop")
    expert.operate_lid(True)
    expert.operate_lid(True, instrument=s.flash)
    arm = 1
    slide = pick("slide", SLIDE_GRASP, s.rotor)
    from .flash_evaporation import TRAY_Z

    # Release above the flat tray so the fingertip pads cannot hit its surface.
    transfer(
        slide,
        s.flash.instrument_point([0, 0, TRAY_Z + 0.014]),
        support=s.flash.machine.get_links()[0],
        release=True,
    )
    if not (expert.base.task_state.has_flash_loaded and expert.base.measure_facts().flash_loaded):
        raise PlanningFailure("slide_not_on_flash_tray")
    expert.operate_lid(False, instrument=s.flash)
    expert.press_start(instrument=s.flash)
