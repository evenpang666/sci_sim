"""Front-opening drying cabinet and contact-based loading of nested glassware.

The requested task ends at door closure. No heating, vacuum or evaporation is
started or inferred; all dimensions approximate the user's reference GIF.
"""

import math

import numpy as np
import sapien

from .scene import DARK, WHITE, Station, box

SHELF_Z = 0.030
LOAD_LOCAL = np.array([0.05, -0.085, SHELF_Z])


def build_dryer(scene, station, initial):
    s = Station("slide_drying", station.origin, station.prefix + "dryer_")
    s.instrument_size = (0.38, 0.36, 0.52)
    s.objects = station.objects
    builder = scene.create_articulation_builder()
    root = builder.create_link_builder()
    root.set_name(s.prefix + "body")
    steel = (0.65, 0.70, 0.74, 1)
    blue = (0.16, 0.38, 0.72, 1)
    box(root, (0.19, 0.18, 0.01), (0, 0, 0.01), blue)
    box(root, (0.175, 0.16, 0.005), (0, 0, SHELF_Z - 0.005), steel)
    for x in (-0.1825, 0.1825):
        box(root, (0.0075, 0.18, 0.25), (x, 0, 0.27), blue)
    box(root, (0.175, 0.01, 0.25), (0, 0.17, 0.27), steel)
    box(root, (0.175, 0.18, 0.01), (0, 0, 0.51), blue)
    # Wire-shelf appearance over a continuous load-bearing collision surface.
    for x in np.linspace(-0.16, 0.16, 17):
        box(root, (0.0012, 0.155, 0.0006), (x, 0, SHELF_Z + 0.0007), WHITE, collision=False)
    for z in (0.08, 0.48):
        box(root, (0.172, 0.003, 0.004), (0, -0.174, z), (0.82, 0.28, 0.08, 1), collision=False)
    for x in (-0.172, 0.172):
        box(root, (0.004, 0.003, 0.20), (x, -0.174, 0.28), (0.82, 0.28, 0.08, 1), collision=False)
    # Left hinge opens toward the cleaner, leaving the right arm a clear loading aisle.
    # SAPIEN revolute axes are X: rotate the hinge frame to vertical -Z.
    hinge_q = [math.sqrt(0.5), 0, math.sqrt(0.5), 0]
    s.hinge_frame = sapien.Pose([-0.18, -0.184, 0.27], hinge_q)
    s.handle_local = sapien.Pose([0.145, -0.254, 0.27])
    lid = builder.create_link_builder(root)
    lid.set_name(s.prefix + "door")
    lid.set_joint_name(s.prefix + "door_hinge")
    lid.set_joint_properties(
        "revolute",
        limits=[[0, 1.8]],
        pose_in_parent=s.hinge_frame,
        pose_in_child=sapien.Pose(q=hinge_q),
        friction=0.4,
        damping=0.15,
    )
    box(lid, (0.164, 0.003, 0.209), (0.18, 0, 0.005), (0.68, 0.85, 0.91, 0.22), density=180)
    for x in (0.352, 0.008):
        box(lid, (0.018, 0.01, 0.25), (x, 0, 0), blue, density=100)
    for z in (-0.232, 0.232):
        box(lid, (0.154, 0.01, 0.018), (0.18, 0, z), blue, density=100)
    box(lid, (0.012, 0.012, 0.055), (0.325, -0.070, 0), DARK, density=150)
    for z in (-0.045, 0.045):
        box(lid, (0.008, 0.035, 0.006), (0.325, -0.035, z), DARK, density=150)
    # Small neutral panel; no temperature/vacuum value is asserted by this task.
    box(lid, (0.04, 0.001, 0.012), (0.065, -0.011, 0.232), (0.72, 0.72, 0.19, 1), collision=False)
    builder.initial_pose = initial
    s.machine = builder.build(name=s.prefix + "cabinet", fix_root_link=True)
    s.lid = s.machine.links_map[s.prefix + "door"]
    s.asset_metadata = {
        "source": "User-provided drying-cabinet GIF",
        "size_xyz_m": list(s.instrument_size),
        "door": "Vertical hinge, transparent window, contact-operated handle",
        "process": "Load cleaned beaker with rack and close door; no heating, vacuum or evaporation model",
    }
    return s


def reset_dryer(station):
    s = station.dryer
    s.layout = {"instrument": station.layout["dryer"]}
    s.machine.set_root_pose(sapien.Pose(s.instrument_point([0, 0, 0])))
    s.machine.set_qpos(np.zeros((1, 1), dtype=np.float32))
    s.machine.set_qvel(np.zeros((1, 1), dtype=np.float32))


def measure_drying_facts(base, facts):
    dryer = base.station.dryer
    beaker = base.station.objects["wash_beaker"]
    facts.dryer_open = dryer.lid_angle > 1.15
    facts.dryer_closed = abs(dryer.lid_angle) < 0.05
    facts.beaker_in_dryer = bool(
        base.close_to(beaker, dryer.instrument_point(LOAD_LOCAL), xy=0.018, z=0.008)
        and base.contact(beaker, dryer.machine.get_links()[0])
    )


def push_door_open(expert):
    """Push the partially opened handle with closed fingertips, without grasping."""
    from .planning import PlanningFailure, sp

    s, arm = expert.base.station.dryer, 1
    expert.operation = "push_dryer_door"
    angle = s.lid_angle

    def target(theta, offset):
        handle = s.handle_pose(float(theta))
        normal = handle.to_transformation_matrix()[:3, :3] @ np.array([0, -1, 0])
        pose = (
            handle
            * sapien.Pose(q=[0, math.sqrt(0.5), math.sqrt(0.5), 0])
            * sapien.Pose(q=[math.cos(math.pi / 8), 0, math.sin(math.pi / 8), 0])
        )
        return sapien.Pose(handle.p - offset * normal + [0, 0, 0.055], pose.q)

    # Close the empty gripper above the handle, then push its inner face.
    # No object is pinched or attached during this second stage.
    expert.gripper(arm, True)
    pose = target(angle, 0.035)
    expert.move(arm, sapien.Pose(pose.p + [0, 0, 0.08], pose.q), center_wrist=True)
    expert.move(arm, pose, linear=True, touch=[s.lid])
    fingers = [
        link
        for link in expert.agents[arm].robot.get_links()
        if link.name
        in (
            "left_inner_finger_pad",
            "right_inner_finger_pad",
            "left_inner_finger",
            "right_inner_finger",
        )
    ]
    for offset in np.linspace(0.035, 0.008, 15):
        expert.move(arm, target(s.lid_angle, offset), linear=True, touch=[s.lid])
        if any(expert.base.contact(finger, s.lid) for finger in fingers):
            break
    else:
        raise PlanningFailure("dryer_push_contact_missing")
    angle = s.lid_angle
    for theta in np.linspace(angle, 1.22, 35)[1:]:
        expert.move(arm, target(theta, offset), linear=True, touch=[s.lid])
    expert.hold(0.3)
    if not expert.base.measure_facts().dryer_open:
        raise PlanningFailure("dryer_push_did_not_open_door")
    current = sp(expert.agents[arm].tcp.pose)
    handle = s.handle_pose(s.lid_angle)
    upright = handle * sapien.Pose(q=[0, math.sqrt(0.5), math.sqrt(0.5), 0])
    # Slide beyond the free edge while straightening the wrist. A vertical
    # lift here would fold the upper arm into the opened door.
    retreat = sapien.Pose(current.p + [0, -0.08, 0], upright.q)
    expert.move(arm, retreat, linear=True, touch=[s.lid])
    if not expert.base.measure_facts().dryer_open:
        raise PlanningFailure("dryer_door_closed_during_push_retreat")


def operate_door(expert, opening):
    from .planning import PlanningFailure, sp

    s, arm = expert.base.station.dryer, 1
    expert.operation = "pull_dryer_door" if opening else "close_dryer_door"
    angle = s.lid_angle
    handle = s.handle_pose(angle)
    grasp = handle * sapien.Pose([0, 0, 0.035], q=[0, math.sqrt(0.5), math.sqrt(0.5), 0])
    approach = grasp.to_transformation_matrix()[:3, 2]
    expert.grippers[arm] = 0.15
    expert.hold(0.4)
    expert.move(arm, sapien.Pose(grasp.p - 0.08 * approach, grasp.q), center_wrist=True)
    angle = s.lid_angle
    handle = s.handle_pose(angle)
    grasp = handle * sapien.Pose([0, 0, 0.035], q=[0, math.sqrt(0.5), math.sqrt(0.5), 0])
    expert.move(arm, grasp, linear=True, touch=[s.lid])
    expert.gripper(arm, True)
    if not expert.base.is_held(s.lid):
        raise PlanningFailure("dryer_handle_grasp_failed")
    expert.payloads[arm] = s.lid
    expert.payload_contacts[arm] = [s.machine.get_links()[0]]
    angle = s.lid_angle
    handle = s.handle_pose(angle)
    relative = handle.inv() * sp(expert.agents[arm].tcp.pose)
    for theta in np.linspace(angle, 0.45 if opening else 0.015, 16 if opening else 28)[1:]:
        expert.move(arm, s.handle_pose(float(theta)) * relative, linear=True)
    expert.grippers[arm] = 0.15
    expert.hold(0.5)
    expert.payloads[arm] = None
    expert.payload_contacts[arm] = []
    current = sp(expert.agents[arm].tcp.pose)
    outward = -current.to_transformation_matrix()[:3, 2]
    expert.move(arm, sapien.Pose(current.p + 0.08 * outward, current.q), linear=True, touch=[s.lid])
    expert.hold(0.4)
    if opening:
        push_door_open(expert)
    if opening and not expert.base.measure_facts().dryer_open:
        raise PlanningFailure("dryer_door_not_open")
    if not opening and not expert.base.measure_facts().dryer_closed:
        raise PlanningFailure("dryer_door_not_closed")
    expert.operation = ""


def run_drying(expert, pick):
    from .planning import PlanningFailure, downward, sp
    from .slide_cleaning import BEAKER_GRASP

    s, arm = expert.base.station, 1
    beaker, rack = s.objects["wash_beaker"], s.objects["slide_rack"]
    expert.move(0, downward(s.world([-0.45, -0.25, 1.20])))
    # Move the nested load to the right of the door sweep before opening it.
    # This temporary tabletop placement keeps the left-hinged door out of the
    # right arm's pickup corridor; the rack remains free inside the cup.
    pick(beaker, BEAKER_GRASP, s.furniture[0], contents=[rack])
    staging = s.world([0.63, -0.32, 0.76])
    relative = sp(beaker.pose).inv() * sp(expert.agents[arm].tcp.pose)
    goal = sapien.Pose(staging, [math.sqrt(0.5), 0, 0, -math.sqrt(0.5)]) * relative
    current = sp(expert.agents[arm].tcp.pose)
    height = max(current.p[2], goal.p[2] + 0.20)
    expert.move(arm, sapien.Pose([*current.p[:2], height], current.q), linear=True)
    expert.move(arm, sapien.Pose([*goal.p[:2], height], goal.q), linear=True)
    relative = sp(beaker.pose).inv() * sp(expert.agents[arm].tcp.pose)
    goal = sapien.Pose(staging + [0, 0, 0.001], [math.sqrt(0.5), 0, 0, -math.sqrt(0.5)]) * relative
    expert.payload_contacts[arm] = [s.furniture[0]]
    expert.move(arm, goal, linear=True)
    expert.gripper(arm, False)
    expert.payloads[arm] = None
    expert.carried_contents[arm] = []
    expert.payload_contacts[arm] = []
    expert.move(arm, sapien.Pose(goal.p + [0, 0, 0.15], goal.q), linear=True, touch=[beaker])
    expert.hold(0.4)
    if not expert.base.measure_facts().rack_in_beaker:
        raise PlanningFailure("rack_left_beaker_during_staging")
    operate_door(expert, True)
    # Front grasp keeps the wrist outside the cavity during horizontal loading.
    grip_point = (sp(beaker.pose) * sapien.Pose(BEAKER_GRASP)).p
    grasp = sapien.Pose(grip_point, [0.27059805, -0.65328148, -0.65328148, -0.27059805])
    expert.gripper(arm, False)
    load_tcp = s.dryer.instrument_point(LOAD_LOCAL) + [0, -0.064, 0.145]
    pregrasp = sapien.Pose(grasp.p - 0.08 * grasp.to_transformation_matrix()[:3, 2], grasp.q)
    lifted = sapien.Pose(grasp.p + [0, 0.08, 0.08], grasp.q)
    approach_path = [
        grasp,
        lifted,
        sapien.Pose(load_tcp + [0, -0.30, 0], grasp.q),
        sapien.Pose(load_tcp, grasp.q),
        sapien.Pose(load_tcp + [0, 0, -0.011], grasp.q),
        sapien.Pose(load_tcp + [0, -0.20, -0.011], grasp.q),
        lifted,
        pregrasp,
    ]
    front_path = expert.move(
        arm,
        pregrasp,
        touch=[beaker],
        center_wrist=True,
        cartesian_after=approach_path,
        clearance=[s.dryer.lid],
        preview_carried=[beaker, rack],
    )
    expert.move(arm, grasp, linear=True, touch=[beaker])
    expert.gripper(arm, True)
    if not expert.base.is_held(beaker):
        raise PlanningFailure("front_beaker_grasp_failed")
    expert.payloads[arm] = beaker
    expert.carried_contents[arm] = [rack]
    expert.payload_contacts[arm] = [s.furniture[0]]
    expert.move(arm, sapien.Pose(grasp.p + [0, 0.08, 0.08], grasp.q), linear=True)
    expert.payload_contacts[arm] = []
    target = s.dryer.instrument_point(LOAD_LOCAL)
    # Enter through the front opening horizontally, below the ceiling. The
    # loaded beaker must never be lowered through the cabinet roof.
    relative = sp(beaker.pose).inv() * sp(expert.agents[arm].tcp.pose)
    outside = target + [0, -0.30, 0.012]
    goal = sapien.Pose(outside, [math.sqrt(0.5), 0, 0, -math.sqrt(0.5)]) * relative
    expert.move(arm, goal, linear=True)
    relative = sp(beaker.pose).inv() * sp(expert.agents[arm].tcp.pose)
    inside = sapien.Pose(target + [0, 0, 0.012], [math.sqrt(0.5), 0, 0, -math.sqrt(0.5)]) * relative
    expert.move(arm, inside, linear=True)
    relative = sp(beaker.pose).inv() * sp(expert.agents[arm].tcp.pose)
    goal = sapien.Pose(target + [0, 0, 0.001], [math.sqrt(0.5), 0, 0, -math.sqrt(0.5)]) * relative
    expert.payload_contacts[arm] = [s.dryer.machine.get_links()[0]]
    expert.move(arm, goal, linear=True)
    expert.gripper(arm, False)
    expert.payloads[arm] = None
    expert.carried_contents[arm] = []
    expert.payload_contacts[arm] = []
    expert.move(arm, sapien.Pose(goal.p + [0, -0.20, 0], goal.q), linear=True, touch=[beaker])
    expert.hold(0.5)
    facts = expert.base.measure_facts()
    if not (
        facts.beaker_in_dryer and facts.rack_in_beaker and expert.base.task_state.has_dryer_loaded
    ):
        raise PlanningFailure("loaded_beaker_not_seated_in_dryer")
    # Return along the checked empty-arm approach, avoiding a new wrist winding.
    expert.move(arm, sapien.Pose(grasp.p + [0, 0.08, 0.08], grasp.q), linear=True)
    expert.move(
        arm,
        sapien.Pose(grasp.p - 0.08 * grasp.to_transformation_matrix()[:3, 2], grasp.q),
        linear=True,
    )
    expert.retrace(arm, front_path)
    operate_door(expert, False)
