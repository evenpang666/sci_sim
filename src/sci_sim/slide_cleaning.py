"""Open ultrasonic bath and nested contact-driven glassware manipulation.

Dimensions are task-scale approximations of the supplied GIF, not vendor CAD.
Water surfaces are visual only; mass includes a nominal water load. Slides are
fixed in the rack. No grasp constraints or object-pose setters are used by the expert.
"""

import math

import numpy as np
import sapien

from .scene import BLUE, DARK, WHITE, actor, box, cylinder, ring

BEAKER_RADIUS = 0.045
BEAKER_FLOOR = 0.004
BEAKER_HEIGHT = 0.115
RACK_RADIUS = 0.030
BATH_FLOOR = 0.040
RACK_GRASP = np.array([0, 0.025, 0.145])
BEAKER_GRASP = np.array([0.064, 0, 0.133])
GLASS = (0.72, 0.91, 0.96, 0.20)
WATER = (0.18, 0.62, 0.85, 0.16)


def draw_slide_rack(b):
    # Perforated white carrier, five 25 x 75 mm slides and a raised lifting stem.
    cylinder(b, RACK_RADIUS, 0.002, (0, 0, 0.002), WHITE, density=300)
    for y in (-0.019, 0.019):
        box(b, (0.025, 0.003, 0.009), (0, y, 0.013), WHITE, density=300)
    for x in np.linspace(-0.020, 0.020, 5):
        box(b, (0.0006, 0.0125, 0.0375), (x, 0, 0.0495), (0.64, 0.84, 0.91, 0.55), density=2300)
        box(b, (0.001, 0.0125, 0.0025), (x, 0, 0.0845), (0.76, 0.92, 0.95, 1), collision=False)
    box(b, (0.005, 0.005, 0.070), (0, 0.025, 0.074), WHITE, density=250)
    box(b, (0.010, 0.009, 0.012), RACK_GRASP, WHITE, density=250)


def draw_cleaning_beaker(b):
    cylinder(b, BEAKER_RADIUS, 0.002, (0, 0, 0.002), GLASS, density=2500)
    ring(
        b,
        BEAKER_RADIUS - 0.001,
        0.002,
        BEAKER_HEIGHT - 0.004,
        (BEAKER_HEIGHT + 0.004) / 2,
        GLASS,
        segments=24,
    )
    ring(
        b,
        BEAKER_RADIUS - 0.001,
        0.0025,
        0.003,
        BEAKER_HEIGHT,
        (0.65, 0.85, 0.89, 0.6),
        segments=24,
        collision=False,
    )
    # A small exterior handling lug keeps 2F-85 fingers away from the loaded rack.
    box(b, (0.011, 0.009, 0.012), BEAKER_GRASP, BLUE, density=450)
    box(b, (0.015, 0.008, 0.004), (0.051, 0, 0.116), BLUE, density=450)
    box(b, (0.004, 0.008, 0.008), (0.064, 0, 0.123), BLUE, density=450)
    for z in (0.025, 0.045, 0.065, 0.085):
        box(b, (0.012, 0.0005, 0.0006), (0, -0.0455, z), WHITE, collision=False)
    # Non-colliding liquid surface; added base mass approximates the water load.
    cylinder(b, 0.042, 0.0006, (0, 0, 0.098), WATER, collision=False)
    b.add_cylinder_collision(
        radius=0.040,
        half_length=0.001,
        pose=sapien.Pose([0, 0, 0.002], [math.sqrt(0.5), 0, math.sqrt(0.5), 0]),
        density=18000,
    )


def build_slide_cleaning(scene, s, initial):
    s.instrument_size = (0.28, 0.28, 0.18)
    s.beaker_water = s.bath_water = True
    s.asset_metadata = {
        "source": "Procedural reconstruction from user slide-cleaning GIF",
        "water": "Pre-filled water in both beaker and ultrasonic bath; visual only",
        "physics": "Open compound collisions, free dynamic beaker and slide rack; contact start and timed cleaning",
        "slides": "Five 25 x 75 mm glass slides fixed within the carrier",
        "handling": "Raised carrier stem and exterior beaker lug for Robotiq 2F-85",
        "size_xyz_m": list(s.instrument_size),
    }

    def cleaner(b):
        steel = (0.63, 0.68, 0.71, 1)
        box(b, (0.14, 0.14, 0.020), (0, 0, 0.020), steel)
        for x in (-0.13, 0.13):
            box(b, (0.01, 0.14, 0.065), (x, 0, 0.105), steel)
        for y in (-0.13, 0.13):
            box(b, (0.12, 0.01, 0.065), (0, y, 0.105), steel)
        # Recessed basin with an unobstructed 240 x 240 mm opening.
        for x in np.linspace(-0.11, 0.11, 9):
            box(
                b,
                (0.001, 0.11, 0.0006),
                (x, 0, BATH_FLOOR + 0.0006),
                (0.78, 0.81, 0.83, 1),
                collision=False,
            )
        for y in np.linspace(-0.11, 0.11, 9):
            box(
                b,
                (0.11, 0.001, 0.0006),
                (0, y, BATH_FLOOR + 0.0006),
                (0.78, 0.81, 0.83, 1),
                collision=False,
            )
        box(b, (0.119, 0.119, 0.0005), (0, 0, 0.140), WATER, collision=False)
        box(b, (0.08, 0.001, 0.035), (0, -0.1405, 0.080), BLUE, collision=False)
        box(b, (0.035, 0.001, 0.018), (-0.025, -0.142, 0.083), DARK, collision=False)
        for x in (-0.043, -0.025, -0.007):
            box(
                b, (0.004, 0.001, 0.010), (x, -0.143, 0.083), (0.98, 0.65, 0.18, 1), collision=False
            )
        for x in (-0.12, 0.12):
            for y in (-0.11, 0.11):
                box(b, (0.012, 0.014, 0.006), (x, y, 0.006), DARK)

    s.machine = actor(scene, s.prefix + "ultrasonic_cleaner", cleaner, initial)
    s.button_local = np.array([0.090, -0.13, 0.174])
    s.button = actor(
        scene,
        s.prefix + "cleaning_start_button",
        lambda b: box(b, (0.014, 0.012, 0.004), color=(0.12, 0.74, 0.35, 1)),
        initial,
    )
    # Thin markings identify the initial rack position and clean rack landing zone.
    for name, color in [
        ("source_rack", (0.7, 0.75, 0.77, 1)),
        ("target_rack", (0.17, 0.62, 0.50, 1)),
    ]:
        s.fixtures[name] = actor(
            scene,
            s.prefix + name,
            lambda b, c=color: box(b, (0.045, 0.045, 0.0005), (0, 0, 0.0005), c, collision=False),
            initial,
        )
    s.objects["slide_rack"] = actor(
        scene, s.prefix + "slide_rack", draw_slide_rack, initial, dynamic=True
    )
    s.objects["wash_beaker"] = actor(
        scene, s.prefix + "wash_beaker", draw_cleaning_beaker, initial, dynamic=True
    )
    from .slide_drying import build_dryer

    s.dryer = build_dryer(scene, s, initial)
    s.asset_metadata["dryer"] = s.dryer.asset_metadata
    return s


def reset_station(s, layout):
    from .slide_drying import reset_dryer

    reset_dryer(s)
    for name, obj in s.fixtures.items():
        obj.set_pose(sapien.Pose(s.world(layout[name])))
    s.machine.set_pose(sapien.Pose(s.world(layout["instrument"])))
    s.button.set_pose(sapien.Pose(s.button_point))
    for name, key in [("slide_rack", "source_rack"), ("wash_beaker", "beaker")]:
        obj = s.objects[name]
        obj.set_pose(sapien.Pose(s.world(np.array(layout[key]) + [0, 0, 0.0005])))
        obj.set_linear_velocity(np.zeros((1, 3)))
        obj.set_angular_velocity(np.zeros((1, 3)))


def measure_cleaning_facts(base, facts):
    s = base.station
    rack, beaker = s.objects["slide_rack"], s.objects["wash_beaker"]
    # Compare in the moving cup's frame, including tilt and actual support contact.
    relative = beaker.pose.sp.inv() * rack.pose.sp
    rotation = relative.to_transformation_matrix()[:3, :3]
    # The carrier is free to slide to the inner wall (13 mm radial clearance).
    radial_clearance = BEAKER_RADIUS - 0.002 - RACK_RADIUS
    facts.rack_in_beaker = bool(
        np.linalg.norm(relative.p[:2]) < radial_clearance + 0.001
        and abs(relative.p[2] - BEAKER_FLOOR) < 0.006
        and rotation[2, 2] > 0.97
        and base.contact(rack, beaker)
    )
    facts.beaker_in_cleaner = bool(
        base.close_to(beaker, s.instrument_point([0, 0, BATH_FLOOR]), xy=0.015, z=0.008)
        and base.contact(beaker, s.machine)
    )
    facts.beaker_on_table = bool(
        base.close_to(
            beaker,
            s.world(s.layout["target_rack" if base.task_state.has_cleaned else "beaker"]),
            xy=0.015,
            z=0.006,
        )
        and base.contact(beaker, s.furniture[0])
    )
    facts.rack_on_table = bool(
        base.close_to(rack, s.world(s.layout["target_rack"]), xy=0.015, z=0.006)
        and base.contact(rack, s.furniture[0])
    )
    from .slide_drying import measure_drying_facts

    measure_drying_facts(base, facts)
    facts.water_ready = s.beaker_water and s.bath_water
    facts.button_contact = any(
        base.contact(link, s.button)
        for a in base.active_agents
        for link in (a.finger1_link, a.finger2_link)
    )


def run_cleaning(expert):
    from .env import xyz
    from .planning import PlanningFailure, downward, sp

    s = expert.base.station
    rack, beaker = s.objects["slide_rack"], s.objects["wash_beaker"]
    arm = 0

    def pick(obj, offset, support, contents=()):
        target = sp(obj.pose) * downward(offset)
        expert.gripper(arm, False)
        expert.move(arm, sapien.Pose(target.p + [0, 0, 0.12], target.q))
        expert.move(arm, target, linear=True, touch=[obj])
        expert.gripper(arm, True)
        if not bool(expert.agents[arm].is_grasping(obj, min_force=0.02).item()):
            raise PlanningFailure(obj.name + ": grasp_not_established")
        expert.payloads[arm] = obj
        expert.carried_contents[arm] = list(contents)
        expert.payload_contacts[arm] = [support]
        expert.move(arm, sapien.Pose(target.p + [0, 0, 0.15], target.q), linear=True)
        expert.payload_contacts[arm] = []

    def place(obj, target, support, contents=()):
        relative = sp(obj.pose).inv() * sp(expert.agents[arm].tcp.pose)
        goal = sapien.Pose(target) * relative
        if contents:
            # Keep the free rack upright and clear the cleaner rim in transit.
            current = sp(expert.agents[arm].tcp.pose)
            height = max(current.p[2], goal.p[2] + 0.24)
            expert.move(arm, sapien.Pose([*current.p[:2], height], current.q), linear=True)
            expert.move(arm, sapien.Pose([*goal.p[:2], height], goal.q), linear=True)
        else:
            expert.move(arm, sapien.Pose(goal.p + [0, 0, 0.15], goal.q))
        # The side lug can settle in the compliant fingers while transporting a
        # loaded cup. Measure its current offset again before vertical insertion.
        relative = sp(obj.pose).inv() * sp(expert.agents[arm].tcp.pose)
        goal = sapien.Pose(target) * relative
        expert.move(arm, sapien.Pose(goal.p + [0, 0, 0.08], goal.q), linear=True)
        relative = sp(obj.pose).inv() * sp(expert.agents[arm].tcp.pose)
        goal = sapien.Pose(target) * relative
        expert.payload_contacts[arm] = [support]
        expert.move(arm, sapien.Pose(goal.p + [0, 0, 0.001], goal.q), linear=True)
        expert.gripper(arm, False)
        expert.payloads[arm] = None
        expert.carried_contents[arm] = []
        expert.payload_contacts[arm] = []
        expert.move(arm, sapien.Pose(goal.p + [0, 0, 0.15], goal.q), linear=True, touch=[obj])
        expert.hold(0.5)
        if not expert.base.close_to(obj, target, xy=0.015, z=0.008):
            raise PlanningFailure(obj.name + ": placement_outside_tolerance")
        if contents and not expert.base.measure_facts().rack_in_beaker:
            raise PlanningFailure("slide_rack_displaced_during_beaker_transport")

    expert.move(1, downward(s.world([0.45, -0.25, 1.20])))
    pick(rack, RACK_GRASP, s.furniture[0])
    place(rack, xyz(beaker) + [0, 0, BEAKER_FLOOR], beaker)
    if not expert.base.task_state.has_rack_loaded:
        raise PlanningFailure("rack_not_seated_in_beaker")
    pick(beaker, BEAKER_GRASP, s.furniture[0], contents=[rack])
    place(beaker, s.instrument_point([0, 0, BATH_FLOOR]), s.machine, contents=[rack])
    expert.move(0, downward(s.world([-0.45, -0.25, 1.20])))
    expert.press_start()
    if not expert.base.task_state.has_cleaned:
        raise PlanningFailure("ultrasonic_cycle_not_completed")
    expert.move(1, downward(s.world([0.45, -0.25, 1.20])))
    pick(beaker, BEAKER_GRASP, s.machine, contents=[rack])
    place(beaker, s.world(s.layout["target_rack"]), s.furniture[0], contents=[rack])
    if not expert.base.task_state.has_beaker_returned:
        raise PlanningFailure("beaker_not_returned_with_rack")
    from .slide_drying import run_drying

    arm = 1
    run_drying(expert, pick)
