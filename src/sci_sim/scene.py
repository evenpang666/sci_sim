"""Procedural lab assets. All lengths are metres; Z points up.

Instrument shells, rack slots and beakers have real collision openings.
Only visual PCR wells are decorative; insertion uses the physical plate bed.
"""
from dataclasses import dataclass, field
import math
import numpy as np
import sapien

from .config import (TABLE_SIZE, TABLE_TOP, asset_root,
                     STAND_TUBE_DIAMETER, STAND_CROSSBAR_Z)
from .stand_geometry import crossbar_mesh_file

WHITE = (0.86, 0.89, 0.89, 1)
STEEL = (0.30, 0.36, 0.39, 1)
DARK = (0.055, 0.075, 0.095, 1)
ORANGE = (0.98, 0.39, 0.07, 1)
BLUE = (0.08, 0.46, 0.67, 1)
RED = (0.44, 0.07, 0.10, 1)


def material(color):
    return sapien.render.RenderMaterial(base_color=color, roughness=0.65)


def box(builder, half, p=(0, 0, 0), color=WHITE, collision=True, q=(1, 0, 0, 0), density=500):
    pose = sapien.Pose(p, q)
    builder.add_box_visual(pose=pose, half_size=half, material=material(color))
    if collision:
        builder.add_box_collision(pose=pose, half_size=half, density=density)


def cylinder(builder, radius, half_length, p=(0, 0, 0), color=WHITE, collision=True,
             density=500):
    # SAPIEN cylinders are aligned with local X.
    pose = sapien.Pose(p, [math.sqrt(.5), 0, math.sqrt(.5), 0])
    builder.add_cylinder_visual(pose=pose, radius=radius, half_length=half_length,
                                material=material(color))
    if collision:
        builder.add_cylinder_collision(pose=pose, radius=radius, half_length=half_length,
                                       density=density)


def ring(builder, radius, thickness, height, z, color, segments=32, collision=True):
    for i in range(segments):
        a = 2 * math.pi * i / segments
        box(builder, (thickness / 2, math.pi * radius / segments + .0003, height / 2),
            (radius * math.cos(a), radius * math.sin(a), z), color, collision,
            q=(math.cos(a / 2), 0, 0, math.sin(a / 2)))


def actor(scene, name, draw, pose, dynamic=False):
    builder = scene.create_actor_builder()
    draw(builder)
    builder.initial_pose = pose
    return builder.build(name=name) if dynamic else builder.build_kinematic(name=name)


def build_table_and_stand(scene, origin, prefix):
    def table(b):
        box(b, (TABLE_SIZE[0] / 2, TABLE_SIZE[1] / 2, TABLE_SIZE[2] / 2),
            (0, 0, TABLE_TOP - TABLE_SIZE[2] / 2))
        for x in (-.60, .60):
            for y in (-.30, .30):
                box(b, (.028, .028, .355), (x, y, .355), STEEL)
        box(b, (.61, .025, .045), (0, .30, .57), STEEL)
    def stand(b):
        # Equal-diameter vertical and horizontal cylinders form the user's T-shaped support.
        # Cabinet front y=-.50 stays 5 cm clear of the independent laboratory table.
        box(b, (.48, .35, .025), (0, -.85, .13), DARK)
        box(b, (.45, .32, .26), (0, -.85, .415), WHITE)
        for x in (-.46, .46):
            for y in (-1.18, -.52):
                box(b, (.018, .018, .285), (x, y, .435), STEEL)
                wheel_pose = sapien.Pose([x, y, .075], [math.sqrt(.5), 0, 0, math.sqrt(.5)])
                b.add_cylinder_visual(pose=wheel_pose, radius=.045, half_length=.022, material=material(DARK))
                b.add_cylinder_collision(pose=wheel_pose, radius=.045, half_length=.022)
                box(b, (.025, .025, .03), (x, y, .11), STEEL)
        for x in (-.218, .218):
            box(b, (.212, .009, .245), (x, -1.179, .415), (.77, .79, .73, 1))
            box(b, (.012, .014, .046), (x * .19, -1.195, .415), DARK)
        box(b, (.49, .36, .023), (0, -.85, .735), STEEL)
        for y in np.linspace(-1.18, -.52, 24):
            box(b, (.465, .0018, .001), (0, y, .759), DARK, collision=False)
        cylinder(b, .115, .012, (0, -.74, .772), WHITE)
        radius = STAND_TUBE_DIAMETER / 2
        bottom = .784
        cylinder(b, radius, (STAND_CROSSBAR_Z-bottom)/2,
                 (0, -.74, (STAND_CROSSBAR_Z+bottom)/2), WHITE)
        horizontal = sapien.Pose([0, -.74, STAND_CROSSBAR_Z])
        crossbar = crossbar_mesh_file()
        b.add_visual_from_file(crossbar, pose=horizontal, material=material(WHITE))
        b.add_convex_collision_from_file(crossbar, pose=horizontal)
    return [actor(scene, prefix + "table", table, sapien.Pose(origin)),
            actor(scene, prefix + "stand", stand, sapien.Pose(origin))]


RACK_SLOTS = [(x, y, .030) for y in (-.023, .023) for x in (-.048, 0, .048)]


def draw_rack(b):
    box(b, (.080, .050, .003), (0, 0, .003), ORANGE)
    # Six open square cells, generous clearance for a 12 mm microcentrifuge tube.
    for x in (-.072, -.024, .024, .072):
        box(b, (.004, .050, .011), (x, 0, .017), ORANGE)
    for y in (-.046, 0, .046):
        box(b, (.076, .004, .011), (0, y, .017), ORANGE)
    # Close-fitting sleeves prevent a released microtube tipping in an oversized cell.
    for x, y, _ in RACK_SLOTS:
        for i in range(16):
            a = i*2*math.pi/16
            box(b, (.0015, .0019, .009),
                (x+.009*math.cos(a), y+.009*math.sin(a), .017), ORANGE,
                q=(math.cos(a/2), 0, 0, math.sin(a/2)))


def draw_tube(b, color=BLUE):
    folder = asset_root()/'labware'
    if (folder/'tube_body.obj').is_file():
        b.add_visual_from_file(str(folder/'tube_body.obj'), material=material((.78, .87, .88, 1)))
        b.add_visual_from_file(str(folder/'tube_cap.obj'), material=material(color))
        q = [math.sqrt(.5), 0, math.sqrt(.5), 0]
        for radius, half, z in ((.006, .018, 0), (.004, .003, -.021), (.0075, .003, .021)):
            b.add_cylinder_collision(radius=radius, half_length=half,
                                     pose=sapien.Pose([0, 0, z], q), density=1000)
        return
    cylinder(b, .006, .018, (0, 0, 0), (0.78, .87, .88, 1), density=1000)
    cylinder(b, .004, .003, (0, 0, -.021), WHITE, density=1000)
    cylinder(b, .0075, .003, (0, 0, .021), color, density=1000)


def draw_plate(b):
    mesh = asset_root()/'labware/plate.obj'
    if mesh.is_file():
        b.add_visual_from_file(str(mesh), material=material((.56, .76, .50, 1)))
        b.add_box_collision(half_size=[.057, .038, .005], density=80)
        return
    box(b, (.057, .038, .005), (0, 0, 0), (0.56, .76, .50, 1), density=80)
    for i in range(12):
        for j in range(8):
            cylinder(b, .0028, .001, ((i - 5.5) * .009, (j - 3.5) * .009, .006),
                     (0.18, .34, .24, 1), collision=False)


def draw_beaker(b):
    cylinder(b, .035, .002, (0, 0, .002), (0.65, .84, .88, .6))
    ring(b, .034, .002, .075, .040, (0.65, .84, .88, .6))
    for z in (.02, .04, .06):
        box(b, (.010, .0008, .0005), (0, -.036, z), BLUE, collision=False)


@dataclass
class Station:
    task: str
    origin: np.ndarray
    prefix: str
    fixtures: dict = field(default_factory=dict)
    objects: dict = field(default_factory=dict)
    furniture: list = field(default_factory=list)
    machine: object = None
    lid: object = None
    button: object = None
    layout: dict = field(default_factory=dict)
    lid_depth: float = 0
    lid_height: float = 0
    machine_height: float = 0
    instrument_size: tuple | None = None
    asset_metadata: dict | None = None
    hinge_frame: object = None
    handle_local: object = None
    button_local: object = None
    button_rotation: object = None
    mixer_local: object = None
    target_local: dict = field(default_factory=dict)
    knob: object = None
    knob_frame: object = None
    beaker_water: bool = False
    bath_water: bool = False
    rotor: object = None
    spin_joint: object = None
    spin_joint_index: int = 0
    drop_visual: object = None
    coating_visual: object = None
    dose_visual: object = None
    puddle_visual: object = None
    dryer: object = None
    flash: object = None
    status_light: object = None

    def world(self, p):
        return self.origin + np.asarray(p)

    def instrument_point(self, local):
        return self.world(np.asarray(self.layout["instrument"]) + local)

    @property
    def button_point(self):
        if self.button_local is not None:
            return self.instrument_point(self.button_local)
        return self.instrument_point([0, -self.lid_depth / 2 + .027, .125])

    @property
    def button_normal(self):
        return np.array([0, 0, 1]) if self.button_rotation is None else self.button_rotation[:, 2]

    def handle_pose(self, angle):
        if self.hinge_frame is not None:
            rotation = sapien.Pose(q=[math.cos(angle/2), math.sin(angle/2), 0, 0])
            return (sapien.Pose(self.instrument_point([0, 0, 0])) * self.hinge_frame *
                    rotation * self.hinge_frame.inv() * self.handle_local)
        hinge = sapien.Pose([0, self.lid_depth / 2 - .012, self.lid_height])
        rotation = sapien.Pose(q=[math.cos(angle / 2), math.sin(angle / 2), 0, 0])
        return (sapien.Pose(self.instrument_point([0, 0, 0])) * hinge * rotation *
                sapien.Pose([0, -self.lid_depth + .044, .025]))

    @property
    def lid_angle(self):
        return float(self.machine.get_qpos()[0, 0].item())

    @property
    def knob_angle(self):
        return 0. if self.knob is None else float(self.machine.get_qpos()[0, 1].item())

    def knob_pose(self):
        return self.lid.pose.sp * self.hinge_frame.inv() * self.knob_frame

    def set_layout(self, layout):
        self.layout = layout
        if self.task == 'spin_coating':
            from .spin_coating import reset_station
            reset_station(self, layout)
            return
        if self.task == 'slide_cleaning':
            from .slide_cleaning import reset_station
            reset_station(self, layout)
            return
        for name, obj in self.fixtures.items():
            obj.set_pose(sapien.Pose(self.world(layout[name])))
        if self.task == "mix_transfer":
            self.machine.set_pose(sapien.Pose(self.world(layout["instrument"])))
        else:
            self.machine.set_root_pose(sapien.Pose(self.world(layout["instrument"])))
            zeros = np.zeros((1, len(self.machine.get_active_joints())), dtype=np.float32)
            self.machine.set_qpos(zeros)
            self.machine.set_qvel(zeros)
            matrix = np.eye(4)
            if self.button_rotation is not None:
                matrix[:3, :3] = self.button_rotation
            matrix[:3, 3] = self.button_point
            self.button.set_pose(sapien.Pose(matrix))
        for i, (name, obj) in enumerate(self.objects.items()):
            if name == "plate":
                point = np.asarray(layout["source_rack"]) + [0, 0, .045]
            else:
                point = np.asarray(layout["source_rack"]) + RACK_SLOTS[i]
            obj.set_pose(sapien.Pose(self.world(point)))
            obj.set_linear_velocity(np.zeros((1, 3)))
            obj.set_angular_velocity(np.zeros((1, 3)))


def build_station(scene, task, origin, prefix, instruments='atombridge') -> Station:
    s = Station(task, np.asarray(origin), prefix)
    s.furniture = build_table_and_stand(scene, origin, prefix)
    initial = sapien.Pose(s.world([0, 0, TABLE_TOP]))
    if task == 'spin_coating':
        from .spin_coating import build_spin_coating
        return build_spin_coating(scene, s, initial)
    if task == 'slide_cleaning':
        from .slide_cleaning import build_slide_cleaning
        return build_slide_cleaning(scene, s, initial)
    def plate_nest(b):
        box(b, (.040, .022, .020), (0, 0, .020), ORANGE)
        for x in (-.060, .060):
            box(b, (.0015, .030, .006), (x, 0, .046), ORANGE)
    for name in ("source_rack", "target_rack"):
        draw = (plate_nest
                if task == 'thermal_cycler' and name == 'source_rack' else draw_rack)
        s.fixtures[name] = actor(scene, prefix + name, draw, initial)
    s.fixtures["beaker"] = actor(scene, prefix + "beaker", draw_beaker, initial)
    names = ("plate",) if task == "thermal_cycler" else (
        ("sample", "balance") if task == "centrifuge" else ("sample",))
    for name in names:
        draw = draw_plate if name == "plate" else (
            lambda b, c=ORANGE if name == "balance" else BLUE: draw_tube(b, c))
        s.objects[name] = actor(scene, prefix + name, draw, initial, dynamic=True)
    if instruments == 'atombridge':
        from .instruments import build_instrument
        return build_instrument(scene, s, initial)
    if task == "mix_transfer":
        def vortex(b):
            box(b, (.061, .0825, .060), (0, 0, .060), BLUE)
            cylinder(b, .022, .021, (0, 0, .141), DARK)
            cylinder(b, .025, .0015, (0, 0, .1635), DARK)
            box(b, (.022, .002, .009), (0, -.083, .055), DARK, collision=False)
        s.machine = actor(scene, prefix + "vortex_genie_2", vortex, initial)
        return s

    thermal = task == "thermal_cycler"
    w, d, h = (.26, .47, .23) if thermal else (.24, .39, .24)
    s.lid_depth, s.lid_height, s.machine_height = d, h - .015, h
    builder = scene.create_articulation_builder()
    root = builder.create_link_builder()
    root.set_name(prefix + "instrument_body")
    color = RED if thermal else WHITE
    # Low floor plus side walls; never a solid cube blocking the work cavity.
    box(root, (w / 2, d / 2, .055), (0, 0, .055), color)
    for x in (-w / 2 + .012, w / 2 - .012):
        box(root, (.012, d / 2, .045), (x, 0, .155), color)
    for y in (-d / 2 + .018, d / 2 - .012):
        box(root, (w / 2, .012, .045), (0, y, .155), color)
    box(root, (.043, .023, .001), (0, -d / 2 + .04, .111), BLUE, collision=False)
    if thermal:
        box(root, (.076, .050, .010), (0, 0, .160), STEEL)
        box(root, (.003, .051, .007), (.079, 0, .177), DARK)
        # Leave a notch for the plate's added grasp tab at the left edge.
        for y in (-.037, .037):
            box(root, (.003, .014, .007), (-.089, y, .177), DARK)
        for y in (-.054, .054):
            box(root, (.079, .003, .007), (0, y, .177), DARK)
    else:
        # Simplified vertical holders, 24 equally spaced around a rotor ring.
        cylinder(root, .087, .004, (0, 0, .136), STEEL)
        cylinder(root, .045, .019, (0, 0, .159), DARK)
        for i in range(24):
            a = 2 * math.pi * i / 24
            # Radial separators leave physical tube insertion openings.
            box(root, (.013, .0013, .014), (.066 * math.cos(a), .066 * math.sin(a), .154),
                STEEL, q=(math.cos(a / 2), 0, 0, math.sin(a / 2)))
        ring(root, .086, .003, .035, .1575, STEEL)
    lid = builder.create_link_builder(root)
    lid.set_name(prefix + "lid")
    lid.set_joint_name(prefix + "lid_hinge")
    lid.set_joint_properties("revolute", limits=[[-1.70, 0]],
                            pose_in_parent=sapien.Pose([0, d / 2 - .012, h - .015]),
                            pose_in_child=sapien.Pose(), friction=.02, damping=.15)
    box(lid, (w / 2, d / 2 - .012, .012), (0, -d / 2 + .012, 0), color, density=120)
    box(lid, (.028, .010, .010), (0, -d + .044, .025), DARK, density=120)
    builder.initial_pose = initial
    s.machine = builder.build(name=prefix + ("t100" if thermal else "5425"), fix_root_link=True)
    s.lid = s.machine.links_map[prefix + "lid"]
    s.button = actor(scene, prefix + "start_button",
                     lambda b: box(b, (.013, .010, .004), color=(.12, .68, .38, 1)), initial)
    return s
