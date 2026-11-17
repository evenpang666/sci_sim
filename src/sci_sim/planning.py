"""Sequential dual-arm MPlib expert with contact-based execution.

Each arm plans while the other holds its measured joint targets. The other arm,
all fixtures, the floor and instrument links are exact FCL collision objects.
Only named task contacts are allowed. Object poses are never teleported during
execution: attachment is a planning collision model, not a physics constraint.
"""
import math
import signal
from contextlib import contextmanager
from copy import deepcopy
from pathlib import Path
import numpy as np
import sapien
import mplib
from scipy.spatial.transform import Rotation, Slerp
from .collision import collision_object

from .env import array, xyz


class PlanningFailure(RuntimeError):
    pass


class EpisodeComplete(Exception):
    """Stop at the first successful terminal transition (no post-done frames)."""


@contextmanager
def planning_deadline(seconds=30):
    """Bound iterative IK and path planning in the Linux collector process."""
    def expired(signum, frame):
        raise PlanningFailure(f"planning_timeout_{seconds}s")
    previous = signal.signal(signal.SIGALRM, expired)
    signal.setitimer(signal.ITIMER_REAL, seconds)
    try:
        yield
    finally:
        signal.setitimer(signal.ITIMER_REAL, 0)
        signal.signal(signal.SIGALRM, previous)


def sp(pose):
    return sapien.Pose(array(pose.p)[0], array(pose.q)[0])


def mp(pose):
    return mplib.Pose(p=pose.p, q=pose.q)


def downward(point):
    # TCP +Z is the approach direction; fingers close along local Y.
    return sapien.Pose(point, [0, 1, 0, 0])


def cartesian_path(planner, goal, qpos, dt):
    """Masked Cartesian IK with dense collision checks and TOPP time scaling."""
    model = planner.pinocchio_model
    link = planner.link_name_2_idx[planner.move_group]
    group = planner.move_group_joint_indices
    mask = np.ones(len(qpos), dtype=bool)
    mask[group] = False
    model.compute_forward_kinematics(qpos)
    start = model.get_link_pose(link).to_transformation_matrix()
    end = goal.to_transformation_matrix()
    rotation = Rotation.from_matrix(np.stack([start[:3, :3], end[:3, :3]]))
    angle = (rotation[1]*rotation[0].inv()).magnitude()
    distance = np.linalg.norm(end[:3, 3]-start[:3, 3])
    count = max(2, math.ceil(distance/.004), math.ceil(angle/.03))
    slerp = Slerp([0, 1], rotation)
    path = [qpos[group].copy()]
    previous = qpos.copy()
    for alpha in np.linspace(0, 1, count+1)[1:]:
        target = np.eye(4)
        target[:3, :3] = slerp(alpha).as_matrix()
        target[:3, 3] = (1-alpha)*start[:3, 3]+alpha*end[:3, 3]
        solution, success, _ = model.compute_IK_CLIK(link, mplib.Pose(target), previous, mask)
        if not success or not planner.wrap_joint_limit(solution):
            return {'status': 'Cartesian IK failed'}
        for j in np.flatnonzero(planner.equiv_joint_mask):
            nearest = solution[j]+2*np.pi*round((previous[j]-solution[j])/(2*np.pi))
            if planner.joint_limits[j, 0] <= nearest <= planner.joint_limits[j, 1]:
                solution[j] = nearest
        if np.max(np.abs(solution[group]-previous[group])) > .5:
            return {'status': 'Cartesian IK discontinuity'}
        checks = max(1, math.ceil(np.max(np.abs(solution[group]-previous[group]))/.015))
        for beta in np.linspace(0, 1, checks+1)[1:]:
            point = (1-beta)*previous+beta*solution
            planner.planning_world.set_qpos_all(point[group])
            if planner.planning_world.is_state_colliding():
                return {'status': 'Cartesian collision'}
            path.append(point[group].copy())
        previous = solution
    times, positions, velocity, acceleration, duration = planner.TOPP(np.array(path), dt)
    return {'status': 'Success', 'time': times, 'position': positions}


class DualArmExpert:
    OPEN = 0.0
    CLOSED = 0.80

    def __init__(self, env, observation, recorder=None, render=False, seed=0):
        self.env = env
        self.base = env.unwrapped
        self.obs = observation
        self.record = recorder
        self.render = render
        self.agents = self.base.active_agents
        self.keys = list(self.base.agent.agents_dict)
        self.indices = []
        self.targets = []
        self.grippers = [self.OPEN, self.OPEN]
        self.payloads = [None, None]
        self.carried_contents = [[], []]
        self.payload_contacts = [[], []]
        self.steps = 0
        mplib.set_global_seed(seed)
        for agent in self.agents:
            names = [j.name for j in agent.robot.get_active_joints()]
            ids = [names.index(n) for n in agent.arm_joint_names]
            self.indices.append(ids)
            self.targets.append(array(agent.robot.get_qpos())[0, ids].copy())

    def action(self):
        result = {}
        for i, (key, agent) in enumerate(self.base.agent.agents_dict.items()):
            active = i - self.base.active_index * 2
            if active in (0, 1):
                result[key] = np.r_[self.targets[active], self.grippers[active]].astype(np.float32)
            else:
                names = [j.name for j in agent.robot.get_active_joints()]
                ids = [names.index(n) for n in agent.arm_joint_names]
                result[key] = np.r_[array(agent.robot.get_qpos())[0, ids], 0].astype(np.float32)
        return result

    def step(self):
        if self.steps >= self.base.lab_config.max_steps:
            raise PlanningFailure("episode_horizon")
        action = self.action()
        before = deepcopy(self.obs) if self.record else self.obs
        self.obs, reward, terminated, truncated, info = self.env.step(action)
        self.steps += 1
        if self.record:
            self.record(before, action, reward, terminated, truncated, info)
        if self.render:
            self.base.render_human()
        if self.base.task_state.success:
            raise EpisodeComplete()
        if self.base.task_state.failed:
            raise PlanningFailure(self.base.task_state.failure_reason)
        if bool(array(truncated).any()) and not self.base.task_state.success:
            raise PlanningFailure("time_limit")
        if bool(array(terminated).any()) and not self.base.task_state.success:
            raise PlanningFailure("terminated_without_success")

    def hold(self, seconds):
        for _ in range(math.ceil(seconds * self.base.control_freq)):
            if self.base.task_state.success:
                return
            self.step()

    def gripper(self, arm, closed):
        self.grippers[arm] = self.CLOSED if closed else self.OPEN
        self.hold(.65)

    def planner(self, arm, touch=(), clearance=()):
        agent = self.agents[arm]
        planner = mplib.Planner(
            urdf=agent.urdf_path, srdf=str(Path(agent.urdf_path).with_suffix(".srdf")),
            move_group=agent.ee_link_name,
            user_link_names=[link.name for link in agent.robot.get_links()],
            user_joint_names=[joint.name for joint in agent.robot.get_active_joints()],
            joint_vel_limits=np.full(len(agent.arm_joint_names), .45),
            joint_acc_limits=np.full(len(agent.arm_joint_names), .8))
        # MPlib 0.2.1 attached-body poses omit the articulation's world base offset.
        # Express the entire planning world in this arm's base frame consistently.
        world_to_base = sp(agent.robot.pose).inv()
        payload = self.payloads[arm]
        wrappers = list(self.base.scene.actors.values())
        for articulation in self.base.scene.articulations.values():
            if articulation is not agent.robot:
                wrappers.extend(articulation.get_links())
        object_names = {}
        for obstacle_index, wrapper in enumerate(wrappers):
            if wrapper is payload or any(wrapper is obj for obj in self.carried_contents[arm]):
                continue
            if payload is self.base.station.lid and wrapper is self.base.station.knob:
                continue  # Child knob travels with the lid; included in its payload bound below.
            # Single CPU scene: _bodies[0] is the underlying SAPIEN Physx component.
            fcl = collision_object(wrapper._bodies[0], f"obstacle_{obstacle_index}_{wrapper.name}",
                                   box_margin=.005 if any(wrapper is obj for obj in clearance) else 0.)
            if fcl is None:
                continue
            fcl = mplib.collision_detection.fcl.FCLObject(
                fcl.name, mp(world_to_base) * fcl.pose, fcl.shapes, fcl.shape_poses)
            planner.planning_world.add_object(fcl)
            object_names[id(wrapper)] = fcl.name
            if any(wrapper is obj for obj in touch):
                for finger in ("left_inner_finger_pad", "right_inner_finger_pad",
                               "left_inner_finger", "right_inner_finger"):
                    planner.acm.set_entry(finger, fcl.name, True)
            if wrapper is self.base.station.furniture[1]:
                for link in ("base_link", "base_link_inertia", "base", "link_base", "panda_link0"):
                    if link in planner.user_link_names:
                        planner.acm.set_entry(link, fcl.name, True)
        if payload is not None:
            parts = [payload, *self.carried_contents[arm]]
            if payload is self.base.station.lid and self.base.station.knob is not None:
                parts.append(self.base.station.knob)
            names = []
            tcp_inverse = mp(sp(agent.tcp.pose).inv())
            for part in parts:
                geometry = collision_object(part._bodies[0], 'payload')
                if geometry is None:
                    raise PlanningFailure('payload_has_no_collision_geometry')
                for shape, local in zip(geometry.shapes, geometry.shape_poses):
                    name = f'attached_{len(names)}'
                    relative = tcp_inverse * geometry.pose * local
                    planner.update_attached_object(shape.get_collision_geometry(), relative, name=name)
                    for other in names:
                        planner.acm.set_entry(name, other, True)  # Same compound rigid payload.
                    names.append(name)
                    for link in planner.user_link_names:
                        if link.startswith(('left_', 'right_', 'robotiq_')):
                            planner.acm.set_entry(name, link, True)
                    for obj in self.payload_contacts[arm]:
                        if id(obj) in object_names:
                            planner.acm.set_entry(name, object_names[id(obj)], True)
        return planner

    def move(self, arm, pose, linear=False, touch=(), center_wrist=False, cartesian_after=(), clearance=(), preview_carried=()):
        print(f'move step={self.steps} arm={arm} linear={linear} xyz={pose.p.tolist()}', flush=True)
        planner = self.planner(arm, touch, clearance)
        planning_pose = sp(self.agents[arm].robot.pose).inv() * pose
        qpos = array(self.agents[arm].robot.get_qpos())[0]
        # PhysX linkage constraints can leave passive fingers microradians beyond
        # URDF limits; plan from the closest legal state, keeping arm joints intact.
        qpos = np.clip(qpos, planner.joint_limits[:, 0], planner.joint_limits[:, 1])
        planner.robot.set_qpos(qpos, True)
        initial_collisions = planner.planning_world.check_collision()
        if initial_collisions:
            print('initial collisions', [(c.link_name1, c.link_name2) for c in initial_collisions], flush=True)
        # Finger joints are not part of the arm move group. Keep them fixed in IK.
        with planning_deadline(60 if cartesian_after else 30):
            if linear:
                result = cartesian_path(planner, mp(planning_pose), qpos, self.base.control_timestep)
            else:
                mask = np.ones(len(qpos), dtype=bool)
                mask[self.indices[arm]] = False
                status, goals = planner.IK(mp(planning_pose), qpos, mask=mask,
                                           return_closest=False, n_init_qpos=40)
                if status == 'Success':
                    goals = np.asarray(goals)
                    for j in np.flatnonzero(planner.equiv_joint_mask):
                        candidate = goals[:, j]+2*np.pi*np.round((qpos[j]-goals[:, j])/(2*np.pi))
                        legal = (candidate >= planner.joint_limits[j, 0]) & (candidate <= planner.joint_limits[j, 1])
                        goals[:, j] = np.where(legal, candidate, goals[:, j])
                    if center_wrist and 'wrist_3_joint' in planner.user_joint_names:
                        j = planner.user_joint_names.index('wrist_3_joint')
                        goals[:, j] = (goals[:, j]+np.pi) % (2*np.pi)-np.pi
                    # Keep the outward-facing UR shoulder/elbow branch through
                    # lid arcs; inverted branches fold back through the mast.
                    if 'elbow_joint' in planner.user_joint_names:
                        elbow = planner.user_joint_names.index('elbow_joint')
                        pan = planner.user_joint_names.index('shoulder_pan_joint')
                        lift = planner.user_joint_names.index('shoulder_lift_joint')
                        positive = goals[(goals[:, elbow] > .02) &
                                         (goals[:, pan] > -.5) & (goals[:, pan] < 3.) &
                                         (goals[:, lift] > -2.8) & (goals[:, lift] < 1.5)]
                        if len(positive):
                            goals = positive
                        else:
                            status = 'IK has no stand-clear UR branch'
                    goals = sorted(goals, key=lambda q: np.linalg.norm(q-qpos))
                    if cartesian_after:
                        # Preview robot reach after pickup without leaving the
                        # picked objects frozen at their initial table poses.
                        # Actual loaded moves still attach their full geometry.
                        carried_names = [name for name in planner.planning_world.get_object_names()
                            if any(name.endswith('_'+obj.name) for obj in preview_carried)]
                        carried_objects = [planner.planning_world.get_object(name) for name in carried_names]
                        def restore_preview_objects():
                            for obj in carried_objects:
                                if not planner.planning_world.has_object(obj.name):
                                    planner.planning_world.add_object(obj)
                        reachable = []
                        rejected = {}
                        for candidate in goals:
                            restore_preview_objects()
                            trial = candidate.copy()
                            for waypoint_index, waypoint in enumerate(cartesian_after):
                                if waypoint_index == 1:
                                    for name in carried_names:
                                        planner.planning_world.remove_object(name)
                                preview = cartesian_path(planner,
                                    mp(sp(self.agents[arm].robot.pose).inv()*waypoint),
                                    trial, self.base.control_timestep)
                                if preview['status'] != 'Success':
                                    key = f"{waypoint_index}:{preview['status']}"
                                    rejected[key] = rejected.get(key, 0)+1
                                    break
                                trial[planner.move_group_joint_indices] = preview['position'][-1]
                            else:
                                reachable.append(candidate)
                                if len(reachable) == 3:
                                    break
                        restore_preview_objects()
                        goals = reachable
                        planner.planning_world.set_qpos_all(qpos[planner.move_group_joint_indices])
                        if not goals:
                            status = f'IK has no collision-free continuation: {rejected}'
                    goals = goals[:3]
                result = (planner.plan_qpos(goals, qpos,
                          time_step=self.base.control_timestep, planning_time=5.0)
                          if status == 'Success' else {'status': status})
                if result['status'] != 'Success':
                    # A closest IK branch can be disconnected by the stand.
                    # Allow RRT to connect another collision-free IK solution.
                    if status == 'Success':
                        result = planner.plan_qpos(goals, qpos, time_step=self.base.control_timestep,
                                                   planning_time=10.)
        if result["status"] != "Success":
            collisions = planner.planning_world.check_collision()
            details = [(c.link_name1, c.link_name2) for c in collisions]
            raise PlanningFailure(f"arm{arm}: {result['status']}; collisions={details}; qpos={qpos.tolist()}")
        positions = np.asarray(result["position"])
        times = np.asarray(result["time"])
        if len(positions) < 2:
            raise PlanningFailure("planner_returned_empty_or_single_sample_path")
        # TOPPRA times need not be exact control ticks. Resample onto simulation time.
        ticks = np.arange(0, float(times[-1]) + self.base.control_timestep,
                          self.base.control_timestep)
        path = np.stack([np.interp(ticks, times, positions[:, j])
                         for j in range(positions.shape[1])], axis=1)
        group_names = [planner.user_joint_names[i] for i in planner.move_group_joint_indices]
        reorder = [group_names.index(n) for n in self.agents[arm].arm_joint_names]
        for row in path:
            self.targets[arm] = row[reorder]
            self.step()
        self.hold(.15)
        error = np.linalg.norm(xyz(self.agents[arm].tcp) - pose.p)
        if error > .018:
            for contact in self.base.scene.get_contacts():
                force = np.linalg.norm(sum((p.impulse for p in contact.points), np.zeros(3)))/self.base.scene.px.timestep
                if force > .1:
                    print('tracking contact', [b.entity.name for b in contact.bodies], float(force), flush=True)
            raise PlanningFailure(f"arm{arm}: tracking_error_{error:.3f}m")
        payload = self.payloads[arm]
        lids = [self.base.station.lid]
        if self.base.station.flash is not None:
            lids.append(self.base.station.flash.lid)
        if self.base.station.dryer is not None:
            lids.append(self.base.station.dryer.lid)
        if payload is not None and all(payload is not lid for lid in lids):
            if not bool(self.agents[arm].is_grasping(payload, min_force=.02).item()):
                raise PlanningFailure(f"arm{arm}: grasp_lost")

        return path[:, reorder].copy()

    def retrace(self, arm, forward_path):
        """Retrace an empty-arm approach after checking the changed scene."""
        if self.payloads[arm] is not None:
            raise PlanningFailure("retrace_requires_empty_gripper")
        planner = self.planner(arm)
        qpos = array(self.agents[arm].robot.get_qpos())[0]
        previous = qpos[self.indices[arm]].copy()
        reverse = np.asarray(forward_path)[::-1]
        if np.max(np.abs(reverse[0]-previous)) > .12:
            raise PlanningFailure("retrace_start_joint_mismatch")
        for row in reverse:
            count = max(1, math.ceil(np.max(np.abs(row-previous))/.012))
            for alpha in np.linspace(0, 1, count+1)[1:]:
                full = qpos.copy()
                full[self.indices[arm]] = previous + alpha*(row-previous)
                planner.robot.set_qpos(full, True)
                if planner.planning_world.is_state_colliding():
                    raise PlanningFailure("retrace_collision_in_updated_scene: " + str(
                        [(c.link_name1,c.link_name2) for c in planner.planning_world.check_collision()]))
            previous = row
        for row in reverse:
            self.targets[arm] = row.copy()
            self.step()
        self.hold(.2)

    def pick(self, name, arm=0):
        s = self.base.station
        obj = s.objects[name]
        offset = np.array([0, 0, .008]) if name == "plate" else np.array([0, 0, .018])
        target = downward(xyz(obj) + offset)
        self.gripper(arm, False)
        self.move(arm, sapien.Pose(target.p + [0, 0, .16], target.q))
        self.move(arm, target, linear=True, touch=[obj])
        self.gripper(arm, True)
        if not bool(self.agents[arm].is_grasping(obj, min_force=.02).item()):
            raise PlanningFailure(f"{name}: grasp_not_established")
        self.payloads[arm] = obj
        self.payload_contacts[arm] = [s.fixtures["source_rack"]]
        self.move(arm, sapien.Pose(target.p + [0, 0, .18], target.q), linear=True)
        self.payload_contacts[arm] = []
        return offset

    def place(self, name, target, arm=0):
        s = self.base.station
        obj = s.objects[name]
        # Preserve the measured object-to-TCP transform rather than an idealized grasp.
        object_to_tcp = sp(obj.pose).inv() * sp(self.agents[arm].tcp.pose)
        goal = sapien.Pose(target) * object_to_tcp
        clearance = .06 if name == 'plate' else .15
        self.move(arm, sapien.Pose(goal.p + [0, 0, clearance], goal.q), linear=name == 'plate')
        if self.base.task_name == 'centrifuge':
            # A tube can settle within the compliant fingers during transfer.
            # Re-estimate its grasp before entering the close-fitting rotor well.
            relative = sp(obj.pose).inv() * sp(self.agents[arm].tcp.pose)
            goal = sapien.Pose(target) * relative
            self.move(arm, sapien.Pose(goal.p+[0, 0, .065], goal.q), linear=True)
            relative = sp(obj.pose).inv() * sp(self.agents[arm].tcp.pose)
            goal = sapien.Pose(target) * relative
        support = (s.fixtures["target_rack"] if self.base.task_name == "mix_transfer"
                   else s.machine.get_links()[0])
        self.payload_contacts[arm] = [support]
        # Keep the finger pads above the cycler's rim, release over the reaction
        # block, then require the plate to settle into the measured target pose.
        release = sapien.Pose(goal.p + [0, 0, .024], goal.q) if name == 'plate' else goal
        self.move(arm, release, linear=True)
        self.gripper(arm, False)
        self.payloads[arm] = None
        self.payload_contacts[arm] = []
        self.move(arm, sapien.Pose(goal.p + [0, 0, clearance], goal.q), linear=True, touch=[obj])
        self.hold(.4)
        if not self.base.close_to(obj, target):
            raise PlanningFailure(f"{name}: placement_outside_tolerance")

    def release_lid(self, instrument=None):
        s = instrument or self.base.station
        arm = 1
        self.grippers[arm] = .15 if self.base.task_name in ('centrifuge', 'spin_coating') else .4
        self.hold(.65)
        self.payloads[arm] = None
        self.payload_contacts[arm] = []
        current = sp(self.agents[arm].tcp.pose)
        outward = -current.to_transformation_matrix()[:3, 2]
        if s.lid_angle > -.07:
            outward = np.array([0, 0, 1])
        self.move(arm, sapien.Pose(current.p + .08*outward, current.q), linear=True,
                  touch=[s.lid])
        self.hold(.35)

    def operate_lid(self, opening, instrument=None):
        s = instrument or self.base.station
        arm = 1
        angle = s.lid_angle
        # Grasp the real handle and follow its hinge arc; no qpos setter here.
        handle = s.handle_pose(angle)
        # Front approach keeps the wrist in reach as the lid rotates upward.
        grasp = handle * sapien.Pose(q=[math.sqrt(.5), -math.sqrt(.5), 0, 0])
        if s.task in ('centrifuge', 'spin_coating'):
            grasp = handle * sapien.Pose(q=[.5, -.5, -.5, -.5])
        # Spin-coater closure preserves the opening handle frame to keep the
        # forearm clear of the stand at the end of the hinge arc.
        if not opening and s.task != 'spin_coating':
            grasp = downward(handle.p)
            if s.task == 'centrifuge':
                grasp = sapien.Pose(handle.p, [0, math.sqrt(.5), -math.sqrt(.5), 0])
        approach = grasp.to_transformation_matrix()[:3, 2]
        self.grippers[arm] = .15 if s.task in ('centrifuge', 'spin_coating') else .4
        self.hold(.4)
        if opening:
            self.move(arm, sapien.Pose(s.world([.35, -.31, 1.25]), grasp.q))
        approach_distance = .04
        if not opening and s.task in ('centrifuge', 'spin_coating'):
            approach_distance = .08 if s.task == 'spin_coating' else .12
        self.move(arm, sapien.Pose(grasp.p - approach_distance*approach, grasp.q))
        self.move(arm, grasp, linear=True, touch=[s.lid])
        if not opening and s.lid_angle > -.07:
            # The approaching fingers may push the lid fully shut before a pinch
            # is established. Accept the measured closure and withdraw normally.
            self.gripper(arm, False)
            return
        self.gripper(arm, True)
        if not self.base.is_held(s.lid):
            raise PlanningFailure("lid_handle_grasp_failed")
        self.payloads[arm] = s.lid
        self.payload_contacts[arm] = [s.machine.get_links()[0], *s.objects.values()]
        relative = handle.inv() * sp(self.agents[arm].tcp.pose)
        # The flash lid is already closed within 0.07 rad. Release just before
        # the hard stop; gravity completes closure without winding wrist 1 past
        # its limit on the rightmost instrument.
        closing_angle = -.045 if s is self.base.station.flash else -.005
        for theta in np.linspace(angle, -1.63 if opening else closing_angle, 28)[1:]:
            self.move(arm, s.handle_pose(float(theta)) * relative, linear=True)
        self.release_lid(instrument=s)
        if opening and s.lid_angle > -1.25:
            raise PlanningFailure("lid_not_open")
        if not opening and s.lid_angle < -.07:
            raise PlanningFailure("lid_not_closed")

    def tighten_knob(self):
        s, arm = self.base.station, 1
        if s.knob is None:
            raise PlanningFailure('thermal_cycler_requires_operable_knob')
        frame = s.knob_pose()
        axis = frame.to_transformation_matrix()[:3, 0]
        grasp = downward(frame.p - axis*.022)
        grasp.q = [0, math.sqrt(.5), -math.sqrt(.5), 0]
        self.grippers[arm] = .25
        self.hold(.4)
        self.move(arm, sapien.Pose(grasp.p + [0, 0, .12], grasp.q))
        self.move(arm, grasp, linear=True, touch=[s.knob])
        self.gripper(arm, True)
        if not self.base.is_held(s.knob):
            raise PlanningFailure('knob_grasp_failed')
        relative = frame.inv() * sp(self.agents[arm].tcp.pose)
        for theta in np.linspace(0, 1.65, 18)[1:]:
            rotation = sapien.Pose(q=[math.cos(theta/2), math.sin(theta/2), 0, 0])
            self.move(arm, frame * rotation * relative, linear=True, touch=[s.knob])
        # Release only enough to clear the knob before lifting past the lid lever.
        names = [joint.name for joint in self.agents[arm].robot.get_active_joints()]
        measured = array(self.agents[arm].robot.get_qpos())[0, names.index('left_outer_knuckle_joint')]
        self.grippers[arm] = max(self.OPEN, float(measured)-.06)
        self.hold(.4)
        self.move(arm, sapien.Pose(xyz(self.agents[arm].tcp)+[0, 0, .12],
                                  sp(self.agents[arm].tcp.pose).q), linear=True, touch=[s.knob])
        if s.knob_angle < 1.4:
            raise PlanningFailure(f'knob_not_tightened_{s.knob_angle:.3f}')

    def press_start(self, instrument=None):
        s = instrument or self.base.station
        normal = s.button_normal
        rotation = np.eye(4)
        rotation[:3, :3] = (s.button_rotation if s.button_rotation is not None else np.eye(3)) @ np.diag([1, -1, -1])
        # Closed fingertips extend 13 mm beyond the nominal TCP; press with
        # their tips, keeping the pads and instrument shell out of overlap.
        rotation[:3, 3] = s.button_point + normal*.015
        target = sapien.Pose(rotation)
        self.gripper(1, True)
        self.move(1, sapien.Pose(target.p + normal*.12, target.q))
        self.move(1, target, linear=True, touch=[s.button])
        self.hold(.3)
        state = self.base.task_state
        started = (state.flash_running if s is self.base.station.flash else
                   state.running or state.has_cleaned or state.has_spun)
        if not started:
            raise PlanningFailure("start_button_or_interlock_failed")
        self.move(1, sapien.Pose(target.p + normal*.12, target.q), linear=True,
                  touch=[s.button])
        seconds = (self.base.lab_config.flash_seconds if s is self.base.station.flash
                   else self.base.lab_config.run_seconds)
        self.hold(seconds + .6)

    def run(self):
        self.hold(.3)
        task = self.base.task_name
        s = self.base.station
        if task == 'spin_coating':
            from .spin_coating import run_spin_coating
            run_spin_coating(self)
        elif task == 'slide_cleaning':
            from .slide_cleaning import run_cleaning
            run_cleaning(self)
        elif task == "mix_transfer":
            # Clear the target rack workspace before the left arm crosses the table.
            self.move(1, downward(s.world([.45, -.25, 1.20])))
            self.pick("sample")
            obj = s.objects["sample"]
            relative = sp(obj.pose).inv() * sp(self.agents[0].tcp.pose)
            # A 3 mm compliant preload establishes real contact despite PD tracking error.
            mix_point = np.array(s.mixer_local if s.mixer_local is not None else [0, 0, .189])
            mixer = sapien.Pose(s.instrument_point(mix_point - [0, 0, .003])) * relative
            self.move(0, sapien.Pose(mixer.p + [0, 0, .12], mixer.q))
            self.payload_contacts[0] = [s.machine]
            self.move(0, mixer, linear=True)
            self.hold(self.base.lab_config.mix_seconds + .25)
            if not self.base.task_state.has_mixed:
                raise PlanningFailure("insufficient_mixer_contact_dwell")
            self.move(0, sapien.Pose(mixer.p + [0, 0, .15], mixer.q), linear=True)
            self.payload_contacts[0] = []
            self.place("sample", self.base.target_position("sample"))
        else:
            self.move(0, downward(s.world([-.45, -.25, 1.20])))
            self.operate_lid(True)
            self.move(1, downward(s.world([.45, -.25, 1.20])))
            # Right arm operates the lid, left arm loads; both remain collision obstacles.
            for name in s.objects:
                self.pick(name)
                self.place(name, self.base.target_position(name))
            self.move(0, downward(s.world([-.45, -.25, 1.20])))
            self.operate_lid(False)
            if task == 'thermal_cycler':
                self.tighten_knob()
            self.press_start()
        self.hold(.6)
        if not self.base.task_state.success:
            raise PlanningFailure("expert_finished_without_task_success")
