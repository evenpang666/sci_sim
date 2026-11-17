"""PhysX to FCL conversion with mesh scaling and unique obstacle names."""
import numpy as np
from sapien import physx
from mplib import Pose
from mplib.collision_detection import fcl


def collision_object(component, name, box_margin=0.0):
    shapes, poses = [], []
    for shape in component.collision_shapes:
        pose = Pose(p=shape.local_pose.p, q=shape.local_pose.q)
        if isinstance(shape, physx.PhysxCollisionShapeBox):
            geometry = fcl.Box(side=(shape.half_size + box_margin) * 2)
        elif isinstance(shape, physx.PhysxCollisionShapeConvexMesh):
            geometry = fcl.Convex(vertices=shape.vertices * shape.scale, faces=shape.triangles)
        elif isinstance(shape, physx.PhysxCollisionShapeTriangleMesh):
            geometry = fcl.BVHModel()
            geometry.begin_model()
            geometry.add_sub_model(vertices=shape.vertices * shape.scale, faces=shape.triangles)
            geometry.end_model()
        elif isinstance(shape, (physx.PhysxCollisionShapeCylinder, physx.PhysxCollisionShapeCapsule)):
            kind = fcl.Cylinder if isinstance(shape, physx.PhysxCollisionShapeCylinder) else fcl.Capsule
            geometry = kind(radius=shape.radius, lz=shape.half_length * 2)
            pose *= Pose(q=[np.sqrt(.5), 0, np.sqrt(.5), 0])
        elif isinstance(shape, physx.PhysxCollisionShapeSphere):
            geometry = fcl.Sphere(radius=shape.radius)
        elif isinstance(shape, physx.PhysxCollisionShapePlane):
            normal = pose.to_transformation_matrix()[:3, 0]
            geometry = fcl.Halfspace(n=normal, d=normal.dot(pose.p))
            pose = Pose()
        else:
            raise TypeError(f'Unsupported collision geometry: {type(shape)}')
        shapes.append(fcl.CollisionObject(geometry))
        poses.append(pose)
    if not shapes:
        return None
    return fcl.FCLObject(name, Pose(p=component.entity.pose.p, q=component.entity.pose.q), shapes, poses)
