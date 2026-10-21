"""Same-arm collision admission from authored CAD and robot joint frames.

Only fixed model geometry, authored collision exclusions and candidate encoder
coordinates enter this checker. No workpiece pose or physical state is written.
The broad own-prefix omission in live scene queries does not cover these pairs.
"""
import itertools
import numpy as np
from pxr import UsdPhysics
from predictive_pair_clearance import transform_proxy, shape_overlap
try:
    import fcl
except ImportError:
    fcl = None


class SameArmClearance:
    def __init__(self, checker, *, margin_m=.0005):
        self.arm = checker.arm
        self.checker = checker
        self.rows = list(checker.pair_boxes)
        self.margin_m = float(margin_m)
        if not 0. <= self.margin_m <= .002:
            raise ValueError('Invalid same-arm clearance margin')
        self.base_path = self.arm.link_paths[0].rsplit('/', 1)[0]
        # The fixed base is intentionally not inserted in checker.pair_boxes:
        # other-arm consumers expect only bodies returned by Arm.bodies().
        # It remains an obstacle to this arm's nonadjacent moving links.
        from isaac_kinematics import world
        from predictive_pair_clearance import proxy
        from fcl_transport import mesh_data
        for prim in checker.stage.Traverse():
            if not prim.HasAPI(UsdPhysics.CollisionAPI) or UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() is False:
                continue
            body = prim
            while body and not body.HasAPI(UsdPhysics.RigidBodyAPI):
                body = body.GetParent()
            if not body or str(body.GetPath()) != self.base_path:
                continue
            data = mesh_data(prim)
            if data is None:
                raise RuntimeError('Unsupported fixed-base self-collision CAD primitive')
            vertices = data[0]
            local = np.linalg.inv(world(checker.stage, self.base_path)) @ world(checker.stage, str(prim.GetPath()))
            self.rows.append((self.base_path, str(prim.GetPath()),
                              proxy(vertices @ local[:3, :3].T + local[:3, 3])))
        filters = set()
        disabled_joint_pairs = set()
        for prim in checker.stage.Traverse():
            if prim.HasAPI(UsdPhysics.FilteredPairsAPI):
                path = str(prim.GetPath())
                filters.update((path, str(target)) for target in
                               UsdPhysics.FilteredPairsAPI(prim).GetFilteredPairsRel().GetTargets())
            if prim.IsA(UsdPhysics.Joint):
                joint = UsdPhysics.Joint(prim)
                if joint.GetCollisionEnabledAttr().Get() is False:
                    disabled_joint_pairs.update(frozenset((str(a), str(b)))
                        for a in joint.GetBody0Rel().GetTargets()
                        for b in joint.GetBody1Rel().GetTargets())
        def exempt(a, b):
            body_a, source_a, _ = a
            body_b, source_b, _ = b
            if body_a == body_b:
                return True
            if frozenset((body_a, body_b)) in disabled_joint_pairs:
                return True
            # Exact authored body/collider relationships only. Descendant path
            # prefixes do not imply collision adjacency in a nested USD robot.
            return any((x, y) in filters or (y, x) in filters
                       for x in (body_a, source_a) for y in (body_b, source_b))
        self.pairs = [(i, j) for i, j in itertools.combinations(range(len(self.rows)), 2)
                      if not exempt(self.rows[i], self.rows[j])]
        self.stats = dict(queries=0, rejected=0, candidate_pairs=len(self.pairs),
                          geometry_rows=len(self.rows),
                          backend='FCL convex CAD' if fcl else 'conservative convex supporting planes',
                          semantic_exclusions='same rigid body, direct joint collision=false, exact authored FilteredPairs')
        self.geometries = None
        if fcl:
            from fcl_transport import convex
            self.geometries = [convex(row[2][3]) for row in self.rows]
            self.objects = [fcl.CollisionObject(g) for g in self.geometries]

        from own_payload_clearance import OwnPayloadClearance
        self.own_payload_clearance = OwnPayloadClearance(self)

    def clear(self, q, opening, *, changed_bodies=None):
        self.stats['queries'] += 1
        transforms = self.arm.bodies(np.asarray(q, float), float(opening))
        transforms[self.base_path] = self.arm.base
        if any(body not in transforms for body, _, _ in self.rows):
            raise RuntimeError('Same-arm CAD has no candidate FK transform')
        if self.geometries is not None:
            for obj, (body, _, _) in zip(self.objects, self.rows):
                t = transforms[body]
                obj.setTransform(fcl.Transform(np.ascontiguousarray(t[:3, :3]),
                                               np.ascontiguousarray(t[:3, 3])))
        else:
            placed = [transform_proxy(row[2], transforms[row[0]]) for row in self.rows]
        for i, j in self.pairs:
            if changed_bodies is not None and self.rows[i][0] not in changed_bodies and self.rows[j][0] not in changed_bodies:
                continue
            if self.geometries is not None:
                a, b = self.objects[i], self.objects[j]
                # World AABB broadphase uses the same candidate transforms.
                # FCL object pair distance is exact for these convex hulls.
                distance = float(fcl.distance(a, b, fcl.DistanceRequest(), fcl.DistanceResult()))
                collision = not np.isfinite(distance) or distance <= 2*self.margin_m
            else:
                distance = None
                collision = shape_overlap(placed[i], placed[j], margin=self.margin_m)
            if collision:
                why = dict(kind='same_arm_self_collision', query=self.rows[i][1],
                           actor=self.rows[j][0], collider=self.rows[j][1],
                           distance_m=distance, required_clearance_m=2*self.margin_m)
                self.stats['rejected'] += 1
                self.stats['last_rejection'] = why
                return False, why
        ok,why=self.own_payload_clearance.clear(self.checker,transforms,changed_bodies=changed_bodies)
        if not ok:return ok,why
        completed=getattr(self.checker,'placed_part_clearance',None)
        if completed is not None:return completed.clear(self.checker,transforms,changed_bodies=changed_bodies)
        return True,None
