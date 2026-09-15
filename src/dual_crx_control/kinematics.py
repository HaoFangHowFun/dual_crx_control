"""URDF-to-KDL chains with world-frame FK and geometric Jacobians."""

import numpy as np
import PyKDL as kdl
from urdf_parser_py.urdf import URDF


class CRXKinematics:
    """A six-revolute-joint chain; fixed base and TCP transforms come from URDF."""

    def __init__(self, robot_description, tip, root='world'):
        robot = URDF.from_xml_string(robot_description)
        chain_names = robot.get_chain(root, tip, joints=True, links=False, fixed=True)
        self.chain = kdl.Chain()
        self.joint_names, lower, upper, velocity = [], [], [], []
        for name in chain_names:
            joint = robot.joint_map[name]
            origin = joint.origin
            xyz = origin.xyz if origin and origin.xyz is not None else [0., 0., 0.]
            rpy = origin.rpy if origin and origin.rpy is not None else [0., 0., 0.]
            frame = kdl.Frame(kdl.Rotation.RPY(*rpy), kdl.Vector(*xyz))
            if joint.type == 'fixed':
                kj = kdl.Joint(name, kdl.Joint.Fixed)
            elif joint.type == 'revolute' and joint.mimic is None:
                axis = np.asarray(joint.axis if joint.axis is not None else [1, 0, 0], float)
                if not np.all(np.isfinite(axis)) or np.linalg.norm(axis) == 0:
                    raise ValueError(f'{name}: invalid joint axis')
                axis /= np.linalg.norm(axis)
                kj = kdl.Joint(name, frame.p, frame.M * kdl.Vector(*axis), kdl.Joint.RotAxis)
                limit = joint.limit
                if limit is None or any(v is None for v in
                                        (limit.lower, limit.upper, limit.velocity)):
                    raise ValueError(f'{name}: missing joint limits')
                self.joint_names.append(name)
                lower.append(limit.lower)
                upper.append(limit.upper)
                velocity.append(limit.velocity)
            else:
                raise ValueError(f'{name}: unsupported joint type or mimic joint')
            self.chain.addSegment(kdl.Segment(joint.child, kj, frame))
        self.lower = np.array(lower)
        self.upper = np.array(upper)
        self.velocity = np.array(velocity)
        if len(lower) != 6 or not np.all(np.isfinite([lower, upper, velocity])):
            raise ValueError('Expected six joints with finite limits')
        if np.any(self.lower >= self.upper) or np.any(self.velocity <= 0):
            raise ValueError('Invalid URDF joint limits')
        self._fk = kdl.ChainFkSolverPos_recursive(self.chain)
        self._jac = kdl.ChainJntToJacSolver(self.chain)

    def valid_joints(self, q):
        q = np.asarray(q, dtype=float)
        return (q.shape == (6,) and np.all(np.isfinite(q))
                and np.all(q >= self.lower) and np.all(q <= self.upper))

    @staticmethod
    def _joints(q):
        q = np.asarray(q, dtype=float)
        if q.shape != (6,) or not np.all(np.isfinite(q)):
            raise ValueError('Expected six finite joint positions')
        result = kdl.JntArray(6)
        for i, value in enumerate(q):
            result[i] = value
        return result

    def fk(self, q):
        """Return a 4x4 TCP pose expressed in the chain root frame."""
        frame = kdl.Frame()
        if self._fk.JntToCart(self._joints(q), frame) < 0:
            raise ValueError('KDL forward kinematics failed')
        pose = np.eye(4)
        pose[:3, :3] = [[frame.M[i, j] for j in range(3)] for i in range(3)]
        pose[:3, 3] = [frame.p[i] for i in range(3)]
        return pose

    def jacobian(self, q):
        """Return [linear; angular] velocity Jacobian at TCP, in root axes."""
        jac = kdl.Jacobian(6)
        if self._jac.JntToJac(self._joints(q), jac) < 0:
            raise ValueError('KDL Jacobian failed')
        return np.array([[jac[i, j] for j in range(6)] for i in range(6)])
