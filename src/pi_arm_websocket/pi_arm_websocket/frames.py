"""URDF forward kinematics for feedback/teaching, including the three mimic joints.

No TF cache: every frame in a result is derived from the same joint snapshot.
RPY is fixed-axis XYZ: R = Rz(yaw) Ry(pitch) Rx(roll).
"""
import hashlib
import math
from pathlib import Path
import xml.etree.ElementTree as ET


def multiply(a, b):
    x, y, z, w = a
    X, Y, Z, W = b
    return [w*X+x*W+y*Z-z*Y, w*Y-x*Z+y*W+z*X,
            w*Z+x*Y-y*X+z*W, w*W-x*X-y*Y-z*Z]


def quaternion(rpy):
    r, p, y = [v/2 for v in rpy]
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy]


def rotate(q, p):
    return multiply(multiply(q, [*p, 0]), [-q[0], -q[1], -q[2], q[3]])[:3]


def pose_record(frame, position, q):
    length = math.sqrt(sum(v*v for v in q))
    x, y, z, w = q = [v/length for v in q]
    pitch = math.asin(max(-1., min(1., 2*(w*y-z*x))))
    # At gimbal lock choose roll=0, with an equivalent yaw; quaternion is retained.
    if abs(math.cos(pitch)) < 1e-7:
        roll, yaw = 0., math.atan2(2*(w*z-x*y), 1-2*(x*x+z*z))
    else:
        roll = math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))
        yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
    return dict(frame_id=frame, position_mm=[v*1000 for v in position],
                rpy_deg=[math.degrees(v) for v in (roll, pitch, yaw)], quaternion_xyzw=q)


class FrameModel:
    def __init__(self, path, joint_names):
        raw = Path(path).read_bytes()
        self.model_id = hashlib.sha256(raw).hexdigest()
        self.names = list(joint_names)
        self.joints = {}
        for element in ET.fromstring(raw).findall('joint'):
            origin, axis, mimic = element.find('origin'), element.find('axis'), element.find('mimic')
            vector = lambda attr: [float(v) for v in (origin.get(attr, '0 0 0') if origin is not None else '0 0 0').split()]
            self.joints[element.get('name')] = dict(
                parent=element.find('parent').get('link'), child=element.find('child').get('link'),
                kind=element.get('type'), xyz=vector('xyz'), rotation=quaternion(vector('rpy')),
                axis=[float(v) for v in (axis.get('xyz') if axis is not None else '1 0 0').split()],
                mimic=None if mimic is None else (mimic.get('joint'), float(mimic.get('multiplier', '1')), float(mimic.get('offset', '0'))))
        if len(self.names) != 6 or not all(name in self.joints for name in self.names):
            raise ValueError('URDF does not contain the six configured joints')

    def forward(self, positions):
        if len(positions) != 6 or not all(math.isfinite(v) for v in positions):
            raise ValueError('six finite joint angles required')
        values = dict(zip(self.names, positions))

        def angle(name, visited=()):
            if name in values:
                return values[name]
            if name in visited:
                raise ValueError('cyclic mimic joints')
            joint = self.joints[name]
            if joint['mimic'] is not None:
                other, gain, offset = joint['mimic']
                return angle(other, (*visited, name))*gain+offset
            if joint['kind'] != 'fixed':
                raise ValueError('unconfigured moving joint: '+name)
            return 0.

        transforms = {'base_link': ([0., 0., 0.], [0., 0., 0., 1.])}
        pending = dict(self.joints)
        while pending:
            progressed = False
            for name, joint in list(pending.items()):
                if joint['parent'] not in transforms:
                    continue
                p, q = transforms[joint['parent']]
                delta = rotate(q, joint['xyz'])
                position = [a+b for a, b in zip(p, delta)]
                rotation = multiply(q, joint['rotation'])
                if joint['kind'] in ('revolute', 'continuous'):
                    theta = angle(name)/2
                    norm = math.sqrt(sum(v*v for v in joint['axis']))
                    rotation = multiply(rotation, [*[v/norm*math.sin(theta) for v in joint['axis']], math.cos(theta)])
                elif joint['kind'] != 'fixed':
                    raise ValueError('unsupported URDF joint type')
                transforms[joint['child']] = (position, rotation)
                del pending[name]
                progressed = True
            if not progressed:
                raise ValueError('URDF is not connected to base_link')
        rows = []
        for name in self.names:
            child = self.joints[name]['child']
            rows.append(dict(joint_name=name, **pose_record(child, *transforms[child])))
        return dict(valid=True, base_frame='base_link', tool_frame='tool0', model_id=self.model_id,
                    joints=rows, tool=pose_record('tool0', *transforms['tool0']))

    def feedback(self, hardware):
        if (not hardware or not hardware.connected or not hardware.kinematics_valid or
                list(hardware.joint_names) != self.names or len(hardware.fresh) != 6 or
                not all(hardware.fresh) or len(hardware.motor_fresh) != 6 or not all(hardware.motor_fresh)):
            return dict(valid=False, model_id=self.model_id, base_frame='base_link', joints=[], tool=None)
        try:
            return self.forward(hardware.positions)
        except (ValueError, OverflowError):
            return dict(valid=False, model_id=self.model_id, base_frame='base_link', joints=[], tool=None)
