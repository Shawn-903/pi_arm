"""Durable teaching-point library. No CAN or motion APIs."""
from contextlib import contextmanager
from datetime import datetime, timezone
import fcntl
import json
import math
import os
from pathlib import Path
import tempfile
import uuid

from .protocol import CODE_INVALID_PARAMS, CODE_STATE_UNAVAILABLE, ProtocolError
from .frames import quaternion


def vector(value):
    if (not isinstance(value, list) or len(value) != 6 or
            any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in value)):
        raise ProtocolError(CODE_INVALID_PARAMS, '点位必须包含六个有限数字')
    return list(value)


class PointStore:
    def __init__(self, path):
        self.path = Path(path).expanduser()

    @contextmanager
    def locked(self):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        with self.path.with_suffix(self.path.suffix+'.lock').open('a') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            yield

    def _read(self):
        if not self.path.exists():
            return dict(schema_version=1, revision=0, points=[])
        try:
            if self.path.stat().st_size > 2*1024*1024:
                raise ValueError('file too large')
            data = json.loads(self.path.read_text())
            if (data['schema_version'] != 1 or type(data['revision']) is not int or data['revision'] < 0 or
                    not isinstance(data['points'], list) or len(data['points']) > 500):
                raise ValueError('invalid schema')
            ids = set()
            for point in data['points']:
                self.validate(point)
                if not isinstance(point['id'], str) or point['id'] in ids:
                    raise ValueError('invalid id')
                ids.add(point['id'])
            return data

        except (ValueError, KeyError, TypeError, ProtocolError) as error:
            raise ProtocolError(CODE_STATE_UNAVAILABLE, '点位文件损坏或格式不兼容，保留原文件，请检查备份') from error

    @staticmethod
    def validate(point):
        if not isinstance(point, dict) or point.get('kind') not in ('joint', 'cartesian'):
            raise ProtocolError(CODE_INVALID_PARAMS, '点位类型必须为 joint 或 cartesian')
        if not isinstance(point.get('name'), str) or not 1 <= len(point['name'].strip()) <= 80:
            raise ProtocolError(CODE_INVALID_PARAMS, '点位名称应为 1–80 个字符')
        if not isinstance(point.get('note', ''), str) or len(point.get('note', '')) > 1000:
            raise ProtocolError(CODE_INVALID_PARAMS, '备注最多 1000 字符')
        vector(point.get('values'))
        if point.get('base_frame') != 'base_link' or point.get('tool_frame') != 'tool0':
            raise ProtocolError(CODE_INVALID_PARAMS, '点位坐标系必须为 base_link / tool0')

    def list(self):
        with self.locked():
            return self._read()

    def change(self, revision, *, point=None, remove=None):
        with self.locked():
            data = self._read()
            if type(revision) is not int or revision != data['revision']:
                raise ProtocolError(CODE_STATE_UNAVAILABLE, '点位库已变化，请刷新列表后重试')
            if point is not None:
                self.validate(point)
                if len(data['points']) >= 500:
                    raise ProtocolError(CODE_INVALID_PARAMS, '最多保存 500 个点位')
                if any(p['name'] == point['name'].strip() for p in data['points']):
                    raise ProtocolError(CODE_INVALID_PARAMS, '名称已存在，请使用新名称或先删除原点位')
                point = dict(point, name=point['name'].strip(), id=uuid.uuid4().hex,
                             created_at=datetime.now(timezone.utc).isoformat())
                data['points'].append(point)
            else:
                if not isinstance(remove, str) or not any(p['id'] == remove for p in data['points']):
                    raise ProtocolError(CODE_INVALID_PARAMS, '待删除点位不存在')
                data['points'] = [p for p in data['points'] if p['id'] != remove]
            data['revision'] += 1
            encoded = json.dumps(data, ensure_ascii=False, allow_nan=False, indent=2)
            if len(encoded.encode('utf-8')) > 2*1024*1024:
                raise ProtocolError(CODE_INVALID_PARAMS, '点位库超过 2 MiB，请导出备份并删除不再需要的点位')
            # Replace only after validation and fsync. A corrupt file is never overwritten.
            name = None
            try:
                with tempfile.NamedTemporaryFile(mode='w', dir=self.path.parent, prefix='.points-', delete=False) as temp:
                    name = temp.name
                    temp.write(encoded)
                    temp.flush()
                    os.fsync(temp.fileno())
                os.replace(name, self.path)
                directory = os.open(self.path.parent, os.O_RDONLY | os.O_DIRECTORY)
                try:
                    os.fsync(directory)
                finally:
                    os.close(directory)
            finally:
                if name and os.path.exists(name):
                    os.unlink(name)
            return data

def dispatch_points(bridge, command, params):
    store = bridge.points
    if command == 'point_list':
        return store.list()
    if command == 'point_delete':
        return store.change(params.get('revision'), remove=params.get('id'))
    source = params.get('source', 'manual')
    kind = params.get('kind')
    if source not in ('manual', 'feedback', 'model') or kind not in ('joint', 'cartesian'):
        raise ProtocolError(CODE_INVALID_PARAMS, '点位类型或来源无效')
    if source == 'model' and kind != 'joint':
        raise ProtocolError(CODE_INVALID_PARAMS, '模型拖拽保存六关节目标；笛卡尔目标请手动录入')
    state = bridge.state_snapshot()
    hardware = state.hardware if state else None
    observed = bridge.frame_model.feedback(hardware)
    joints, raw = None, None
    if source == 'feedback':
        if ('values' in params or not observed['valid'] or not hardware.calibrated or
                state.task.status == 1 or hardware.axis_test_active or hardware.motion_stop_pending or
                hardware.motion_stop_failed or len(hardware.motor_moving) != 6 or any(hardware.motor_moving) or
                any(hardware.motor_errors) or any(hardware.motor_error_codes)):
            raise ProtocolError(CODE_STATE_UNAVAILABLE, '示教记录要求新鲜、有效且静止的反馈；停止后再记录，不接收客户端替代角度')
        joints = [math.degrees(v) for v in hardware.positions]
        raw = list(hardware.raw_protocol_positions_deg)
        pose = observed['tool']['position_mm'] + observed['tool']['rpy_deg']
        orientation = observed['tool']['quaternion_xyzw']
        values = joints if kind == 'joint' else pose
    else:
        values = vector(params.get('values'))
        if kind == 'joint':
            joints = values
            target = bridge.frame_model.forward([math.radians(v) for v in values])['tool']
            pose = target['position_mm'] + target['rpy_deg']
            orientation = target['quaternion_xyzw']
        else:
            pose = values
            orientation = quaternion([math.radians(v) for v in pose[3:]])
    point = dict(name=params.get('name'), note=params.get('note', ''), kind=kind, source=source,
                 values=values, joint_angles_deg=joints, pose_mm_deg=pose, raw_protocol_positions_deg=raw,
                 orientation_xyzw=orientation,
                 base_frame='base_link', tool_frame='tool0', joint_names=bridge.frame_model.names,
                 units='deg' if kind == 'joint' else 'mm,deg', rpy_convention='Rz(yaw) Ry(pitch) Rx(roll)',
                 hardware_type=bridge.hardware_type, model_id=bridge.frame_model.model_id,
                 calibration_id=hardware.calibration_id if hardware else '',
                 feedback_stamp_sec=(hardware.stamp.sec + hardware.stamp.nanosec/1e9) if source == 'feedback' else None)
    return store.change(params.get('revision'), point=point)
