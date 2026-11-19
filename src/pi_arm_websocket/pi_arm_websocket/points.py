"""持久化示教点位；用版本号检查并发编辑，以文件锁和原子替换保护存档。"""
from contextlib import contextmanager
from copy import deepcopy
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

POINT_UNITS = {'joint': 'deg', 'cartesian': 'mm,deg'}


def check_point_units(point):
    expected = POINT_UNITS[point['kind']]
    if point.get('units') != expected:
        raise ProtocolError(CODE_INVALID_PARAMS, f"点位「{point['name']}」需要使用 {expected} 单位")


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
        if 'revision' in point and (type(point['revision']) is not int or point['revision'] < 1):
            raise ProtocolError(CODE_INVALID_PARAMS, '点位 revision 必须为正整数')
        if point.get('base_frame') != 'base_link' or point.get('tool_frame') != 'tool0':
            raise ProtocolError(CODE_INVALID_PARAMS, '点位坐标系必须为 base_link / tool0')

    def list(self):
        with self.locked():
            return self._read()

    def change(self, revision, *, point=None, remove=None, update=None):
        with self.locked():
            data = self._read()
            if type(revision) is not int or revision != data['revision']:
                raise ProtocolError(CODE_STATE_UNAVAILABLE, '点位库已变化，请刷新列表后重试')
            if update is not None:
                old = next((p for p in data['points'] if p['id'] == update.get('id')), None)
                if old is None:
                    raise ProtocolError(CODE_INVALID_PARAMS, '待修改点位不存在')
                updated = dict(old, **update)
                self.validate(updated)
                if any(p['id'] != old['id'] and p['name'] == updated['name'].strip() for p in data['points']):
                    raise ProtocolError(CODE_INVALID_PARAMS, '点位名称已存在')
                updated.update(name=updated['name'].strip(), revision=old.get('revision', 1)+1,
                               updated_at=datetime.now(timezone.utc).isoformat())
                data['points'][data['points'].index(old)] = updated
            elif point is not None:
                self.validate(point)
                if len(data['points']) >= 500:
                    raise ProtocolError(CODE_INVALID_PARAMS, '最多保存 500 个点位')
                if any(p['name'] == point['name'].strip() for p in data['points']):
                    raise ProtocolError(CODE_INVALID_PARAMS, '名称已存在，请使用新名称或先删除原点位')
                point = dict(point, name=point['name'].strip(), id=uuid.uuid4().hex, revision=1,
                             created_at=datetime.now(timezone.utc).isoformat())
                data['points'].append(point)
            else:
                if not isinstance(remove, str) or not any(p['id'] == remove for p in data['points']):
                    raise ProtocolError(CODE_INVALID_PARAMS, '待删除点位不存在')
                data['points'] = [p for p in data['points'] if p['id'] != remove]
            data['revision'] += 1
            self._write(data)
            return data

    def _write(self, data):
        encoded = json.dumps(data, ensure_ascii=False, allow_nan=False, indent=2)
        if len(data['points']) > 500 or len(encoded.encode('utf-8')) > 2*1024*1024:
            raise ProtocolError(CODE_INVALID_PARAMS, '点位库上限为 500 个点位或 2 MiB')
        backup = self.path.with_suffix(self.path.suffix+'.v0.5.bak')
        if self.path.exists() and not backup.exists():
            with backup.open('xb') as out:
                out.write(self.path.read_bytes())
                out.flush()
                os.fsync(out.fileno())
        name = None
        try:
            with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8', dir=self.path.parent,
                                             prefix='.points-', delete=False) as temp:
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

    def import_points(self, document, revision, conflict='skip', *, commit=False):
        if (not isinstance(document, dict) or document.get('schema_version') != 1 or
                not isinstance(document.get('points'), list) or len(document['points']) > 500 or
                conflict not in ('skip', 'update')):
            raise ProtocolError(CODE_INVALID_PARAMS, '点位文件或冲突处理方式无效')
        incoming, ids, names = deepcopy(document['points']), set(), set()
        for point in incoming:
            self.validate(point)
            check_point_units(point)
            identifier = point.get('id')
            if not isinstance(identifier, str) or not identifier or identifier in ids or point['name'].strip() in names:
                raise ProtocolError(CODE_INVALID_PARAMS, '导入文件中点位编号或名称重复/无效')
            ids.add(identifier)
            names.add(point['name'].strip())
        with self.locked():
            data = self._read()
            if type(revision) is not int or revision != data['revision']:
                raise ProtocolError(CODE_STATE_UNAVAILABLE, '点位库已变化，请重新预检')
            conflicts, added, updated, skipped = [], 0, 0, 0
            for point in incoming:
                same_id = next((p for p in data['points'] if p['id'] == point['id']), None)
                same_name = next((p for p in data['points'] if p['name'] == point['name'].strip() and p['id'] != point['id']), None)
                if same_id or same_name:
                    conflicts.append(dict(id=point['id'], name=point['name'], reason='name' if same_name else 'id'))
                    if conflict == 'skip':
                        skipped += 1
                        continue
                    if same_name:
                        raise ProtocolError(CODE_INVALID_PARAMS, '名称属于另一个点位，不能按相同 ID 更新')
                point.update(name=point['name'].strip(), original_source=point.get('original_source', point.get('source')),
                             source='import', imported_at=datetime.now(timezone.utc).isoformat(),
                             revision=(same_id.get('revision', 1)+1 if same_id else point.get('revision', 1)))
                if same_id:
                    data['points'][data['points'].index(same_id)] = point
                    updated += 1
                else:
                    data['points'].append(point)
                    added += 1
            if len(data['points']) > 500 or len(json.dumps(data, allow_nan=False).encode()) > 2*1024*1024:
                raise ProtocolError(CODE_INVALID_PARAMS, '导入后点位库超过容量限制')
            summary = dict(revision=data['revision'], conflicts=conflicts, added=added, updated=updated, skipped=skipped)
            if commit:
                data['revision'] += 1
                self._write(data)
                return dict(data, import_summary=summary)
            return dict(summary, valid=True)

def dispatch_points(bridge, command, params):
    store = bridge.points
    if command == 'point_list':
        return store.list()
    if command == 'point_export':
        return dict(content=json.dumps(store.list(), ensure_ascii=False, allow_nan=False, indent=2))
    if command == 'point_import':
        if type(params.get('commit', False)) is not bool:
            raise ProtocolError(CODE_INVALID_PARAMS, 'commit 必须是布尔值')
        return store.import_points(params.get('document'), params.get('revision'), params.get('conflict', 'skip'),
                                   commit=params.get('commit', False))
    if command == 'point_delete':
        return store.change(params.get('revision'), remove=params.get('id'))
    old = None
    if command == 'point_update':
        if set(params) - {'id', 'revision', 'name', 'note', 'values'}:
            raise ProtocolError(CODE_INVALID_PARAMS, '点位修改仅接受名称、备注和数值')
        old = next((p for p in store.list()['points'] if p['id'] == params.get('id')), None)
        if old is None:
            raise ProtocolError(CODE_INVALID_PARAMS, '点位不存在')
        if 'values' not in params:
            return store.change(params.get('revision'), update={k:v for k,v in params.items() if k != 'revision'})
        params = dict(params, kind=old['kind'], source='manual', name=params.get('name', old['name']),
                      note=params.get('note', old.get('note', '')))
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
        if (state is None or state.state == state.DISCONNECTED or 'values' in params or
                not observed['valid'] or not hardware.calibrated or
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
                 units=POINT_UNITS[kind], rpy_convention='Rz(yaw) Ry(pitch) Rx(roll)',
                 hardware_type=bridge.hardware_type, model_id=bridge.frame_model.model_id,
                 calibration_id=hardware.calibration_id if hardware else '',
                 feedback_stamp_sec=(hardware.stamp.sec + hardware.stamp.nanosec/1e9) if source == 'feedback' else None)
    if old:
        return store.change(params.get('revision'), update=dict(point, id=old['id']))
    return store.change(params.get('revision'), point=point)
