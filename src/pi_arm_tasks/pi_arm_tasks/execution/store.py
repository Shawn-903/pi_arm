"""程序、执行快照和请求结果的事务存储。"""

from contextlib import contextmanager
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sqlite3
import threading
import uuid


def utc_now():
    return datetime.now(timezone.utc).isoformat()


def encode(value):
    return json.dumps(value, ensure_ascii=False, allow_nan=False, separators=(",", ":"))


class Store:
    def __init__(self, path):
        self.path = Path(path).expanduser()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.lock = threading.RLock()
        self.db = sqlite3.connect(self.path, timeout=3, check_same_thread=False)
        self.db.execute("PRAGMA journal_mode=WAL")
        self.db.execute("PRAGMA synchronous=FULL")
        self.db.executescript("""
            CREATE TABLE IF NOT EXISTS programs(id TEXT PRIMARY KEY, name TEXT UNIQUE, revision INTEGER, document TEXT);
            CREATE TABLE IF NOT EXISTS runs(id TEXT PRIMARY KEY, document TEXT);
            CREATE TABLE IF NOT EXISTS snapshots(id TEXT PRIMARY KEY, document TEXT);
            CREATE TABLE IF NOT EXISTS steps(id TEXT PRIMARY KEY, run_id TEXT, document TEXT);
            CREATE INDEX IF NOT EXISTS steps_run ON steps(run_id);
            CREATE TABLE IF NOT EXISTS requests(id TEXT PRIMARY KEY, fingerprint TEXT, response TEXT);
            CREATE TABLE IF NOT EXISTS events(sequence INTEGER PRIMARY KEY, run_id TEXT, time TEXT, kind TEXT, document TEXT);
        """)
        self.db.commit()

    @contextmanager
    def transaction(self):
        with self.lock, self.db:
            yield self.db

    def program_list(self):
        with self.lock:
            return [
                json.loads(row[0]) for row in self.db.execute("SELECT document FROM programs ORDER BY name")
            ]

    def program_get(self, identifier):
        with self.lock:
            row = self.db.execute("SELECT document FROM programs WHERE id=?", (identifier,)).fetchone()
        if not row:
            raise ValueError("程序不存在")
        return json.loads(row[0])

    def program_save(self, document, revision):
        doc = json.loads(encode(document))
        identifier = doc.get("id") or uuid.uuid4().hex
        with self.transaction() as db:
            row = db.execute("SELECT revision FROM programs WHERE id=?", (identifier,)).fetchone()
            expected = row[0] if row else 0
            if type(revision) is not int or revision != expected:
                raise ValueError("程序已变化，请刷新后重试")
            doc.update(id=identifier, revision=expected + 1, updated_at=utc_now())
            try:
                db.execute(
                    "INSERT INTO programs VALUES(?,?,?,?) ON CONFLICT(id) DO UPDATE SET "
                    "name=excluded.name,revision=excluded.revision,document=excluded.document",
                    (identifier, doc["name"], doc["revision"], encode(doc)),
                )
            except sqlite3.IntegrityError as error:
                raise ValueError("程序名称已存在") from error
        return doc

    def program_delete(self, identifier, revision):
        with self.transaction() as db:
            row = db.execute("SELECT revision FROM programs WHERE id=?", (identifier,)).fetchone()
            if not row or row[0] != revision or type(revision) is not int:
                raise ValueError("程序不存在或版本已变化")
            db.execute("DELETE FROM programs WHERE id=?", (identifier,))

    def put(self, table, document):
        if table not in ("runs", "steps"):
            raise ValueError(table)
        with self.transaction() as db:
            if table == "runs":
                db.execute("INSERT OR REPLACE INTO runs VALUES(?,?)", (document["run_id"], encode(document)))
            else:
                db.execute(
                    "INSERT OR REPLACE INTO steps VALUES(?,?,?)",
                    (document["step_id"], document["run_id"], encode(document)),
                )

    def get(self, table, identifier):
        if table not in ("runs", "steps"):
            raise ValueError(table)
        with self.lock:
            row = self.db.execute(f"SELECT document FROM {table} WHERE id=?", (identifier,)).fetchone()
        if not row:
            raise ValueError("运行记录不存在")
        result = json.loads(row[0])
        if table == "runs":
            with self.lock:
                frozen = self.db.execute(
                    "SELECT document FROM snapshots WHERE id=?", (identifier,)
                ).fetchone()
            if frozen:
                result.update(json.loads(frozen[0]))
        return result

    def snapshot(self, identifier, document):
        with self.transaction() as db:
            db.execute("INSERT INTO snapshots VALUES(?,?)", (identifier, encode(document)))

    def run_list(self, limit=100):
        with self.lock:
            return [
                {k: v for k, v in json.loads(row[0]).items() if k not in ("snapshot", "steps")}
                for row in self.db.execute(
                    "SELECT document FROM runs ORDER BY rowid DESC LIMIT ?", (min(max(int(limit), 1), 500),)
                )
            ]

    def run_export(self, identifier):
        with self.lock:
            run = self.get("runs", identifier)
            steps = [
                json.loads(row[0])
                for row in self.db.execute(
                    "SELECT document FROM steps WHERE run_id=? ORDER BY rowid", (identifier,)
                )
            ]
            events = [
                dict(sequence=row[0], time=row[1], kind=row[2], data=json.loads(row[3]))
                for row in self.db.execute(
                    "SELECT sequence,time,kind,document FROM events WHERE run_id=? ORDER BY sequence",
                    (identifier,),
                )
            ]
        return dict(schema_version=1, run=run, steps=steps, events=events)

    def event(self, run_id, kind, document):
        with self.transaction() as db:
            db.execute(
                "INSERT INTO events(run_id,time,kind,document) VALUES(?,?,?,?)",
                (run_id, utc_now(), kind, encode(document)),
            )

    def request(self, request_id, payload, response=None):
        if not isinstance(request_id, str) or not 1 <= len(request_id) <= 128:
            raise ValueError("request_id 必须是非空字符串，最多 128 字符")
        fingerprint = hashlib.sha256(
            json.dumps(payload, sort_keys=True, allow_nan=False).encode()
        ).hexdigest()
        with self.transaction() as db:
            row = db.execute("SELECT fingerprint,response FROM requests WHERE id=?", (request_id,)).fetchone()
            if row:
                if row[0] != fingerprint:
                    raise ValueError("request_id 已用于其他请求")
                return json.loads(row[1])
            if response is not None:
                db.execute("INSERT INTO requests VALUES(?,?,?)", (request_id, fingerprint, encode(response)))
        return None

    def request_get(self, request_id):
        with self.lock:
            row = self.db.execute("SELECT response FROM requests WHERE id=?", (request_id,)).fetchone()
        return json.loads(row[0]) if row else None

    def interrupt_unfinished(self):
        with self.transaction() as db:
            for identifier, raw in db.execute("SELECT id,document FROM runs").fetchall():
                doc = json.loads(raw)
                if doc["status"] in ("RUNNING", "PAUSING", "PAUSED", "STOPPING", "STARTING"):
                    doc.update(
                        status="INTERRUPTED",
                        message="执行服务已重启",
                        finished_at=utc_now(),
                        stop_confirmed=False,
                    )
                    db.execute("UPDATE runs SET document=? WHERE id=?", (encode(doc), identifier))
            for identifier, raw in db.execute("SELECT id,document FROM steps").fetchall():
                doc = json.loads(raw)
                if doc["status"] in ("PENDING", "RUNNING"):
                    doc.update(
                        status="INTERRUPTED", code=3104, message="执行服务已重启", finished_at=utc_now()
                    )
                    db.execute("UPDATE steps SET document=? WHERE id=?", (encode(doc), identifier))

    def close(self):
        with self.lock:
            self.db.close()
