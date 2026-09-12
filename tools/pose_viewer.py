#!/usr/bin/env python3
"""Read-only LAN viewer for online Tcw poses. Uses only Python's standard library."""
from __future__ import annotations

import argparse
import csv
import io
import json
import math
import mimetypes
import os
from pathlib import Path
import re
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, unquote, urlsplit

GOOD_STATES = {2, 5}
RUN_NAME = re.compile(r"run_[0-9]{8}_[0-9]{6}\Z")


def inverse_pose(t: list[float], q: list[float]) -> tuple[list[float], list[float]]:
    """Invert world-to-camera (xyzw quaternion), including rotation of translation."""
    if not all(math.isfinite(v) for v in t + q):
        raise ValueError("non-finite pose")
    norm = math.sqrt(sum(v * v for v in q))
    if not math.isfinite(norm) or norm < 1e-8:
        raise ValueError("zero quaternion")
    x, y, z, w = [v / norm for v in q]
    # Rcw transpose multiplied by -tcw.
    r = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
         [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
         [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
    position = [-sum(r[j][i] * t[j] for j in range(3)) for i in range(3)]
    return position, [-x, -y, -z, w]


def read_frames(path: Path) -> dict:
    if not path.is_file():
        return {"frames": [], "invalid_rows": 0, "map_ids_recorded": False}
    # Only complete newline-terminated rows are visible while the producer writes.
    with path.open("rb") as f:
        raw = f.read(128 * 1024 * 1024 + 1)
    if len(raw) > 128 * 1024 * 1024:
        raise ValueError("recording exceeds 128 MiB; split into shorter sessions")
    raw = raw[:raw.rfind(b"\n") + 1]
    reader = csv.DictReader(io.StringIO(raw.decode("utf-8")))
    map_ids_recorded = "map_id" in (reader.fieldnames or [])
    frames, invalid = [], 0
    for row in reader:
        try:
            state = int(row["state"])
            timestamp = float(row["timestamp_s"])
            if not math.isfinite(timestamp):
                raise ValueError("timestamp")
            map_id = int(row["map_id"]) if map_ids_recorded else None
            support = int(row.get("tracked_features", row.get("map_points", "1")))
            source_segment = int(row["source_segment"]) if "source_segment" in row else None
            position, quaternion = inverse_pose(
                [float(row[k]) for k in ("tx", "ty", "tz")],
                [float(row[k]) for k in ("qx", "qy", "qz", "qw")])
            frames.append({"frame": int(row["frame"]), "t": timestamp,
                           "state": state, "valid": state in GOOD_STATES and support > 0,
                           "source_segment": source_segment,
                           "map_id": map_id, "p": position, "q": quaternion})
        except (KeyError, TypeError, ValueError):
            # Keep an explicit break: never connect across a malformed row.
            invalid += 1
            if frames:
                frames.append({**frames[-1], "valid": False, "state": -2})
    if len(frames) > 200000:
        raise ValueError("recording exceeds 200000 frames; split into shorter sessions")
    return {"frames": frames, "invalid_rows": invalid,
            "map_ids_recorded": map_ids_recorded}


def read_cloud(path: Path, limit: int = 20000) -> list[list[float]]:
    """Read the adapter's ASCII PLY, with a bounded displayed sample."""
    if not path.is_file():
        return []
    if path.stat().st_size > 256 * 1024 * 1024:
        raise ValueError("point cloud exceeds preview limit")
    with path.open(encoding="ascii") as f:
        if f.readline().strip() != "ply" or f.readline().strip() != "format ascii 1.0":
            raise ValueError("preview supports ASCII PLY only")
        count = 0
        for _ in range(100):
            line = f.readline().strip()
            if line.startswith("element vertex "):
                count = int(line.split()[-1])
            if line == "end_header":
                break
        else:
            raise ValueError("invalid PLY header")
        stride = max(1, math.ceil(count / limit))
        points = []
        for index, line in enumerate(f):
            if index >= count:
                break
            if index % stride:
                continue
            try:
                point = [float(v) for v in line.split()[:3]]
            except ValueError:
                continue
            if len(point) == 3 and all(math.isfinite(v) for v in point):
                points.append(point)
        return points


class Store:
    def __init__(self, root: Path):
        self.root = root.resolve()
        self.runtime = self.root / "data/runtime/orbslam3"
        self.static = self.root / "web/pose_viewer"
        self.cache = {}
        self.lock = threading.Lock()

    def run(self, name: str) -> Path:
        if not RUN_NAME.fullmatch(name):
            raise ValueError("invalid run id")
        path = (self.runtime / name).resolve()
        if path.parent != self.runtime.resolve() or not path.is_dir():
            raise ValueError("run not found")
        return path

    def names(self) -> list[str]:
        return sorted((p.name for p in self.runtime.glob("run_*")
                       if p.is_dir() and not p.is_symlink() and RUN_NAME.fullmatch(p.name)),
                      reverse=True)

    def live(self, name: str) -> dict:
        run = self.run(name)
        path = run / "viewer_state.json"
        if not path.is_file():
            return {"run": name, "available": False, "active": False,
                    "reason": "该记录没有实时遥测，请使用回放。"}
        with path.open() as f:
            state = json.load(f)
        age = max(0, time.time() * 1000 - float(state["unix_ms"])) / 1000
        pid = int(state.get("pid", 0))
        process_alive = False
        if pid > 0 and os.name == "posix":
            try:
                cmdline = Path(f"/proc/{pid}/cmdline").read_bytes()
                process_alive = str(self.root / "bin/orbslam3_rervision").encode() in cmdline
            except OSError:
                pass
        active = state.get("lifecycle") == "running" and age < 3 and process_alive
        return {"run": name, "available": True, "active": active,
                "age_s": round(age, 2), "telemetry": state}

    def recording(self, name: str) -> dict:
        run = self.run(name)
        file = run / "poses.csv"
        stamp = (file.stat().st_mtime_ns, file.stat().st_size) if file.exists() else None
        with self.lock:
            cached = self.cache.get(name)
            if cached and cached[0] == stamp:
                return cached[1]
            parsed = read_frames(file)
            result = {"run": name, **parsed, "coordinate": "Twc; metres; xyzw",
                      "cloud_available": (run / "dense_map.ply").is_file() or (run / "map.ply").is_file()}
            if len(self.cache) >= 4:
                self.cache.pop(next(iter(self.cache)))
            self.cache[name] = (stamp, result)
            return result


def make_handler(store: Store):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_args):
            pass

        def send_bytes(self, body: bytes, content_type: str, status: int = 200):
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            try:
                url = urlsplit(self.path)
                query = parse_qs(url.query)
                name = query.get("run", [""])[0]
                if url.path == "/api/runs":
                    names = store.names()
                    result = {"runs": names, "latest": names[0] if names else None}
                elif url.path == "/api/run":
                    result = store.recording(name)
                elif url.path == "/api/live":
                    names = store.names()
                    if not name and names:
                        name = names[0]
                    result = store.live(name) if name else {"available": False, "active": False}
                elif url.path == "/api/cloud":
                    run = store.run(name)
                    path = run / "dense_map.ply"
                    if not path.is_file():
                        path = run / "map.ply"
                    result = {"points": read_cloud(path), "source": path.name,
                              "warning": "导出点云仅作参考：在线轨迹未随回环重新优化，历史数据也未记录地图身份。"}
                elif url.path.startswith("/api/"):
                    self.send_bytes(b'{"error":"not found"}', "application/json", 404)
                    return
                else:
                    relative = unquote(url.path).lstrip("/") or "index.html"
                    file = (store.static / relative).resolve()
                    if not file.is_relative_to(store.static.resolve()) or not file.is_file():
                        self.send_bytes(b"Not found", "text/plain", 404)
                        return
                    mime = "text/javascript" if file.suffix == ".js" else mimetypes.guess_type(file.name)[0]
                    self.send_bytes(file.read_bytes(), (mime or "application/octet-stream") + "; charset=utf-8")
                    return
                self.send_bytes(json.dumps(result, ensure_ascii=False, allow_nan=False).encode(),
                                "application/json; charset=utf-8")
            except (BrokenPipeError, ConnectionResetError):
                pass
            except (ValueError, OSError, KeyError, UnicodeError) as error:
                self.send_bytes(json.dumps({"error": str(error)}, ensure_ascii=False).encode(),
                                "application/json; charset=utf-8", 400)
    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8765)
    args = parser.parse_args()
    server = ThreadingHTTPServer((args.host, args.port), make_handler(Store(args.root)))
    print(f"SLAM pose viewer: http://{args.host}:{args.port} (read-only)", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
