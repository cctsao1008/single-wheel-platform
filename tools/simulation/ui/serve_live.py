#!/usr/bin/env python3
"""Serve the Single console and stream persistent Rust SITL evidence over SSE.

The browser remains a read-only evidence consumer. This is simulation evidence only.
This bridge owns only the localhost transport/process lifetime. Plant, estimator,
controller, supervisor, and authority semantics remain in the Rust
`single_sitl_live` process.
"""

from __future__ import annotations

import argparse
import errno
import json
import math
import subprocess
import time
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]

CLIENT_DISCONNECT_ERRORS = (BrokenPipeError, ConnectionResetError, ConnectionAbortedError)
CLIENT_DISCONNECT_ERRNOS = {errno.EPIPE, errno.ECONNRESET, errno.ECONNABORTED}
CLIENT_DISCONNECT_WINERRORS = {10053, 10054, 10058}


def is_client_disconnect(error: BaseException) -> bool:
    if isinstance(error, CLIENT_DISCONNECT_ERRORS):
        return True
    if isinstance(error, OSError):
        if error.errno in CLIENT_DISCONNECT_ERRNOS:
            return True
        if getattr(error, "winerror", None) in CLIENT_DISCONNECT_WINERRORS:
            return True
    return False


def sse_payload(event: str, payload: object) -> bytes:
    data = json.dumps(payload, separators=(",", ":"), ensure_ascii=False)
    return f"event: {event}\ndata: {data}\n\n".encode("utf-8")


def live_command() -> list[str]:
    return [
        "cargo",
        "run",
        "--quiet",
        "--manifest-path",
        str(ROOT / "tools" / "sitl" / "Cargo.toml"),
        "--bin",
        "single_sitl_live",
    ]


def stop_process(process: subprocess.Popen[str] | None) -> None:
    if process is None or process.poll() is not None:
        return
    process.terminate()
    try:
        process.wait(timeout=1.0)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=1.0)


class Handler(SimpleHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(ROOT), **kwargs)

    def log_message(self, fmt: str, *args) -> None:
        print(f"[{self.log_date_time_string()}] {fmt % args}")

    def handle(self) -> None:
        try:
            super().handle()
        except OSError as error:
            if not is_client_disconnect(error):
                raise

    def finish(self) -> None:
        try:
            super().finish()
        except OSError as error:
            if not is_client_disconnect(error):
                raise

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        if parsed.path == "/api/health":
            body = json.dumps({"ok": True, "service": "single-sitl-live"}).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        if parsed.path == "/api/live":
            self.stream_live(parsed.query)
            return
        super().do_GET()

    def write_sse(self, event: str, payload: object) -> bool:
        try:
            self.wfile.write(sse_payload(event, payload))
            self.wfile.flush()
            return True
        except OSError as error:
            if is_client_disconnect(error):
                return False
            raise

    def stream_live(self, query: str) -> None:
        params = parse_qs(query)
        try:
            speed = float(params.get("speed", ["1"])[0])
            fps = float(params.get("fps", ["60"])[0])
        except ValueError:
            self.send_error(400, "speed and fps must be numeric")
            return
        if not math.isfinite(speed) or not (0.1 <= speed <= 10.0):
            self.send_error(400, "speed must be in [0.1, 10]")
            return
        if not math.isfinite(fps) or not (5.0 <= fps <= 120.0):
            self.send_error(400, "fps must be in [5, 120]")
            return

        self.close_connection = True
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream; charset=utf-8")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Connection", "close")
        self.send_header("X-Accel-Buffering", "no")
        self.end_headers()

        process: subprocess.Popen[str] | None = None
        try:
            if not self.write_sse("status", {"phase": "starting"}):
                return

            process = subprocess.Popen(
                live_command(),
                cwd=ROOT,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                bufsize=1,
            )
            assert process.stdout is not None

            meta_sent = False
            next_display_t: float | None = None
            wall_anchor: float | None = None
            sim_anchor: float | None = None
            display_period = 1.0 / fps
            last_display_t: float | None = None

            for raw in process.stdout:
                line = raw.strip()
                if not line:
                    continue
                try:
                    message = json.loads(line)
                except json.JSONDecodeError as error:
                    raise RuntimeError(f"invalid single_sitl_live JSON: {error}: {line[:160]}") from error

                message_type = message.get("type")
                if message_type == "meta":
                    meta = dict(message)
                    meta["display_fps_limit"] = fps
                    meta["speed"] = speed
                    meta["transport"] = "localhost SSE from persistent Rust SITL"
                    if not self.write_sse("meta", meta):
                        return
                    meta_sent = True
                    continue

                if message_type != "sample":
                    continue
                if not meta_sent:
                    raise RuntimeError("single_sitl_live emitted a sample before metadata")

                record = message.get("record")
                if not isinstance(record, dict):
                    raise RuntimeError("single_sitl_live sample missing record")
                t_s = float(record["time_s"])
                if next_display_t is None:
                    next_display_t = t_s
                if t_s + 1e-12 < next_display_t:
                    continue
                while next_display_t <= t_s + 1e-12:
                    next_display_t += display_period

                if wall_anchor is None:
                    wall_anchor = time.perf_counter()
                    sim_anchor = t_s
                else:
                    assert sim_anchor is not None
                    target_wall = wall_anchor + (t_s - sim_anchor) / speed
                    delay = target_wall - time.perf_counter()
                    if delay > 0:
                        time.sleep(delay)

                if not self.write_sse("sample", record):
                    return
                last_display_t = t_s

            return_code = process.wait()
            if return_code != 0:
                stderr = process.stderr.read().strip() if process.stderr else ""
                raise RuntimeError(stderr or f"single_sitl_live exited with code {return_code}")
            self.write_sse("status", {"phase": "ended", "t_s": last_display_t})
        except Exception as error:
            if is_client_disconnect(error):
                return
            self.write_sse("stream-error", {"message": str(error)})
        finally:
            stop_process(process)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=8000)
    args = parser.parse_args()

    server = ThreadingHTTPServer(("127.0.0.1", args.port), Handler)
    url = f"http://127.0.0.1:{args.port}/tools/simulation/ui/"
    print("Single Control & Evidence Console live server")
    print(f"viewer: {url}")
    print("transport: localhost only")
    print("Ctrl+C to stop")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
