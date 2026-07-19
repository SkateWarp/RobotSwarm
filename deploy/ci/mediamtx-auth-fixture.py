#!/usr/bin/env python3
"""Small HTTP authorization service used by the MediaMTX CI probe."""

import json
import os
import re
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path


AUTH_PATH = "/api/viewer/auth"
MAX_REQUEST_SIZE = 16 * 1024
CANONICAL_SCENE_PATH = re.compile(
    r"session/([0-9a-f]{32})/scene-\1\Z"
)


def required_setting(name):
    value = os.environ.get(name, "")
    if not value:
        raise SystemExit(f"{name} is required")
    return value


expected_path = required_setting("MEDIAMTX_AUTH_EXPECTED_PATH")
publish_token = required_setting("MEDIAMTX_AUTH_PUBLISH_TOKEN")
read_token = required_setting("MEDIAMTX_AUTH_READ_TOKEN")
events_path = Path(required_setting("MEDIAMTX_AUTH_EVENTS"))

if not CANONICAL_SCENE_PATH.fullmatch(expected_path):
    raise SystemExit("MEDIAMTX_AUTH_EXPECTED_PATH is not a canonical scene path")


class AuthHandler(BaseHTTPRequestHandler):
    server_version = "RobotSwarmMediaAuthFixture/1"

    def do_GET(self):
        if self.path != "/health":
            self.send_error(404)
            return

        self.send_response(204)
        self.end_headers()

    def do_POST(self):
        if self.path != AUTH_PATH:
            self._finish(404, "unexpected endpoint")
            return

        try:
            length = int(self.headers.get("Content-Length", "0"))
        except ValueError:
            self._finish(400, "invalid content length")
            return

        if length <= 0 or length > MAX_REQUEST_SIZE:
            self._finish(413, "invalid request size")
            return

        try:
            request = json.loads(self.rfile.read(length))
        except (json.JSONDecodeError, UnicodeDecodeError):
            self._finish(400, "invalid json")
            return
        if not isinstance(request, dict):
            self._finish(400, "invalid request")
            return

        action = request.get("action")
        path = request.get("path")
        protocol = request.get("protocol")
        token = request.get("token")
        expected_token = {
            "publish": publish_token,
            "read": read_token,
        }.get(action)

        reason = "accepted"
        if expected_token is None:
            reason = "unexpected action"
        elif path != expected_path:
            reason = "non-canonical path"
        elif protocol not in {"rtsp", "hls"}:
            reason = "unexpected protocol"
        elif token != expected_token:
            reason = "wrong token"

        accepted = reason == "accepted"
        event = {
            "accepted": accepted,
            "action": action,
            "path": path,
            "protocol": protocol,
            "reason": reason,
        }
        with events_path.open("a", encoding="utf-8") as events:
            events.write(json.dumps(event, separators=(",", ":")) + "\n")

        self._finish(200 if accepted else 401, reason)

    def log_message(self, _format, *_args):
        # The event file is easier to assert and never records a token.
        return

    def _finish(self, status, message):
        body = message.encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "text/plain; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


events_path.parent.mkdir(parents=True, exist_ok=True)
server = ThreadingHTTPServer(("0.0.0.0", 44336), AuthHandler)
server.serve_forever()
