#!/usr/bin/env python3
"""Optional LAN proxy for the loopback-only Intervention Controller."""

from __future__ import annotations

import os
import urllib.error
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer


LISTEN_HOST = os.environ.get("BRINGUP_CONTROL_HOST", "0.0.0.0")
LISTEN_PORT = int(os.environ.get("BRINGUP_CONTROL_PORT", "8780"))
CONTROLLER_PORT = int(os.environ.get("INTERVENTION_CONTROL_PORT", "8779"))
CONTROLLER = f"http://127.0.0.1:{CONTROLLER_PORT}"
CONTROLLER_TOKEN = os.environ.get("INTERVENTION_CONTROL_TOKEN", "").strip()
FORWARDED_HEADERS = {
    "accept",
    "cache-control",
    "content-type",
    "x-control-client",
    "x-control-token",
}


class ControllerProxy(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def do_GET(self) -> None:  # noqa: N802
        self._proxy()

    def do_POST(self) -> None:  # noqa: N802
        self._proxy()

    def _proxy(self) -> None:
        length = int(self.headers.get("Content-Length", "0") or 0)
        body = self.rfile.read(length) if length else None
        headers = {
            key: value
            for key, value in self.headers.items()
            if key.lower() in FORWARDED_HEADERS
        }
        if CONTROLLER_TOKEN:
            headers["X-Control-Token"] = CONTROLLER_TOKEN
        request = urllib.request.Request(
            CONTROLLER + self.path,
            data=body,
            headers=headers,
            method=self.command,
        )
        try:
            response = urllib.request.urlopen(request, timeout=70)
        except urllib.error.HTTPError as error:
            response = error
        except (urllib.error.URLError, TimeoutError) as error:
            payload = f"Intervention Controller unavailable: {error}".encode()
            self.send_response(503)
            self.send_header("Content-Type", "text/plain; charset=utf-8")
            self.send_header("Content-Length", str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)
            return

        self.send_response(response.status)
        content_type = response.headers.get("Content-Type")
        if content_type:
            self.send_header("Content-Type", content_type)
        content_length = response.headers.get("Content-Length")
        if content_length:
            self.send_header("Content-Length", content_length)
        self.send_header("Cache-Control", response.headers.get("Cache-Control", "no-store"))
        self.end_headers()
        while True:
            chunk = response.read(64 * 1024)
            if not chunk:
                break
            self.wfile.write(chunk)
            self.wfile.flush()

    def log_message(self, format_string: str, *args: object) -> None:
        print(f"[ai_worker_control] {self.address_string()} {format_string % args}", flush=True)


if __name__ == "__main__":
    server = ThreadingHTTPServer((LISTEN_HOST, LISTEN_PORT), ControllerProxy)
    print(
        f"AI Worker Control UI proxy listening on {LISTEN_HOST}:{LISTEN_PORT}; "
        f"controller={CONTROLLER}",
        flush=True,
    )
    server.serve_forever()
