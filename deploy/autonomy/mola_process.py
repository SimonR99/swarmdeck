"""Bounded process supervision for the native MOLA JSONL runtime."""

from __future__ import annotations

import json
import os
from pathlib import Path
import select
import subprocess
import time
import uuid

MAX_PROTOCOL_LINE_BYTES = 64 * 1024
MAX_PROTOCOL_REQUEST_BYTES = 64 * 1024


class MolaProcessError(RuntimeError):
    """The native runtime failed or violated its process protocol."""


class NativeRequestError(MolaProcessError):
    """A well-formed native response rejected one request."""

    def __init__(self, code: str, message: str):
        super().__init__(f"native runtime {code}: {message}")
        self.code = code


class PersistentImporter:
    """Synchronous JSONL client for one supervised native importer process."""

    def __init__(
        self,
        executable: Path,
        timeout_s: float,
        *,
        max_points_per_map: int,
        max_resident_points: int,
        max_maps: int,
        max_output_bytes: int,
    ):
        self.executable = Path(executable)
        self.timeout_s = timeout_s
        self._limits = {
            "max_points_per_map": max_points_per_map,
            "max_resident_points": max_resident_points,
            "max_maps": max_maps,
            "max_output_bytes": max_output_bytes,
        }
        self._process: subprocess.Popen[bytes] | None = None
        self._read_buffer = bytearray()

    def _abort(self) -> None:
        process, self._process = self._process, None
        self._read_buffer.clear()
        if process is None:
            return
        if process.stdin is not None:
            try:
                process.stdin.close()
            except OSError:
                pass
        if process.poll() is None:
            try:
                process.terminate()
            except ProcessLookupError:
                pass
            try:
                process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                try:
                    process.kill()
                except ProcessLookupError:
                    pass
        try:
            process.wait(timeout=1.0)
        except subprocess.TimeoutExpired:
            try:
                process.kill()
            except ProcessLookupError:
                pass
            process.wait()
        if process.stdout is not None:
            process.stdout.close()

    def close(self) -> None:
        self._abort()

    def _read_message(self, deadline: float) -> dict[str, object]:
        process = self._process
        if process is None or process.stdout is None:
            raise MolaProcessError("native runtime is not running")
        fd = process.stdout.fileno()
        while True:
            newline = self._read_buffer.find(b"\n")
            if newline >= 0:
                raw = bytes(self._read_buffer[:newline])
                del self._read_buffer[: newline + 1]
                if not raw or len(raw) > MAX_PROTOCOL_LINE_BYTES:
                    raise MolaProcessError(
                        "native runtime returned an invalid response size"
                    )
                try:
                    value = json.loads(raw)
                except (UnicodeDecodeError, json.JSONDecodeError) as exc:
                    raise MolaProcessError(
                        "native runtime returned malformed JSON"
                    ) from exc
                if not isinstance(value, dict):
                    raise MolaProcessError("native runtime response must be an object")
                return value
            if len(self._read_buffer) > MAX_PROTOCOL_LINE_BYTES:
                raise MolaProcessError("native runtime response exceeds 64 KiB")
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise MolaProcessError(f"MOLA import exceeded {self.timeout_s:g}s")
            readable, _, _ = select.select((fd,), (), (), remaining)
            if not readable:
                raise MolaProcessError(f"MOLA import exceeded {self.timeout_s:g}s")
            block = os.read(fd, min(8192, MAX_PROTOCOL_LINE_BYTES + 1))
            if not block:
                code = process.poll()
                suffix = f" (status {code})" if code is not None else ""
                raise MolaProcessError(
                    f"native runtime exited before responding{suffix}"
                )
            self._read_buffer.extend(block)

    def _write_message(self, value: dict[str, object], deadline: float) -> None:
        process = self._process
        if process is None or process.stdin is None:
            raise MolaProcessError("native runtime is not running")
        payload = (
            json.dumps(value, sort_keys=True, separators=(",", ":")).encode() + b"\n"
        )
        if len(payload) > MAX_PROTOCOL_REQUEST_BYTES:
            raise MolaProcessError("native runtime request exceeds 64 KiB")
        fd = process.stdin.fileno()
        view = memoryview(payload)
        while view:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise MolaProcessError(f"MOLA import exceeded {self.timeout_s:g}s")
            _, writable, _ = select.select((), (fd,), (), remaining)
            if not writable:
                raise MolaProcessError(f"MOLA import exceeded {self.timeout_s:g}s")
            try:
                written = os.write(fd, view)
            except BlockingIOError:
                continue
            except BrokenPipeError as exc:
                raise MolaProcessError(
                    "native runtime exited while receiving a request"
                ) from exc
            if written <= 0:
                raise MolaProcessError("native runtime stopped receiving its request")
            view = view[written:]

    def _start(self, deadline: float) -> None:
        if self._process is not None and self._process.poll() is None:
            return
        self._abort()
        try:
            self._process = subprocess.Popen(
                (
                    str(self.executable),
                    "--serve",
                    "--max-points-per-map",
                    str(self._limits["max_points_per_map"]),
                    "--max-resident-points",
                    str(self._limits["max_resident_points"]),
                    "--max-maps",
                    str(self._limits["max_maps"]),
                    "--max-output-bytes",
                    str(self._limits["max_output_bytes"]),
                ),
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                # Inherit stderr for normal service logs and avoid an undrained
                # diagnostics pipe blocking the long-lived child.
                stderr=None,
            )
        except OSError as exc:
            raise MolaProcessError(f"cannot start native runtime: {exc}") from exc
        assert self._process.stdin is not None
        os.set_blocking(self._process.stdin.fileno(), False)
        try:
            ready = self._read_message(deadline)
            limits = ready.get("limits")
            if (
                ready.get("protocol") != 1
                or ready.get("type") != "ready"
                or not isinstance(limits, dict)
                or limits.get("max_line_bytes") != MAX_PROTOCOL_REQUEST_BYTES
                or limits.get("max_response_bytes") != MAX_PROTOCOL_LINE_BYTES
                or any(limits.get(name) != value for name, value in self._limits.items())
            ):
                raise MolaProcessError(
                    "native runtime returned an incompatible ready event"
                )
        except BaseException:
            self._abort()
            raise

    def apply(self, request: dict[str, object]) -> dict[str, object]:
        deadline = time.monotonic() + self.timeout_s
        self._start(deadline)
        request_id = uuid.uuid4().hex
        message = {
            "protocol": 1,
            "type": "request",
            "request_id": request_id,
            "op": "apply",
            **request,
        }
        try:
            self._write_message(message, deadline)
            response = self._read_message(deadline)
            if (
                response.get("protocol") != 1
                or response.get("type") != "response"
                or response.get("request_id") != request_id
                or response.get("op") != "apply"
                or not isinstance(response.get("ok"), bool)
            ):
                raise MolaProcessError("native runtime returned a mismatched response")
            if not response["ok"]:
                error = response.get("error")
                if not isinstance(error, dict):
                    raise MolaProcessError(
                        "native runtime returned an invalid error response"
                    )
                code, detail = error.get("code"), error.get("message")
                if not isinstance(code, str) or not isinstance(detail, str):
                    raise MolaProcessError(
                        "native runtime returned an invalid error response"
                    )
                raise NativeRequestError(code, detail[:2000])
            return response
        except NativeRequestError:
            raise
        except BaseException:
            self._abort()
            raise

    def release(self, map_id: str) -> None:
        deadline = time.monotonic() + self.timeout_s
        self._start(deadline)
        request_id = uuid.uuid4().hex
        request = {
            "protocol": 1,
            "type": "request",
            "request_id": request_id,
            "op": "release",
            "map_id": map_id,
        }
        try:
            self._write_message(request, deadline)
            response = self._read_message(deadline)
            if (
                response.get("protocol") != 1
                or response.get("type") != "response"
                or response.get("request_id") != request_id
                or response.get("op") != "release"
                or response.get("ok") is not True
                or response.get("map_id") != map_id
                or response.get("result") not in ("released", "absent")
            ):
                raise MolaProcessError("native runtime returned a mismatched response")
        except BaseException:
            self._abort()
            raise
