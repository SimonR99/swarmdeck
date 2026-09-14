import hashlib
import importlib.util
from pathlib import Path
import struct
import time


def _observer():
    path = Path("tests/deployment/fleet_replica_acceptance.py")
    spec = importlib.util.spec_from_file_location("fleet_replica_acceptance", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_colored_chunk_acceptance_validates_planar_xyzrgba_wire_format(monkeypatch):
    observer = _observer()
    points = struct.pack("<fff", 1.0, 2.0, 3.0)
    body = (
        observer.XYZRGBA_MAGIC + struct.pack("<Q", 1) + points + bytes((9, 8, 7, 255))
    )
    digest = hashlib.sha256(body).hexdigest()
    monkeypatch.setattr(
        observer,
        "read_response",
        lambda *_args: (body, {"Content-Length": str(len(body))}, 0.01),
    )

    count, latency = observer.verify_chunk(
        "http://unused",
        digest,
        {
            "encoding": observer.XYZRGBA_ENCODING,
            "size_bytes": len(body),
            "point_count": 1,
        },
        time.monotonic() + 1,
        0.1,
    )

    assert count == 1
    assert latency == 0.01


def test_colored_chunk_cannot_be_declared_as_plain_xyz(monkeypatch):
    observer = _observer()
    body = observer.XYZRGBA_MAGIC + struct.pack(
        "<QfffBBBB", 1, 1.0, 2.0, 3.0, 9, 8, 7, 255
    )
    digest = hashlib.sha256(body).hexdigest()
    monkeypatch.setattr(observer, "read_response", lambda *_args: (body, {}, 0.0))

    try:
        observer.verify_chunk(
            "http://unused",
            digest,
            {
                "encoding": observer.XYZ_ENCODING,
                "size_bytes": len(body),
                "point_count": 1,
            },
            time.monotonic() + 1,
            0.1,
        )
    except observer.ObserverError as error:
        assert "format header" in str(error)
    else:
        raise AssertionError("colored bytes were accepted under the XYZ descriptor")
