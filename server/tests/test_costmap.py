import base64
import zlib

import numpy as np
from fastapi.testclient import TestClient

from swarmdeck_server.api.app import app
from swarmdeck_server.api.map_routes import reset_costmaps


def test_costmap_upload_is_stored_and_flipped_for_the_browser():
    reset_costmaps()
    cells = np.array([[0, 10, 100], [-1, 40, 80]], dtype=np.int8)
    query = (
        "/api/adapter/costmap?robot_id=r0&kind=local&resolution=0.05"
        "&width=3&height=2&origin_x=1.0&origin_y=-2.0&frame_id=r0%2Fmap"
    )
    with TestClient(app) as client:
        response = client.post(query, content=zlib.compress(cells.tobytes()))
        assert response.status_code == 200
        assert response.json()["seq"] == 1

        snapshot = client.get("/api/map/costmap/r0/local")
        assert snapshot.status_code == 200
        payload = snapshot.json()
        assert payload["frame_id"] == "r0/map"
        assert payload["origin"] == {"x": 1.0, "y": -2.0}
        restored = np.frombuffer(
            zlib.decompress(base64.b64decode(payload["data"])), dtype=np.int8
        ).reshape(2, 3)
        np.testing.assert_array_equal(restored, np.flipud(cells))

    reset_costmaps()
