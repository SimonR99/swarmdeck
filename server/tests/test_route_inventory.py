"""Pin the public route and inbound message inventory before server moves.

Dropping a router, changing a method/path, or losing a dispatcher branch must
change this inventory deliberately. Behavioral protocol tests live in test_stack.
"""

import ast
import inspect

from swarmdeck_server.api.app import app
from swarmdeck_server.api.adapter_socket import handle_adapter_message
from swarmdeck_server.api.gui_socket import handle_gui_message

EXPECTED_ROUTES = set("""
DELETE /api/fleet/{robot_id}
GET /api/autonomy/chunks/{digest}
GET /api/autonomy/replicas
GET /api/autonomy/replicas/components
GET /api/autonomy/replicas/components/live/{session_id}
GET /api/autonomy/replicas/components/view/{session_id}
GET /api/autonomy/replicas/view/{robot_id}/{session_id}
GET /api/autonomy/replicas/{robot_id}/{session_id}
GET /api/camera/{robot_id}
GET /api/config
GET /api/detection/classes
GET /api/detections
GET /api/fleet
GET /api/map/gaussians
GET /api/map/optimized
GET /api/map/optimized/{scope}
GET /api/map/reset/{robot_id}
GET /api/map/status
GET /api/robot/{robot_id}/vision
GET /api/session
GET /api/settings
GET /api/sim/reset
GET /docs
GET /docs/oauth2-redirect
GET /openapi.json
GET /redoc
HEAD /api/autonomy/chunks/{digest}
HEAD /docs
HEAD /docs/oauth2-redirect
HEAD /openapi.json
HEAD /redoc
POST /api/adapter/camera
POST /api/autonomy/replicas
POST /api/autonomy/replicas/components/live/{session_id}/goal
POST /api/fleet/{robot_id}/discard
POST /api/map/reset
POST /api/map/reset/{robot_id}
POST /api/robot/{robot_id}/body
POST /api/robot/{robot_id}/cancel
POST /api/robot/{robot_id}/drive
POST /api/robot/{robot_id}/goal
POST /api/robot/{robot_id}/stop
POST /api/session/start
POST /api/session/stop
POST /api/sim/reset
PUT /api/autonomy/chunks/{digest}
PUT /api/settings
WEBSOCKET /adapter
WEBSOCKET /ws
""".splitlines()[1:])


def route_inventory(router, prefix=""):
    # FastAPI versions either flatten includes or keep an included-router node.
    for route in router.routes:
        if hasattr(route, "original_router"):
            yield from route_inventory(
                route.original_router, prefix + route.include_context.prefix
            )
        else:
            for method in getattr(route, "methods", None) or {"WEBSOCKET"}:
                yield f"{method} {prefix}{route.path}"


def test_http_and_websocket_route_inventory():
    routes = list(route_inventory(app))
    assert len(routes) == len(set(routes)), "duplicate method/path registration"
    assert set(routes) == EXPECTED_ROUTES


def message_types(handler):
    """Inventory literal type branches, not arbitrary strings in payloads."""
    result = set()
    for node in ast.walk(ast.parse(inspect.getsource(handler))):
        if not isinstance(node, ast.Compare):
            continue
        if not isinstance(node.left, ast.Name) or node.left.id != "kind":
            continue
        value = ast.literal_eval(node.comparators[0])
        result.update([value] if isinstance(value, str) else value)
    return result


def test_inbound_websocket_message_inventory():
    assert message_types(handle_adapter_message) == {
        "hello",
        "robot_state",
        "detections",
        "reset_done",
    }
    assert message_types(handle_gui_message) == {
        "acknowledge_alert",
        "body_command",
        "cancel_goal",
        "detection_accept",
        "detection_clear_proposals",
        "detection_delete_all",
        "detection_forget",
        "detection_forget_all",
        "detection_ignore",
        "detection_merge",
        "detection_unignore",
        "discard_robot",
        "drive",
        "remove_robot",
        "report_target",
        "reset_sim",
        "return_home",
        "select_robots",
        "start_explore",
        "stop_all",
        "stop_explore",
        "switch_camera",
    }
