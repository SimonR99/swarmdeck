#!/usr/bin/env bash
# Publish the running SwarmDeck stack on a public URL.
#
#   scripts/tunnel.sh                 # pick whichever tunnel tool is usable
#   scripts/tunnel.sh --tool ngrok    # insist on ngrok
#   scripts/tunnel.sh --port 5173     # something other than the Docker UI
#
# Port 5173 is the whole app: the nginx in the `ui` container serves the built
# frontend and proxies /api and /ws to the backend, so one tunnel is enough.
# Do NOT tunnel 8080 as well — it is the same backend without the frontend, and
# publishing it adds reachable surface for nothing.
#
# ngrok needs an account: `ngrok config add-authtoken <token>` once, after which
# this script prefers it. cloudflared's quick tunnels need no account at all,
# which is why they are the fallback.
set -euo pipefail

PORT=5173
TOOL=auto

while [[ $# -gt 0 ]]; do
  case "$1" in
    --port) PORT="$2"; shift 2 ;;
    --tool) TOOL="$2"; shift 2 ;;
    -h|--help) sed -n '2,14p' "$0" | sed 's/^# \?//'; exit 0 ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done

if ! curl -fs -o /dev/null --max-time 5 "http://localhost:${PORT}/" 2>/dev/null; then
  echo "Nothing is serving http://localhost:${PORT}/ — start the stack first:" >&2
  echo "    make up-server   (server + UI: the always-on core)" >&2
  echo "    make up-sim      (Gazebo fleet)" >&2
  echo "    make up-mock     (synthetic fleet, no Gazebo)" >&2
  exit 1
fi

ngrok_ready() {
  command -v ngrok >/dev/null 2>&1 || return 1
  # ngrok v3 refuses to open any tunnel without an authtoken, so an installed
  # binary is not on its own enough to choose it over cloudflared.
  ngrok config check >/dev/null 2>&1
}

if [[ "$TOOL" == auto ]]; then
  if ngrok_ready; then
    TOOL=ngrok
  elif command -v cloudflared >/dev/null 2>&1; then
    TOOL=cloudflared
  else
    echo "Neither an authenticated ngrok nor cloudflared is available." >&2
    echo "  ngrok:       install it, then \`ngrok config add-authtoken <token>\`" >&2
    echo "  cloudflared: https://developers.cloudflare.com/cloudflare-one/" >&2
    exit 1
  fi
fi

cat <<'WARNING'
------------------------------------------------------------------------
This URL has no authentication. Anyone who has it can drive the robots,
set navigation goals, and issue STOP ALL — the dashboard exposes all of
that over the same websocket it uses to draw the map.

Fine for a simulator you are watching. Before pointing a tunnel at a
stack running adapter_ros2 against real hardware, put an authenticating
proxy in front of it.
------------------------------------------------------------------------
WARNING

case "$TOOL" in
  ngrok)
    echo "Tunnelling http://localhost:${PORT} with ngrok. Ctrl-C to stop."
    exec ngrok http "$PORT"
    ;;
  cloudflared)
    echo "Tunnelling http://localhost:${PORT} with cloudflared. Ctrl-C to stop."
    echo "Quick tunnels are ephemeral: the URL changes every restart."
    exec cloudflared tunnel --no-autoupdate --url "http://localhost:${PORT}"
    ;;
  *)
    echo "unknown tool: $TOOL (expected ngrok or cloudflared)" >&2
    exit 2
    ;;
esac
