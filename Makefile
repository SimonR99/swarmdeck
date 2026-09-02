.PHONY: help install ui ui-build server mock sim test clean tunnel \
        build-server up-server down-server \
        build-argos up-argos down-argos up-argos-gpu up-argos-dri up-argos-dev \
        build-sim up-sim down-sim \
        build-mock up-mock down-mock \
        up-agent down-agent \
        build-deploy up-deploy down-deploy \
        deploy \
        docker-up-gpu docker-up-cslam docker-down docker-logs \
        docker-ps docker-test docker-test-launch \
        test-slam install-slam slam local-ai-up local-ai-pull local-ai-shadow local-ai-eval local-ai-down

help:
	@echo "SwarmDeck"
	@echo "  make install         install ui + server dependencies (local)"
	@echo "  make ui              run frontend dev server (http://localhost:5173)"
	@echo "  make ui-build        production build of the frontend"
	@echo "  make slam            run pose-graph back-end (http://localhost:8090)"
	@echo "  make mock            run mock adapter (N=4 robots, no ROS needed)"
	@echo "  make demo            server + mock + ui, all at once"
	@echo "  make [build|up|down]-server  Docker: backend + UI only"
	@echo "  make up-argos        Docker: ARGoS + Fast-LIVO2 + SLAM/Nav2/adapter_sim"
	@echo "  make up-argos-gpu    as up-argos, rendering on an NVIDIA GPU"
	@echo "  make up-argos-dri    as up-argos, rendering on an Intel/AMD GPU"
	@echo "  make up-argos-dev    FAST: 3 robots, drift odometry, no estimator"
	@echo "  make up-argos-bistro Docker: 4 robots in Amazon Lumberyard Bistro scenario"
	@echo "  make [build|up|down]-sim     Docker: LEGACY Gazebo/SLAM/Nav2/adapter_sim"
	@echo "  make [build|up|down]-mock    Docker: synthetic mock fleet (needs server up)"
	@echo "  make [up|down]-agent         Docker: Cortex AI sidecar (opt-in, see compose)"
	@echo "  make [build|up|down]-deploy  REAL FLEET: server + UI + Zenoh router (see docs/operations/hardware-bringup.md)"
	@echo "  make deploy ROBOT=botman    operator-side sync + override + build + reset + up"
	@echo "  make deploy ROBOT=all       deploy every profile in deploy/robots/"
	@echo "  make docker-up-gpu   full stack, ARGoS rendering on an NVIDIA GPU"
	@echo "  make docker-up-cslam as docker-up-gpu, plus Swarm-SLAM collaborative SLAM"
	@echo "  make docker-down     stop everything (server, ui, sim, mock)"
	@echo "  make docker-logs     follow Docker logs"
	@echo "  make docker-ps       list running containers"
	@echo "  make docker-test     run backend tests inside Docker"
	@echo "  make docker-test-launch  build every LaunchDescription in the ROS image"
	@echo "  make local-ai-up     start the optional private Ollama planner service"
	@echo "  make local-ai-pull   pull LOCAL_MODEL for shadow-planner evaluation"
	@echo "  make local-ai-shadow run Ollama beside AGY without changing chat behavior"
	@echo "  make local-ai-eval   score the local planner without executing robot tools"
	@echo "  make sim             launch the ARGoS simulation (host ROS + host argos3)"
	@echo "  make visual-test     capture per-robot RGB, depth and lidar frames as PNGs"
	@echo "  make visual-test-bistro capture Bistro scenario RGB, depth and lidar frames"
	@echo "  make tunnel          publish the running stack on a public URL"
	@echo "  make test            run all tests (local venv)"
	@echo "  make install-slam    bootstrap the SLAM back-end venv (Python 3.12)"
	@echo "  make test-slam       run collaborative SLAM back-end tests only"

install:
	cd ui && npm install
	cd server && python3 -m venv .venv && .venv/bin/pip install -q -e "../adapters/protocol" -e ".[dev]"

ui:
	cd ui && npm run dev

ui-build:
	cd ui && npm run build

# ROS must NOT be on the path — the backend is ROS-free by design.
CLEANENV = env -u PYTHONPATH -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH

server:
	cd server && $(CLEANENV) SWARMDECK_SLAM_URL=http://127.0.0.1:8090 .venv/bin/python -m swarmdeck_server

slam:
	cd slam && $(CLEANENV) SWARMDECK_SERVER_URL=http://127.0.0.1:8080 .venv/bin/python -m swarmdeck_slam --host 127.0.0.1 --port 8090

N ?= 4
mock:
	cd adapters/adapter_mock && $(CLEANENV) ../../server/.venv/bin/python mock_adapter.py --robots $(N)

demo:
	@echo "Starting server, mock adapter and ui..."
	@$(MAKE) -j3 server mock ui

# Host development. `launch_argos:=true` starts argos3 from the launch file
# rather than leaving it to the `argos` container, and runtime_dir has to be
# somewhere writable that is SHORT: a Unix socket path over 107 bytes fails to
# bind, and the failure names neither the socket nor the limit.
#
# Needs the ARGoS fork installed (see docs/architecture/simulation.md) and
# Fast-LIVO2 reachable on RUNTIME_DIR/uf.sock. Without the estimator the
# fleet has no odometry at all; `make visual-test` is the sensor-only path that
# needs neither.
RUNTIME_DIR ?= /tmp/swarmdeck
sim:
	mkdir -p $(RUNTIME_DIR)
	cd swarmdeck_ros && . install/setup.bash && \
	  ros2 launch swarmdeck_bringup session.launch.py \
	    sim_backend:=argos launch_argos:=true runtime_dir:=$(RUNTIME_DIR)

# Sensor frames only: starts ARGoS with the synthetic drift model in place of
# the estimator, drives the fleet gently, and writes a contact sheet. This is
# the check that the photorealistic path is wired end to end, and it is the one
# that catches the failures that are otherwise silent — a robot with no glTF
# descriptor is invisible to its neighbours, a camera whose frames never arrive
# publishes black, and neither logs anything.
visual-test:
	python3 tests/integration/run_visual_test.py

visual-test-bistro:
	python3 tests/integration/run_visual_test.py --config configs/4robot_bistro.yaml

# One tunnel to port 5173 publishes the whole app: nginx there serves the UI and
# proxies /api and /ws. The URL has no authentication — see the script.
tunnel:
	./scripts/tunnel.sh

test:
	$(CLEANENV) server/.venv/bin/pytest -q
	$(MAKE) test-slam
	cd ui && npm run check

# The SLAM back-end runs its own interpreter: gtsam 4.2.2 segfaults under the
# numpy 2.x the server venv is built on, so it is pinned to Python 3.12 and kept
# in a separate distribution. Bootstrapped with `make install-slam`.
test-slam:
	cd slam && $(CLEANENV) .venv/bin/python -m pytest tests/ -q

install-slam:
	cd slam && uv venv --allow-existing --python 3.12 .venv && \
	  uv pip install --python .venv/bin/python -e ../adapters/protocol -e ".[dev]"

# Full stack with the simulator rendering on an NVIDIA GPU.
GPU_COMPOSE = -f deploy/compose/docker-compose.yml -f deploy/compose/docker-compose.gpu.yml

# --- Base Compose Command
# Pin the project name so every Make target shares one network and one set of
# containers regardless of the directory from which Compose derives its name.
COMPOSE_PROJECT ?= swarmdeck
export SSH_AUTH_SOCK_REAL ?= $(shell readlink -f $${SSH_AUTH_SOCK:-/dev/null} 2>/dev/null || echo "/dev/null")
COMPOSE ?= docker compose -p $(COMPOSE_PROJECT) -f deploy/compose/docker-compose.yml

# --- operator-side physical robot deployment
# ROBOT is one profile name or `all`; DEPLOY_ARGS can carry --dry-run,
# --no-build, --no-reset, etc. The matrix and implementation live in
# scripts/deploy and deploy/robots/, not in per-robot Make recipes.
deploy:
	./scripts/deploy $(if $(ROBOT),$(ROBOT),all) $(DEPLOY_ARGS)

# --- server + UI: the always-on core. Everything else depends on it being up.
build-server:
	$(COMPOSE) build server ui slam

up-server:
	$(COMPOSE) up --build -d server ui slam
	@echo "SwarmDeck UI:     http://localhost:5173"
	@echo "Backend API:      http://localhost:8080/api/config"
	@echo "SLAM back-end:    http://localhost:8090/status"

down-server:
	$(COMPOSE) stop server ui slam
	$(COMPOSE) rm -f server ui slam

# --- simulated fleet: ARGoS + Fast-LIVO2 + SLAM/Nav2 + adapter_sim.
#
# Three services around one volume; see docker-compose.yml. The first build is
# long: Dockerfile.argos compiles the ARGoS fork against the Filament SDK.
build-argos:
	$(COMPOSE) --profile argos build argos sim fast_livo2

up-argos:
	$(COMPOSE) --profile argos up --build -d server ui slam argos sim fast_livo2
	@echo "SwarmDeck UI:     http://localhost:5173"
	@echo "SLAM back-end:    http://localhost:8090/status"
	@echo "Fleet:            ARGoS + Fast-LIVO2 (allow ~90s for robots to appear)"
	@echo "Rendering:        software Vulkan; use up-argos-gpu or up-argos-dri for a device"

# --- development fleet: three robots, synthetic drift odometry, no estimator.
#
# The configuration to work against. Two independent savings, both real and
# both measured:
#
#   3 robots instead of 4   almost all the cost is per robot (a SLAM Toolbox
#                           instance, a Nav2 stack, an estimator and a set of
#                           rendered sensors each), and configs/3robot.yaml
#                           keeps one of every platform rather than dropping
#                           the Spot.
#   odometry:=drift         ~4x, measured 0.230x against 0.056x real time. It
#                           takes Fast-LIVO2 out of the lockstep exchange,
#                           and the estimator service is not started at all.
#
# What it costs: the drift model perturbs ground-truth motion with Gaussian
# noise. It cannot slip a wheel against an obstacle or lose a scan to
# degeneracy, which are the failures swarmdeck-slam exists to survive. Use this
# to develop; use `make up-argos` to judge whether mapping actually works.
up-argos-dev:
	SWARMDECK_CONFIG=/app/configs/3robot.yaml \
	SWARMDECK_ODOMETRY=drift \
	  docker compose -p $(COMPOSE_PROJECT) $(DRI_COMPOSE) --profile argos \
	    up --build -d server ui slam duck_detector mediamtx sim argos
	@echo "SwarmDeck UI:     http://localhost:5173"
	@echo "Fleet:            3 robots, drift odometry, no Fast-LIVO2"
	@echo "Fidelity:         development only -- see docs/architecture/simulation.md"

DRI_COMPOSE = -f deploy/compose/docker-compose.yml -f deploy/compose/docker-compose.dri.yml
up-argos-dri:
	docker compose -p $(COMPOSE_PROJECT) $(DRI_COMPOSE) --profile argos up --build -d
	@echo "Fleet:            ARGoS on the Intel/AMD render node"

up-argos-gpu:
	docker compose -p $(COMPOSE_PROJECT) $(GPU_COMPOSE) --profile argos up --build -d
	@echo "Fleet:            ARGoS on the NVIDIA GPU"

up-argos-bistro:
	SWARMDECK_CONFIG=/app/configs/4robot_bistro.yaml \
	  $(COMPOSE) --profile argos up --build -d server ui slam argos sim fast_livo2
	@echo "SwarmDeck UI:     http://localhost:5173"
	@echo "SLAM back-end:    http://localhost:8090/status"
	@echo "Fleet:            ARGoS + Fast-LIVO2 (Bistro environment)"

up-argos-bistro-dri:
	SWARMDECK_CONFIG=/app/configs/4robot_bistro.yaml \
	  docker compose -p $(COMPOSE_PROJECT) $(DRI_COMPOSE) --profile argos up --build -d
	@echo "Fleet:            ARGoS Bistro on the Intel/AMD render node"

up-argos-bistro-gpu:
	SWARMDECK_CONFIG=/app/configs/4robot_bistro.yaml \
	  docker compose -p $(COMPOSE_PROJECT) $(GPU_COMPOSE) --profile argos up --build -d
	@echo "Fleet:            ARGoS Bistro on the NVIDIA GPU"

down-argos:
	$(COMPOSE) --profile argos stop argos sim fast_livo2
	$(COMPOSE) --profile argos rm -f argos sim fast_livo2

# --- simulated fleet: Gazebo + SLAM/Nav2 + adapter_sim. `depends_on: server` in
# docker-compose.yml means this brings the server up too if it isn't already.
build-sim:
	$(COMPOSE) --profile gazebo build gazebo

up-sim:
	$(COMPOSE) --profile gazebo up --build -d server ui slam gazebo
	@echo "SwarmDeck UI:     http://localhost:5173"
	@echo "SLAM back-end:    http://localhost:8090/status"
	@echo "Fleet:            Gazebo + adapter_sim (allow ~45s for robots to appear)"

down-sim:
	$(COMPOSE) --profile gazebo stop gazebo
	$(COMPOSE) --profile gazebo rm -f gazebo

# --- synthetic fleet: no simulator, no ROS. Also depends_on: server.
build-mock:
	$(COMPOSE) --profile mock build mock

up-mock:
	$(COMPOSE) --profile mock up --build -d mock
	@echo "Fleet:            mock adapter (no simulator)"

down-mock:
	$(COMPOSE) --profile mock stop mock
	$(COMPOSE) --profile mock rm -f mock

# --- Cortex: the AI fleet-intelligence sidecar, on port 8085.
#
# Opt in, because it bind-mounts an operator's personal tooling by absolute
# path; see the service in docker-compose.yml. Read those mounts before the
# first run on a new machine.
up-agent:
	$(COMPOSE) --profile agent up --build -d agent
	@echo "Cortex:           http://localhost:8085/health"

down-agent:
	$(COMPOSE) --profile agent stop agent
	$(COMPOSE) --profile agent rm -f agent

# --- real fleet deployment: server + UI + a Zenoh router, so robots on other
# machines can reach both the backend and each other's ROS graph. No Gazebo, no
# cslam, no mock — this is the actual bring-up target.
ZENOH_COMPOSE = -p $(COMPOSE_PROJECT) -f deploy/compose/docker-compose.yml -f deploy/compose/docker-compose.zenoh.yml

build-deploy:
	docker compose $(ZENOH_COMPOSE) build server ui

up-deploy:
	SWARMDECK_CONFIG=/app/configs/hardware_fleet.yaml \
	  SWARMDECK_SLAM_REGISTRATION_MODE=graph \
	  SWARMDECK_SLAM_ANCHOR_ROBOT=aslan_0 \
	  SWARMDECK_SLAM_CAPTURE_DIR=/app/sessions/captures/hardware-live \
	  SWARMDECK_SLAM_RESTORE_CAPTURE=true \
	  docker compose $(ZENOH_COMPOSE) up --build -d server ui mediamtx zenoh-router slam
	@echo "SwarmDeck UI:     http://localhost:5173"
	@echo "Backend API:      http://localhost:8080/api/config"
	@echo "SLAM back-end:    http://localhost:8090/status"
	@echo "Zenoh router:     tcp/<this-host>:7447"
	@echo "On each robot:    RMW_IMPLEMENTATION=rmw_zenoh_cpp, session config -> tcp/<this-host>:7447"
	@echo "                  then: adapter_ros2.py --robot-id <id> --config <cfg> --host <this-host>"

down-deploy:
	docker compose $(ZENOH_COMPOSE) down --remove-orphans

# Local inference is intentionally opt-in. It is not a replacement for the
# default AGY coding/runtime path until its shadow decisions pass evaluation.
LOCAL_MODEL ?= qwen3.5:9b-q4_K_M
local-ai-up:
	$(COMPOSE) --profile local-ai up -d ollama

local-ai-pull: local-ai-up
	$(COMPOSE) --profile local-ai exec ollama ollama pull $(LOCAL_MODEL)

local-ai-shadow: local-ai-pull
	$(COMPOSE) --profile agent build agent
	CORTEX_SHADOW_PLANNER=true \
	  CORTEX_PLANNER_PROVIDER=ollama \
	  CORTEX_PLANNER_MODEL=$(LOCAL_MODEL) \
	  $(COMPOSE) --profile agent --profile local-ai up -d --no-deps agent
	@echo "Cortex:           AGY live, Ollama $(LOCAL_MODEL) shadow planning"

local-ai-eval: local-ai-shadow
	$(COMPOSE) --profile agent --profile local-ai exec -T agent \
	  python /app/agent/evals/run_planner_eval.py --model $(LOCAL_MODEL)

local-ai-down:
	$(COMPOSE) --profile local-ai stop ollama

# The default full stack, on a GPU. Kept as an alias for up-argos-gpu because
# it is the command every doc and every habit already reaches for.
docker-up-gpu: up-argos-gpu
	@echo "Backend API:      http://localhost:8080/api/config"

# Collaborative SLAM: the fleet plus Swarm-SLAM.
CSLAM_COMPOSE = $(GPU_COMPOSE) -f deploy/compose/docker-compose.cslam.yml
docker-up-cslam:
	SWARMDECK_CONFIG=/app/configs/4robot_3d.yaml SLAM_BACKEND=rtabmap \
	  docker compose $(CSLAM_COMPOSE) --profile gazebo up --build -d
	@echo "SwarmDeck UI:     http://localhost:5173"
	@echo "Fleet:            Gazebo (GPU) + RTAB-Map + Swarm-SLAM"

docker-down:
	$(COMPOSE) --profile argos --profile gazebo --profile mock --profile agent down

docker-logs:
	$(COMPOSE) --profile argos --profile gazebo --profile mock --profile agent logs -f

docker-ps:
	$(COMPOSE) --profile argos --profile gazebo --profile mock --profile agent ps

docker-test:
	$(COMPOSE) build server
	$(COMPOSE) run --rm --no-deps server python -m pytest /app/server/tests -q

# Launch files are plain Python that nothing executes until the simulator starts,
# so an undefined name in one costs a full stack startup to find. This builds
# every LaunchDescription in the ROS image, which takes under a second.
docker-test-launch:
	$(COMPOSE) --profile argos build sim
	$(COMPOSE) --profile argos run --rm --no-deps --entrypoint bash sim -lc \
	  'source /opt/ros/jazzy/setup.bash && source /app/swarmdeck_ros/install/setup.bash && \
	   cd /app && python3 -m pytest swarmdeck_ros/src/swarmdeck_bringup/test -q'

clean:
	rm -rf ui/node_modules ui/dist server/.venv swarmdeck_ros/{build,install,log} argos/build
	$(COMPOSE) --profile argos --profile gazebo --profile mock --profile agent down --rmi local --volumes --remove-orphans 2>/dev/null || true
