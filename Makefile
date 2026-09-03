# ==============================================================================
# SwarmDeck Makefile
# ==============================================================================
#
# Primary entry point for local development, simulation bring-up, testing,
# and operator-side physical robot deployment.
#
# Quick Reference:
#   make help                  Show all available targets and options
#   make up-server             Start Web UI + Server + SLAM back-end
#   make up-argos              Start ARGoS simulation (software Vulkan)
#   make up-argos-gpu          Start ARGoS simulation (NVIDIA GPU)
#   make up-argos-bistro-gpu   Start ARGoS Bistro scenario (NVIDIA GPU)
#   make up-sim SCENARIO=bistro RENDER=gpu  Modular simulation bring-up
#   make deploy ROBOT=botman   Deploy stack to a physical robot over SSH
#   make docker-down           Stop all Docker containers
# ==============================================================================

.PHONY: help \
        install ui ui-build server slam install-slam mock demo \
        build-server up-server down-server \
        build-argos up-argos down-argos up-argos-gpu up-argos-dri up-argos-dev \
        up-argos-bistro up-argos-bistro-dri up-argos-bistro-gpu \
        build-sim up-sim down-sim \
        build-mock up-mock down-mock \
        up-agent down-agent \
        build-deploy up-deploy down-deploy deploy \
        docker-up-gpu docker-up-cslam docker-down docker-logs docker-ps \
        docker-test docker-test-launch \
        test test-slam visual-test visual-test-bistro sim tunnel clean \
        local-ai-up local-ai-pull local-ai-shadow local-ai-eval local-ai-down

# ------------------------------------------------------------------------------
# Configurable Parameters (Override inline: make up-sim SCENARIO=bistro RENDER=gpu)
# ------------------------------------------------------------------------------
COMPOSE_PROJECT ?= swarmdeck
SCENARIO        ?= default        # default (4robot) | bistro | 3robot | dev | path/to/cfg.yaml
RENDER          ?= software       # software | gpu (nvidia) | dri (intel/amd)
ODOMETRY        ?= fast_livo2     # fast_livo2 | drift
TARGETS         ?= 10             # Perception targets to scatter
EXPLORE         ?= 0              # Initial autonomous exploration seconds
ROBOT           ?= all            # Target robot profile for deploy (botman, aslan, scout, spot, asimov, all)
DEPLOY_ARGS     ?=                # Additional flags for scripts/deploy (--dry-run, --no-build, etc.)
N               ?= 4              # Number of robots for mock adapter
LOCAL_MODEL     ?= qwen3.5:9b-q4_K_M

# Compose definitions
export SSH_AUTH_SOCK_REAL ?= $(shell readlink -f $${SSH_AUTH_SOCK:-/dev/null} 2>/dev/null || echo "/dev/null")
COMPOSE       ?= docker compose -p $(COMPOSE_PROJECT) -f deploy/compose/docker-compose.yml
GPU_COMPOSE   ?= -f deploy/compose/docker-compose.yml -f deploy/compose/docker-compose.gpu.yml
DRI_COMPOSE   ?= -f deploy/compose/docker-compose.yml -f deploy/compose/docker-compose.dri.yml
ZENOH_COMPOSE ?= -p $(COMPOSE_PROJECT) -f deploy/compose/docker-compose.yml -f deploy/compose/docker-compose.zenoh.yml
CSLAM_COMPOSE ?= $(GPU_COMPOSE) -f deploy/compose/docker-compose.cslam.yml

# Environment hygiene: prevent host ROS variables from poisoning non-ROS Python backends
CLEANENV = env -u PYTHONPATH -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH

# ------------------------------------------------------------------------------
# 1. Help & Discovery
# ------------------------------------------------------------------------------
help:
	@echo "================================================================================"
	@echo "SwarmDeck Development & Operations"
	@echo "================================================================================"
	@echo "Local Development:"
	@echo "  make install             Install frontend (npm) and backend Python dependencies"
	@echo "  make ui                  Run frontend dev server (http://localhost:5173)"
	@echo "  make ui-build            Production build of the Svelte frontend"
	@echo "  make server              Run backend server on host (http://localhost:8080)"
	@echo "  make slam                Run pose-graph SLAM backend (http://localhost:8090)"
	@echo "  make mock                Run synthetic mock fleet adapter (N=4, no ROS)"
	@echo "  make demo                Launch server + mock + ui simultaneously on host"
	@echo ""
	@echo "Simulation (Docker Compose + ARGoS 3):"
	@echo "  make up-sim              Modular launch: SCENARIO=[default|bistro|3robot] RENDER=[software|gpu|dri]"
	@echo "  make up-argos            4-robot indoor scene with software Vulkan (portable)"
	@echo "  make up-argos-gpu        4-robot indoor scene with NVIDIA GPU hardware acceleration"
	@echo "  make up-argos-dri        4-robot indoor scene with Intel/AMD DRI hardware acceleration"
	@echo "  make up-argos-bistro     Amazon Bistro scene with software Vulkan"
	@echo "  make up-argos-bistro-gpu Amazon Bistro scene with NVIDIA GPU acceleration"
	@echo "  make up-argos-bistro-dri Amazon Bistro scene with Intel/AMD DRI acceleration"
	@echo "  make up-argos-dev        Fast dev mode: 3 robots, synthetic drift, no estimator"
	@echo "  make down-argos          Stop ARGoS simulation stack"
	@echo ""
	@echo "Core Backend Services:"
	@echo "  make up-server           Docker: Backend server + Web UI + SLAM back-end"
	@echo "  make down-server         Docker: Stop core backend services"
	@echo "  make up-mock             Docker: Synthetic mock fleet adapter"
	@echo ""
	@echo "Physical Fleet Deployment (over SSH):"
	@echo "  make up-deploy           Start operator stack (server + UI + SLAM + Zenoh router)"
	@echo "  make down-deploy         Stop operator deployment stack"
	@echo "  make deploy ROBOT=name   Deploy to robot: botman, aslan, scout, spot, asimov, all"
	@echo "                           Options: DEPLOY_ARGS='--dry-run' (or: --no-build, --no-reset)"
	@echo ""
	@echo "Docker Stack Utilities:"
	@echo "  make docker-down         Stop all running SwarmDeck containers"
	@echo "  make docker-logs         Follow logs across all containers"
	@echo "  make docker-ps           List running containers and health status"
	@echo "  make docker-test         Run pytest test suite inside Docker"
	@echo "  make docker-test-launch  Validate all ROS 2 launch files across robots"
	@echo ""
	@echo "Testing & Quality:"
	@echo "  make test                Run server unit tests, SLAM tests, and UI checks"
	@echo "  make test-slam           Run collaborative SLAM tests only"
	@echo "  make visual-test         Capture RGB/Depth/LiDAR contact sheet from ARGoS"
	@echo "  make visual-test-bistro  Capture contact sheet from Bistro environment"
	@echo "  make clean               Clean build artifacts, venvs, and local Docker caches"
	@echo "================================================================================"

# ------------------------------------------------------------------------------
# 2. Local Development (Host OS)
# ------------------------------------------------------------------------------
install:
	cd ui && npm install
	cd server && python3 -m venv .venv && .venv/bin/pip install -q -e "../adapters/protocol" -e ".[dev]"

ui:
	cd ui && npm run dev

ui-build:
	cd ui && npm run build

server:
	cd server && $(CLEANENV) SWARMDECK_SLAM_URL=http://127.0.0.1:8090 .venv/bin/python -m swarmdeck_server

slam:
	cd slam && $(CLEANENV) SWARMDECK_SERVER_URL=http://127.0.0.1:8080 .venv/bin/python -m swarmdeck_slam --host 127.0.0.1 --port 8090

install-slam:
	cd slam && uv venv --allow-existing --python 3.12 .venv && \
	  uv pip install --python .venv/bin/python -e ../adapters/protocol -e ".[dev]"

mock:
	cd adapters/adapter_mock && $(CLEANENV) ../../server/.venv/bin/python mock_adapter.py --robots $(N)

demo:
	@echo "Starting server, mock adapter and UI on host..."
	@$(MAKE) -j3 server mock ui

# ------------------------------------------------------------------------------
# 3. Core Docker Stack (Server + Web UI + SLAM backend)
# ------------------------------------------------------------------------------
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

build-mock:
	$(COMPOSE) --profile mock build mock

up-mock:
	$(COMPOSE) --profile mock up --build -d mock
	@echo "Fleet:            mock adapter (no simulator)"

down-mock:
	$(COMPOSE) --profile mock stop mock
	$(COMPOSE) --profile mock rm -f mock

# ------------------------------------------------------------------------------
# 4. Simulation Bring-Up (ARGoS 3 / Fast-LIVO2 / SLAM / Nav2 / adapter_sim)
# ------------------------------------------------------------------------------
build-argos:
	$(COMPOSE) --profile argos build argos sim fast_livo2

# Modular simulation target accepting SCENARIO, RENDER, and ODOMETRY overrides:
up-sim:
	./scripts/sim-up --scenario $(SCENARIO) --render $(RENDER) --odometry $(ODOMETRY) \
	  --targets $(TARGETS) --explore $(EXPLORE)

up-argos:
	./scripts/sim-up --scenario default --render software --odometry fast_livo2

up-argos-gpu:
	./scripts/sim-up --scenario default --render gpu --odometry fast_livo2

up-argos-dri:
	./scripts/sim-up --scenario default --render dri --odometry fast_livo2

up-argos-dev:
	./scripts/sim-up --scenario 3robot --render dri --odometry drift

up-argos-bistro:
	./scripts/sim-up --scenario bistro --render software --odometry fast_livo2

up-argos-bistro-gpu:
	./scripts/sim-up --scenario bistro --render gpu --odometry fast_livo2

up-argos-bistro-dri:
	./scripts/sim-up --scenario bistro --render dri --odometry fast_livo2

down-argos:
	./scripts/sim-up --down

# Legacy Gazebo simulation path
build-sim:
	$(COMPOSE) --profile gazebo build gazebo

down-sim:
	$(COMPOSE) --profile gazebo stop gazebo
	$(COMPOSE) --profile gazebo rm -f gazebo

# ------------------------------------------------------------------------------
# 5. Physical Fleet Deployment (Operator-Side over SSH)
# ------------------------------------------------------------------------------
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
	@echo "Deploy robot:     make deploy ROBOT=<botman|aslan|scout|spot|asimov|all>"

down-deploy:
	docker compose $(ZENOH_COMPOSE) down --remove-orphans

deploy:
	./scripts/deploy $(if $(ROBOT),$(ROBOT),all) $(DEPLOY_ARGS)

# ------------------------------------------------------------------------------
# 6. Sidecars & AI Extensions (Cortex / Local AI)
# ------------------------------------------------------------------------------
up-agent:
	$(COMPOSE) --profile agent up --build -d agent
	@echo "Cortex:           http://localhost:8085/health"

down-agent:
	$(COMPOSE) --profile agent stop agent
	$(COMPOSE) --profile agent rm -f agent

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

# ------------------------------------------------------------------------------
# 7. Docker Cluster Management & Tests
# ------------------------------------------------------------------------------
docker-up-gpu: up-argos-gpu
	@echo "Backend API:      http://localhost:8080/api/config"

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

docker-test-launch:
	$(COMPOSE) --profile argos build sim
	$(COMPOSE) --profile argos run --rm --no-deps --entrypoint bash sim -lc \
	  'source /opt/ros/jazzy/setup.bash && source /app/swarmdeck_ros/install/setup.bash && \
	   cd /app && python3 -m pytest swarmdeck_ros/src/swarmdeck_bringup/test -q'

# ------------------------------------------------------------------------------
# 8. Testing, Verification & Cleanup
# ------------------------------------------------------------------------------
test:
	$(CLEANENV) server/.venv/bin/pytest -q
	$(MAKE) test-slam
	cd ui && npm run check

test-slam:
	cd slam && $(CLEANENV) .venv/bin/python -m pytest tests/ -q

visual-test:
	python3 tests/integration/run_visual_test.py

visual-test-bistro:
	python3 tests/integration/run_visual_test.py --config configs/4robot_bistro.yaml

RUNTIME_DIR ?= /tmp/swarmdeck
sim:
	mkdir -p $(RUNTIME_DIR)
	cd swarmdeck_ros && . install/setup.bash && \
	  ros2 launch swarmdeck_bringup session.launch.py \
	    sim_backend:=argos launch_argos:=true runtime_dir:=$(RUNTIME_DIR)

tunnel:
	./scripts/tunnel.sh

clean:
	rm -rf ui/node_modules ui/dist server/.venv swarmdeck_ros/{build,install,log} argos/build
	$(COMPOSE) --profile argos --profile gazebo --profile mock --profile agent down --rmi local --volumes --remove-orphans 2>/dev/null || true
