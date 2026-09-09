# SwarmDeck: local development, ARGoS simulation, and fleet operations.
.DEFAULT_GOAL := help

export COMPOSE_PROJECT ?= swarmdeck
SCENARIO ?= default
RENDER ?= software
ODOMETRY ?= fast_livo2
TARGETS ?= 10
EXPLORE ?= 0
SIM_ARGS ?=
ROBOT ?= all
DEPLOY_ARGS ?=
N ?= 4
LOCAL_MODEL ?= qwen3.5:9b-q4_K_M
VISUAL_CONFIG ?= configs/4robot.yaml

export SSH_AUTH_SOCK_REAL ?= $(shell readlink -f $${SSH_AUTH_SOCK:-/dev/null} 2>/dev/null || echo /dev/null)
COMPOSE ?= docker compose -p $(COMPOSE_PROJECT) -f deploy/compose/docker-compose.yml
DEPLOY_COMPOSE = $(COMPOSE) -f deploy/compose/docker-compose.zenoh.yml
SIM = ./scripts/sim-up --scenario "$(SCENARIO)" --render "$(RENDER)" --odometry "$(ODOMETRY)" --targets "$(TARGETS)" --explore "$(EXPLORE)"
CLEANENV = env -u PYTHONPATH -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH

.PHONY: help install install-ui install-server install-agent install-slam \
        ui ui-build server slam mock demo build-server up-server down-server \
        build-mock up-mock down-mock build-sim up-sim down-sim \
        build-deploy up-deploy down-deploy deploy up-agent down-agent \
        local-ai-up local-ai-pull local-ai-shadow local-ai-eval local-ai-down \
        docker-down docker-logs docker-ps docker-purge docker-test docker-test-launch \
        test test-server test-agent test-ui test-slam visual-test clean

help: ## Show commands and common options
	@awk 'BEGIN {FS = ":.*## "} /^[a-z][a-z0-9-]*:.*## / {printf "  make %-20s %s\n", $$1, $$2}' $(MAKEFILE_LIST)
	@printf '\nSimulation: SCENARIO=default|bistro|3robot|path.yaml RENDER=software|gpu|dri\n'
	@printf '            ODOMETRY=fast_livo2|drift TARGETS=10 EXPLORE=0\n'
	@printf '            SIM_ARGS="--dry-run" or "--no-build"\n'
	@printf 'Deployment: ROBOT=botman DEPLOY_ARGS="--dry-run"\n'
	@printf 'Other:      N=4 LOCAL_MODEL=<model> VISUAL_CONFIG=configs/4robot_bistro.yaml\n'

# Local development
install: install-ui install-server ## Install UI and server dependencies

install-ui: ## Install locked UI dependencies
	cd ui && npm ci

install-server: ## Install server and shared protocol in server/.venv
	cd server && $(CLEANENV) python3 -m venv .venv && $(CLEANENV) .venv/bin/pip install -q -e "../adapters/protocol" -e ".[dev]"

install-agent: ## Install Cortex and test dependencies in agent/.venv
	cd agent && $(CLEANENV) python3 -m venv .venv && $(CLEANENV) .venv/bin/pip install -q -e ".[dev]"

install-slam: ## Install SLAM with Python 3.12 (requires uv)
	cd slam && uv venv --allow-existing --python 3.12 .venv && \
	  uv pip install --python .venv/bin/python -e ../adapters/protocol -e ".[dev]"

ui: ## Run the UI dev server on :5173
	cd ui && npm run dev

ui-build: ## Build the production UI
	cd ui && npm run build

server: ## Run the fleet server on :8080
	cd server && $(CLEANENV) SWARMDECK_SLAM_URL=http://127.0.0.1:8090 .venv/bin/python -m swarmdeck_server

slam: ## Run collaborative SLAM on :8090
	cd slam && $(CLEANENV) SWARMDECK_SERVER_URL=http://127.0.0.1:8080 .venv/bin/python -m swarmdeck_slam --host 127.0.0.1 --port 8090

mock: ## Run a synthetic fleet (N=4)
	cd adapters/adapter_mock && $(CLEANENV) ../../server/.venv/bin/python mock_adapter.py --robots $(N)

demo: ## Run local server, mock fleet, and UI together
	@$(MAKE) -j3 server mock ui

# Docker services
build-server: ## Build server, UI, and SLAM images without starting them
	$(COMPOSE) build server ui slam

up-server: ## Start server, UI, and SLAM
	$(COMPOSE) up --build -d server ui slam

down-server: ## Stop and remove server, UI, and SLAM containers
	$(COMPOSE) stop server ui slam
	$(COMPOSE) rm -f server ui slam

build-mock: ## Build the synthetic adapter image
	$(COMPOSE) --profile mock build mock

up-mock: ## Start the synthetic adapter and its dependencies
	$(COMPOSE) --profile mock up --build -d mock

down-mock: ## Stop and remove the synthetic adapter
	$(COMPOSE) --profile mock stop mock
	$(COMPOSE) --profile mock rm -f mock

# Simulation always means ARGoS. Legacy Gazebo commands live in the docs.
build-sim: ## Build the selected ARGoS stack without starting it
	$(SIM) --build-only $(SIM_ARGS)

up-sim: ## Start ARGoS (SCENARIO, RENDER, ODOMETRY, SIM_ARGS)
	$(SIM) $(SIM_ARGS)

down-sim: ## Stop ARGoS services and remove their generated runtime volume
	$(SIM) --down

# Physical fleet deployment
build-deploy: ## Build operator server and UI images
	$(DEPLOY_COMPOSE) build server ui

up-deploy: ## Start operator services and the Zenoh router
	SWARMDECK_CONFIG=/app/configs/hardware_fleet.yaml \
	  SWARMDECK_SLAM_REGISTRATION_MODE=graph \
	  SWARMDECK_SLAM_ANCHOR_ROBOT=aslan_0 \
	  SWARMDECK_SLAM_CAPTURE_DIR=/app/sessions/captures/hardware-live \
	  SWARMDECK_SLAM_RESTORE_CAPTURE=true \
	  $(DEPLOY_COMPOSE) up --build -d server ui mediamtx zenoh-router slam

down-deploy: ## Stop the operator deployment stack
	$(DEPLOY_COMPOSE) down --remove-orphans

deploy: ## Deploy over SSH (ROBOT=all or a profile, DEPLOY_ARGS)
	./scripts/deploy $(ROBOT) $(DEPLOY_ARGS)

# Optional Cortex and local model evaluation
up-agent: ## Start the opt-in Cortex service
	$(COMPOSE) --profile agent up --build -d agent

down-agent: ## Stop and remove Cortex
	$(COMPOSE) --profile agent stop agent
	$(COMPOSE) --profile agent rm -f agent

local-ai-up: ## Start the optional Ollama service
	$(COMPOSE) --profile local-ai up -d ollama

local-ai-pull: local-ai-up ## Download LOCAL_MODEL into Ollama
	$(COMPOSE) --profile local-ai exec ollama ollama pull $(LOCAL_MODEL)

local-ai-shadow: local-ai-pull ## Start Cortex with a tool-free Ollama shadow planner
	$(COMPOSE) --profile agent build agent
	CORTEX_SHADOW_PLANNER=true \
	  CORTEX_PLANNER_PROVIDER=ollama \
	  CORTEX_PLANNER_MODEL=$(LOCAL_MODEL) \
	  $(COMPOSE) --profile agent --profile local-ai up -d --no-deps agent

local-ai-eval: local-ai-shadow ## Run planner evaluations against LOCAL_MODEL
	$(COMPOSE) --profile agent --profile local-ai exec -T agent \
	  python /app/agent/evals/run_planner_eval.py --model $(LOCAL_MODEL)

local-ai-down: ## Stop Ollama
	$(COMPOSE) --profile local-ai stop ollama

# Stack inspection and teardown
# Include MGG so teardown also covers the sidecar started by up-sim.
docker-down: ## Stop the project stack, keeping volumes and images
	$(COMPOSE) -f deploy/compose/docker-compose.mgg.yml --profile '*' down --remove-orphans

docker-logs: ## Follow project container logs
	$(COMPOSE) -f deploy/compose/docker-compose.mgg.yml --profile '*' logs -f

docker-ps: ## Show project containers
	$(COMPOSE) -f deploy/compose/docker-compose.mgg.yml --profile '*' ps

docker-purge: ## DESTRUCTIVE: remove project containers, volumes, and local images
	$(COMPOSE) -f deploy/compose/docker-compose.mgg.yml --profile '*' down --rmi local --volumes --remove-orphans

# Validation
docker-test: ## Run server tests inside its image
	$(COMPOSE) build server
	$(COMPOSE) run --rm --no-deps server python -m pytest /app/server/tests -q

docker-test-launch: ## Validate ROS launch files in the simulation image
	$(COMPOSE) --profile argos build sim
	$(COMPOSE) --profile argos run --rm --no-deps --entrypoint bash sim -lc \
	  'source /opt/ros/jazzy/setup.bash && source /app/swarmdeck_ros/install/setup.bash && \
	   cd /app && python3 -m pytest swarmdeck_ros/src/swarmdeck_bringup/test -q'

test: ## Run server/adapter, Cortex, SLAM, and UI checks
	$(MAKE) test-server
	$(MAKE) test-agent
	$(MAKE) test-slam
	$(MAKE) test-ui

test-server: ## Run ROS-free Python checks; report unavailable asset/ROS checks
	$(CLEANENV) server/.venv/bin/python -m pytest -q -rs

test-agent: ## Run Cortex tests without live providers or robots
	cd agent && $(CLEANENV) .venv/bin/python -m pytest tests/ -q

test-ui: ## Check Svelte types and map regressions
	cd ui && npm run check
	cd ui && npm run test:map3d

test-slam: ## Run collaborative SLAM tests in its separate environment
	cd slam && $(CLEANENV) .venv/bin/python -m pytest tests/ -q

visual-test: ## Capture ARGoS sensors using VISUAL_CONFIG (requires host simulator)
	python3 tests/integration/run_visual_test.py --config "$(VISUAL_CONFIG)"

clean: ## Remove local build outputs and dependency environments; keep Docker data
	rm -rf ui/node_modules ui/dist server/.venv agent/.venv slam/.venv \
	  swarmdeck_ros/build swarmdeck_ros/install swarmdeck_ros/log argos/build
