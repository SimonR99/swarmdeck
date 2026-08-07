.PHONY: help install ui ui-build server mock sim test clean up-scout \
        build-server up-server down-server \
        build-sim up-sim down-sim \
        build-mock up-mock down-mock \
        build-deploy up-deploy down-deploy \
        docker-up-gpu docker-up-cslam docker-down docker-logs \
        docker-ps docker-test docker-test-launch

help:
	@echo "SwarmDeck"
	@echo "  make install         install ui + server dependencies (local)"
	@echo "  make ui              run frontend dev server (http://localhost:5173)"
	@echo "  make ui-build        production build of the frontend"
	@echo "  make server          run backend (http://localhost:8080)"
	@echo "  make mock            run mock adapter (N=4 robots, no ROS needed)"
	@echo "  make demo            server + mock + ui, all at once"
	@echo "  make [build|up|down]-server  Docker: backend + UI only"
	@echo "  make [build|up|down]-sim     Docker: Gazebo/SLAM/Nav2/adapter_sim (needs server up)"
	@echo "  make [build|up|down]-mock    Docker: synthetic mock fleet (needs server up)"
	@echo "  make [build|up|down]-deploy  REAL FLEET: server + UI + Zenoh router (see hardware-bringup.md)"
	@echo "  make up-scout        start Scout ROS/SLAM, camera, adapter and media on ssh host 'scout'"
	@echo "  make docker-up-gpu   full stack (server+ui+sim), Gazebo rendering on an NVIDIA GPU"
	@echo "  make docker-up-cslam as docker-up-gpu, plus Swarm-SLAM collaborative SLAM"
	@echo "  make docker-down     stop everything (server, ui, sim, mock)"
	@echo "  make docker-logs     follow Docker logs"
	@echo "  make docker-test     run backend tests inside Docker"
	@echo "  make docker-test-launch  build every LaunchDescription in the ROS image"
	@echo "  make sim             launch Gazebo simulation (host ROS)"
	@echo "  make test            run all tests (local venv)"

install:
	cd ui && npm install
	cd server && python3 -m venv .venv && .venv/bin/pip install -q -e ".[dev]"

ui:
	cd ui && npm run dev

ui-build:
	cd ui && npm run build

# ROS must NOT be on the path — the backend is ROS-free by design.
CLEANENV = env -u PYTHONPATH -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH

server:
	cd server && $(CLEANENV) .venv/bin/python -m swarmdeck_server

N ?= 4
mock:
	cd adapters/adapter_mock && $(CLEANENV) ../../server/.venv/bin/python mock_adapter.py --robots $(N)

demo:
	@echo "Starting server, mock adapter and ui..."
	@$(MAKE) -j3 server mock ui

sim:
	cd swarmdeck_ros && . install/setup.bash && ros2 launch swarmdeck_bringup session.launch.py

test:
	cd server && $(CLEANENV) .venv/bin/python -m pytest tests -q
	$(CLEANENV) server/.venv/bin/python -m pytest \
	  swarmdeck_ros/src/swarmdeck_sim/test \
	  swarmdeck_ros/src/swarmdeck_bringup/test \
	  adapters/test -q
	cd ui && npm run check

# --- Scout Mini hardware: start the host ROS graph and robot-side containers.
# Override SCOUT_HOST, SCOUT_REPO, ROBOT_IP, BACKEND_HOST or OUSTER_HOST_NAME as needed.
up-scout:
	./scripts/scout-up

# --- server + UI: the always-on core. Everything else depends on it being up.
build-server:
	docker compose build server ui

up-server:
	docker compose up --build -d server ui
	@echo "SwarmDeck UI:     http://localhost:5173"
	@echo "Backend API:      http://localhost:8080/api/config"

down-server:
	docker compose stop server ui
	docker compose rm -f server ui

# --- simulated fleet: Gazebo + SLAM/Nav2 + adapter_sim. `depends_on: server` in
# docker-compose.yml means this brings the server up too if it isn't already.
build-sim:
	docker compose --profile gazebo build gazebo

up-sim:
	docker compose --profile gazebo up --build -d gazebo
	@echo "Fleet:            Gazebo + adapter_sim (allow ~45s for robots to appear)"

down-sim:
	docker compose --profile gazebo stop gazebo
	docker compose --profile gazebo rm -f gazebo

# --- synthetic fleet: no Gazebo/ROS. Also depends_on: server.
build-mock:
	docker compose --profile mock build mock

up-mock:
	docker compose --profile mock up --build -d mock
	@echo "Fleet:            mock adapter (no Gazebo)"

down-mock:
	docker compose --profile mock stop mock
	docker compose --profile mock rm -f mock

# --- real fleet deployment: server + UI + a Zenoh router, so robots on other
# machines can reach both the backend and each other's ROS graph. No Gazebo, no
# cslam, no mock — this is the actual bring-up target. UNVERIFIED against
# physical robots (see docker-compose.zenoh.yml); adapters run per-robot,
# outside Docker, per docs/hardware-bringup.md.
ZENOH_COMPOSE = -f docker-compose.yml -f docker-compose.zenoh.yml

build-deploy:
	docker compose $(ZENOH_COMPOSE) build server ui

up-deploy:
	docker compose $(ZENOH_COMPOSE) up --build -d server ui zenoh-router
	@echo "SwarmDeck UI:     http://localhost:5173"
	@echo "Backend API:      http://localhost:8080/api/config"
	@echo "Zenoh router:     tcp/<this-host>:7447"
	@echo "On each robot:    RMW_IMPLEMENTATION=rmw_zenoh_cpp, session config -> tcp/<this-host>:7447"
	@echo "                  then: adapter_ros2.py --robot-id <id> --config <cfg> --host <this-host>"

down-deploy:
	docker compose $(ZENOH_COMPOSE) stop server ui zenoh-router
	docker compose $(ZENOH_COMPOSE) rm -f server ui zenoh-router

# Full stack (server+ui+sim) with Gazebo rendering on the GPU. Software rendering caps the sim
# at ~0.58x real time, which is the budget every lidar-fidelity increase has to
# come out of; see docker-compose.gpu.yml.
GPU_COMPOSE = -f docker-compose.yml -f docker-compose.gpu.yml
docker-up-gpu:
	docker compose $(GPU_COMPOSE) --profile gazebo up --build -d
	@echo "SwarmDeck UI:     http://localhost:5173"
	@echo "Backend API:      http://localhost:8080/api/config"
	@echo "Fleet:            Gazebo (GPU) + adapter_sim (allow ~45s for robots to appear)"

# Collaborative SLAM: the fleet plus Swarm-SLAM. Needs the 3D lidar profile and
# the RTAB-Map backend, since cslam's motion prior is the lidar odometry.
CSLAM_COMPOSE = -f docker-compose.yml -f docker-compose.gpu.yml -f docker-compose.cslam.yml
docker-up-cslam:
	SWARMDECK_CONFIG=/app/study/4robot_3d.yaml SLAM_BACKEND=rtabmap \
	  docker compose $(CSLAM_COMPOSE) --profile gazebo up --build -d
	@echo "SwarmDeck UI:     http://localhost:5173"
	@echo "Fleet:            Gazebo (GPU) + RTAB-Map + Swarm-SLAM"

docker-down:
	docker compose --profile gazebo --profile mock down

docker-logs:
	docker compose --profile gazebo --profile mock logs -f

docker-ps:
	docker compose --profile gazebo --profile mock ps

docker-test:
	docker compose build server
	docker compose run --rm --no-deps server python -m pytest /app/server/tests -q

# Launch files are plain Python that nothing executes until Gazebo starts, so an
# undefined name in one costs a full stack startup to find. This builds every
# LaunchDescription in the ROS image, which takes under a second.
docker-test-launch:
	docker compose build gazebo
	docker compose run --rm --no-deps --entrypoint bash gazebo -lc \
	  'source /opt/ros/jazzy/setup.bash && source /app/swarmdeck_ros/install/setup.bash && \
	   cd /app && python3 -m pytest swarmdeck_ros/src/swarmdeck_bringup/test -q'

clean:
	rm -rf ui/node_modules ui/dist server/.venv swarmdeck_ros/{build,install,log}
	docker compose --profile gazebo --profile mock down --rmi local --volumes --remove-orphans 2>/dev/null || true
