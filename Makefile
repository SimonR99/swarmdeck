.PHONY: help install ui ui-build server mock sim test clean \
        docker-build docker-up docker-up-mock docker-down docker-logs docker-ps docker-test

help:
	@echo "SwarmDeck"
	@echo "  make install         install ui + server dependencies (local)"
	@echo "  make ui              run frontend dev server (http://localhost:5173)"
	@echo "  make ui-build        production build of the frontend"
	@echo "  make server          run backend (http://localhost:8080)"
	@echo "  make mock            run mock adapter (N=4 robots, no ROS needed)"
	@echo "  make demo            server + mock + ui, all at once"
	@echo "  make docker-up       Docker: server + UI + Gazebo/SLAM/Nav2/adapter_sim"
	@echo "  make docker-up-mock  Docker: server + UI + synthetic mock fleet"
	@echo "  make docker-down     stop Docker stack"
	@echo "  make docker-logs     follow Docker logs"
	@echo "  make docker-test     run backend tests inside Docker"
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
	$(CLEANENV) server/.venv/bin/python -m pytest swarmdeck_ros/src/swarmdeck_sim/test -q
	cd ui && npm run check

docker-build:
	docker compose --profile gazebo build

docker-up:
	docker compose --profile gazebo up --build -d
	@echo "SwarmDeck UI:     http://localhost:5173"
	@echo "Backend API:      http://localhost:8080/api/config"
	@echo "Fleet:            Gazebo + adapter_sim (allow ~45s for robots to appear)"

docker-up-mock:
	docker compose --profile mock up --build -d
	@echo "SwarmDeck UI:     http://localhost:5173"
	@echo "Fleet:            mock adapter (no Gazebo)"

docker-down:
	docker compose --profile gazebo --profile mock down

docker-logs:
	docker compose --profile gazebo --profile mock logs -f

docker-ps:
	docker compose --profile gazebo --profile mock ps

docker-test:
	docker compose build server
	docker compose run --rm --no-deps server python -m pytest /app/server/tests -q

clean:
	rm -rf ui/node_modules ui/dist server/.venv swarmdeck_ros/{build,install,log}
	docker compose --profile gazebo --profile mock down --rmi local --volumes --remove-orphans 2>/dev/null || true
