.PHONY: help install ui ui-build server mock sim test clean

help:
	@echo "SwarmDeck"
	@echo "  make install    install ui + server dependencies"
	@echo "  make ui         run frontend dev server (http://localhost:5173)"
	@echo "  make ui-build   production build of the frontend"
	@echo "  make server     run backend (http://localhost:8080)"
	@echo "  make mock       run mock adapter (N=4 robots, no ROS needed)"
	@echo "  make demo       server + mock + ui, all at once"
	@echo "  make sim        launch Gazebo simulation"
	@echo "  make test       run all tests"

install:
	cd ui && npm install
	cd server && python3 -m venv .venv && .venv/bin/pip install -q -e ".[dev]" websockets

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
	cd ui && npm run check

clean:
	rm -rf ui/node_modules ui/dist server/.venv swarmdeck_ros/{build,install,log}
