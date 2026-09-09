ROS_DISTRO := jazzy
GBPLANNER3_VERSION := 3.0.0

# Image tag. Deliberately NOT "gbplanner": a gbplanner:jazzy-3.0.0 image already
# exists on this machine from an unrelated build (2026-03-17), and reusing the
# name would move the tag off it.
IMAGE_NAME := gbplanner-ros2
ROOT_DIR := $(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))

# Launch file passed to `ros2 launch gbplanner`. Unlike roslaunch, ros2 launch
# resolves a launch file by exact name inside the package's share/launch dir,
# so subdirectories are flattened at install time and the basename is enough.
# Override on the command line: make run LAUNCH_FILE=ugv_gz_urban_exploration.launch.py
LAUNCH_FILE ?= uav_gz_cave_exploration.launch.py

# ---------------------------------------------------------------------------
# Named scenarios
# ---------------------------------------------------------------------------
# One line per demo from the upstream wiki that this repository can actually
# run: `make run-sim <name>` starts simulator, planner, control interface and
# RViz for it in one container. The name is also the container name suffix, so
# `make ps` and `make stop` see it.
#
# Kept as SCENARIO_<name> variables rather than a table make would have to
# parse, because $(SCENARIO_$(word)) is the whole lookup.
SCENARIOS := uav_cave uav_cargo uav_niosh ugv_niosh ugv_urban anymal_niosh

SCENARIO_uav_cave   := uav_gz_cave_exploration.launch.py
SCENARIO_uav_cargo  := uav_gz_cargo_inspection.launch.py
SCENARIO_uav_niosh  := uav_gz_niosh_exploration.launch.py
SCENARIO_ugv_niosh  := ugv_gz_niosh_exploration.launch.py
SCENARIO_ugv_urban  := ugv_gz_urban_exploration.launch.py
SCENARIO_anymal_niosh := anymal_gz_niosh_exploration.launch.py

DESC_uav_cave   := UAV, DARPA SubT cave (assets)
DESC_uav_cargo  := UAV, cargo tank, actuated camera (assets)
DESC_uav_niosh  := UAV, NIOSH mine (assets)
DESC_ugv_niosh  := UGV, NIOSH mine (assets)
DESC_ugv_urban  := UGV, SubT Urban Circuit, multi-level (assets)
DESC_anymal_niosh := ANYmal, NIOSH mine, ground-robot planning (assets)

# Extra `ros2 launch` arguments for a scenario, e.g. to run one without the
# external assets or without the gz GUI:
#   make run-sim ugv_niosh ARGS="world:=cave_box headless:=true"
ARGS ?=

# Package rebuilt by `make rebuild` / `make run-sim <ns> rebuild`.
REBUILD_PKG ?= gbplanner

# ---------------------------------------------------------------------------
# Multi-robot isolation
# ---------------------------------------------------------------------------
# ROS 2 has no roscore and no ROS_MASTER_URI. Two robots are separated either by
# namespace (same DDS graph, they can see each other - what multi-robot task
# allocation needs) or by ROS_DOMAIN_ID (fully disjoint graphs). Default is one
# domain, distinct namespaces:
#   make run-sim robot0
#   make run-sim robot1
# Set ROS_DOMAIN_ID explicitly only to isolate whole robots from each other.
ROS_DOMAIN_ID ?= 0

# Where the subt_cave_sim tiles live, for the DARPA cave worlds. 3.9 GB with
# git-lfs, so mounted rather than vendored; the ROS 1 checkout already has them.
# Irrelevant when running world:=cave_box.
SUBT_CAVE_SIM ?= $(shell dirname $(ROOT_DIR))/gbplanner_ros/bootstrap/sim/subt_cave_sim

# Whether run-sim brings up RViz alongside the planner. Turn it off for the
# robots you are not watching - one RViz per robot is rarely what you want:
#   make run-sim robot1 RVIZ=false
RVIZ ?= true

# `make run-sim <word> [rebuild]` - read the words after the target as one
# argument plus an optional "rebuild" keyword, then register a no-op rule for
# each of them so make does not treat them as goals of their own. While run-sim
# is the first goal, "rebuild" is one of those stubs, so the real target below
# is defined out - run-sim reaches it through a sub-make instead.
#
# The one word means two things, and which one is decided by looking it up in
# SCENARIOS rather than by a flag. A scenario name starts that scenario;
# anything else is a namespace, which is what run-sim meant before scenarios
# existed and what multi-robot still needs. The lookup is exact, so a mistyped
# scenario does fall through to the namespace reading - but not silently: that
# branch prints which reading it took and lists the scenario names, so a run
# that came up without a simulator says why on its first line.
ifeq (run-sim,$(firstword $(MAKECMDGOALS)))
  RUN_SIM_ARGS := $(wordlist 2,$(words $(MAKECMDGOALS)),$(MAKECMDGOALS))
  RUN_SIM_REBUILD := $(filter rebuild,$(RUN_SIM_ARGS))
  RUN_SIM_WORD := $(firstword $(filter-out rebuild,$(RUN_SIM_ARGS)))
  ifneq ($(RUN_SIM_WORD),)
    ifneq ($(filter $(RUN_SIM_WORD),$(SCENARIOS)),)
      SCENARIO := $(RUN_SIM_WORD)
    else
      NAMESPACE := $(RUN_SIM_WORD)
    endif
  endif
  $(foreach arg,$(RUN_SIM_ARGS),$(eval $(arg):;@:))
endif
SCENARIO ?=
NAMESPACE ?= robot0

export ROOT_DIR
export IMAGE_NAME
export ROS_DISTRO
export GBPLANNER3_VERSION
export LAUNCH_FILE
export NAMESPACE
export REBUILD_PKG
export ROS_DOMAIN_ID
export SUBT_CAVE_SIM
export RVIZ
export SCENARIO
export LAUNCH_ARGS

COMPOSE := docker compose
CONTAINER_NAME := gbplanner_ros2

default: help

build: ## Build the gbplanner container image only
	@echo "Building $(IMAGE_NAME):$(ROS_DISTRO)-$(GBPLANNER3_VERSION) container image..."
	@$(COMPOSE) build base

bootstrap: ## Clone the third-party workspace packages into bootstrap/
	@test -n "$$SSH_AUTH_SOCK" || (echo "SSH_AUTH_SOCK is not set. Start ssh-agent and run ssh-add before make bootstrap." && exit 1)
	@echo "Cloning gbplanner3 workspace sources into bootstrap/..."
	@chmod +x $(ROOT_DIR)/docker/bootstrap.sh
	@$(COMPOSE) run --rm bootstrap

build-ws: ## colcon build the whole workspace over the bootstrapped sources
	@$(COMPOSE) run --rm build-ws

build-all: ## Build everything: image, workspace sources, workspace
	@$(MAKE) build
	@$(MAKE) bootstrap
	@$(MAKE) build-ws

run-dev: ## Interactive dev shell over the workspace
	@test -n "$$SSH_AUTH_SOCK" || (echo "SSH_AUTH_SOCK is not set. Start ssh-agent and run ssh-add before make run-dev." && exit 1)
	@xhost +SI:localuser:root >/dev/null
	@$(COMPOSE) run --rm --name $(CONTAINER_NAME)-dev dev

run: ## Launch gbplanner (LAUNCH_FILE=$(LAUNCH_FILE))
	@test -n "$$SSH_AUTH_SOCK" || (echo "SSH_AUTH_SOCK is not set. Start ssh-agent and run ssh-add before make run." && exit 1)
	@xhost +SI:localuser:root >/dev/null
	@echo "Launching gbplanner $(LAUNCH_FILE)..."
	@$(COMPOSE) run --rm run

# Guarded because `rebuild` doubles as a run-sim keyword, and the no-op stub
# above would otherwise clash with this recipe.
ifneq (run-sim,$(firstword $(MAKECMDGOALS)))
rebuild: ## Rebuild one package (REBUILD_PKG=$(REBUILD_PKG)) in the workspace
	@test -d $(ROOT_DIR)/bootstrap/gbplanner3_ws/install || (echo "workspace not built. Run: make build-all" && exit 1)
	@echo "Rebuilding $(REBUILD_PKG) in gbplanner3_ws..."
	@$(COMPOSE) run --rm --no-deps rebuild
endif

run-sim: ## Run a scenario (make run-sim ugv_niosh) or one namespaced stack (make run-sim robot0) [rebuild]
	@test -n "$$SSH_AUTH_SOCK" || (echo "SSH_AUTH_SOCK is not set. Start ssh-agent and run ssh-add before make run-sim." && exit 1)
ifneq ($(RUN_SIM_REBUILD),)
	@$(MAKE) rebuild
endif
	@xhost +SI:localuser:root >/dev/null
ifneq ($(SCENARIO),)
	@echo "Scenario '$(SCENARIO)': $(DESC_$(SCENARIO))"
	@echo "Launching $(SCENARIO_$(SCENARIO)) $(ARGS) on ROS_DOMAIN_ID=$(ROS_DOMAIN_ID) (rviz:=$(RVIZ))..."
	@# -p isolates each scenario into its own compose project, the same way the
	@# namespace path below does, so two of them do not recreate each other's
	@# containers.
	@LAUNCH_FILE=$(SCENARIO_$(SCENARIO)) LAUNCH_ARGS="$(ARGS)" 	  $(COMPOSE) -p gbplanner-$(SCENARIO) up --abort-on-container-exit --remove-orphans run-scenario
else
	@echo "'$(NAMESPACE)' is not a scenario name, so it is read as a namespace."
	@echo "  scenarios: $(SCENARIOS)   (make scenarios for what each one is)"
	@echo "Launching the gbplanner stack for namespace '$(NAMESPACE)' on ROS_DOMAIN_ID=$(ROS_DOMAIN_ID) (use_sim_time:=true, rviz:=$(RVIZ))..."
	@echo "  This half expects a simulator someone else started; a scenario brings its own."
	@$(COMPOSE) -p gbplanner-$(NAMESPACE) up --abort-on-container-exit --remove-orphans run-sim
endif

scenarios: ## List the named scenarios run-sim accepts
	@echo "make run-sim <scenario> [rebuild] [ARGS=\"...\"]"
	@echo ""
	@$(foreach s,$(SCENARIOS),printf '  \033[36m%-12s\033[0m %s\n' '$(s)' '$(DESC_$(s))';)
	@echo ""
	@echo "  (assets) = needs the subt_cave_sim model set, mounted from SUBT_CAVE_SIM"
	@echo "             from $(SUBT_CAVE_SIM)"
	@echo "  Most scenarios also accept world:=cave_box, which needs no assets:"
	@echo "    make run-sim ugv_niosh ARGS=\"world:=cave_box\""
	@echo ""
	@echo "Any other word is read as a namespace instead, for multi-robot:"
	@echo "  make run-sim robot0 / make run-sim robot1 RVIZ=false"

stop-sim: ## Stop the stack for one scenario or namespace: make stop-sim SCENARIO=ugv_niosh
	@$(COMPOSE) -p gbplanner-$(if $(SCENARIO),$(SCENARIO),$(NAMESPACE)) down

enter-dev: ## Attach a shell to the running dev container
	@echo "Entering $(CONTAINER_NAME)-dev container..."
	@docker exec -it $(CONTAINER_NAME)-dev bash

stop: ## Stop and remove any running gbplanner containers
	@# `compose down` only removes the services of this project. Containers
	@# started by `compose run` are one-off, get generated names, and survive it -
	@# and because every service uses network_mode: host with the same
	@# ROS_DOMAIN_ID, a survivor keeps publishing on the same topics as the next
	@# run. RViz then shows two alternating streams, which looks like a flickering
	@# display rather than a leftover process.
	@$(COMPOSE) down --remove-orphans
	@docker ps -a --filter "name=$(CONTAINER_NAME)-" --format '{{.ID}}' | xargs -r docker rm -f >/dev/null
	@echo "Remaining $(CONTAINER_NAME) containers: $$(docker ps -a --filter "name=$(CONTAINER_NAME)-" --format '{{.ID}}' | wc -l)"
	@# Every service runs with ipc: host, so Fast DDS's shared-memory segments
	@# live in the host's /dev/shm. A container that is killed rather than shut
	@# down leaves its segments behind, and once a few hundred have piled up ROS
	@# 2 discovery inside a *new* container stops working entirely: `ros2 topic
	@# list` comes back empty while gz is perfectly healthy, which reads as a
	@# simulator that failed to start. Counted here rather than deleted, because
	@# any ROS 2 process outside this repo owns segments in the same directory.
	@n=$$(ls /dev/shm 2>/dev/null | grep -c '^fastrtps_' || true); 	if [ "$$n" -gt 100 ]; then 	  echo "Stale Fast DDS segments in /dev/shm: $$n. Run 'make clean-shm' with no ROS 2 process running."; 	fi

clean-shm: ## Remove leaked Fast DDS shared-memory segments (stop everything first)
	@test -z "$$(pgrep -f '[g]bplanner_node|[p]ci_general_ros_node|[p]arameter_bridge|[g]z sim')" 	  || (echo "ROS 2 or gz processes are still running; 'make stop' first." && exit 1)
	@# The segments are owned by root because they were created inside the
	@# containers, so the removal happens inside one too.
	@$(COMPOSE) run --rm --no-deps dev bash -c 	  'before=$$(ls /dev/shm | grep -c "^fastrtps_"); rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*; 	   echo "removed $$before segments, $$(ls /dev/shm | grep -c "^fastrtps_") left"'

ps: ## Show what this repo currently has running
	@echo "Containers:"
	@docker ps --filter "name=$(CONTAINER_NAME)" --format '  {{.Names}}	{{.Status}}' || true
	@echo "ROS processes on the host (a live one competes on the DDS domain):"
	@pgrep -af '[g]bplanner_node|[p]ci_general_ros_node|[g]z sim' | cut -c1-100 | sed 's/^/  /' || echo "  none"

clean: stop ## Stop containers and remove the built image
	@echo "Cleaning up $(IMAGE_NAME):$(ROS_DISTRO)-$(GBPLANNER3_VERSION) container image..."
	@docker rmi $(IMAGE_NAME):$(ROS_DISTRO)-$(GBPLANNER3_VERSION) || true

help: ## Show this help message
	@echo "Usage: make [target]"
	@echo ""
	@echo "Targets:"
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | sort | awk 'BEGIN {FS = ":.*?## "} {printf "  \033[36m%-20s\033[0m %s\n", $$1, $$2}'
	@echo ""
	@echo "Scenarios for run-sim:"
	@$(foreach s,$(SCENARIOS),printf '  \033[36m%-20s\033[0m %s\n' '$(s)' '$(DESC_$(s))';)
	@echo "  (make scenarios for the assets each one needs)"

.PHONY: build bootstrap build-ws build-all rebuild run-dev run run-sim scenarios stop-sim enter-dev stop clean-shm ps clean help
