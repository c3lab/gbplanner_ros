ROS_DISTRO := noetic
GBPLANNER3_VERSION := 3.0.0
ROOT_DIR := $(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))

# Launch file passed to `roslaunch gbplanner`. File name only, no directory:
# roslaunch matches launch files by base name (roslib.packages.find_resource
# walks the package), so a path like "uav/gz/x.launch" never resolves.
# Override on the command line: make run LAUNCH_FILE=ugv_gzc_urban_exploration.launch
LAUNCH_FILE ?= uav_gz_cave_exploration.launch

# Package rebuilt by `make rebuild` / `make run-sim <ns> rebuild`.
REBUILD_PKG ?= gbplanner

# ---------------------------------------------------------------------------
# Simulation stack (`make run-sim <namespace>`)
# ---------------------------------------------------------------------------
BRIDGE_DIR := $(ROOT_DIR)/bridge

# Read the tag straight out of the submodule's Makefile so the two never drift
# apart after a submodule bump.
BRIDGE_VERSION ?= $(shell sed -n 's/^VERSION[[:space:]]*:*=[[:space:]]*//p' $(BRIDGE_DIR)/Makefile 2>/dev/null)

# Master both containers of one run-sim pair attach to. Each robot gets its own
# roscore, so running several needs a distinct port per robot - the planner's
# roslaunch starts a master on whatever port this names, and the bridge binds
# to the same one:
#   make run-sim robot0 ROS_MASTER_URI=http://localhost:11311
#   make run-sim robot1 ROS_MASTER_URI=http://localhost:11312
# Both containers run with network_mode: host, so "localhost" is the host's.
ROS_MASTER_URI ?= http://localhost:11311

# Whether run-sim brings up RViz alongside the planner. Turn it off for the
# robots you are not watching - one RViz per robot is rarely what you want:
#   make run-sim robot1 RVIZ=false
RVIZ ?= true

# `make run-sim robot0 [rebuild]` - read the words after the target as the
# namespace plus an optional "rebuild" keyword, then register a no-op rule for
# each of them so make does not treat them as goals of their own. While run-sim
# is the first goal, "rebuild" is one of those stubs, so the real target below
# is defined out - run-sim reaches it through a sub-make instead.
ifeq (run-sim,$(firstword $(MAKECMDGOALS)))
  RUN_SIM_ARGS := $(wordlist 2,$(words $(MAKECMDGOALS)),$(MAKECMDGOALS))
  RUN_SIM_REBUILD := $(filter rebuild,$(RUN_SIM_ARGS))
  RUN_SIM_NS := $(firstword $(filter-out rebuild,$(RUN_SIM_ARGS)))
  ifneq ($(RUN_SIM_NS),)
    NAMESPACE := $(RUN_SIM_NS)
  endif
  $(foreach arg,$(RUN_SIM_ARGS),$(eval $(arg):;@:))
endif
NAMESPACE ?= robot0

export ROOT_DIR
export ROS_DISTRO
export GBPLANNER3_VERSION
export LAUNCH_FILE
export NAMESPACE
export BRIDGE_VERSION
export REBUILD_PKG
export ROS_MASTER_URI
export RVIZ

COMPOSE := docker compose
CONTAINER_NAME := gbplanner_ros1

default: help

build: ## Build the gbplanner container image only
	@echo "Building gbplanner:$(ROS_DISTRO)-$(GBPLANNER3_VERSION) container image..."
	@$(COMPOSE) build base

build-bridge: ## Build the ros1_bridge image only (needs planner_msgs)
	@test -f $(BRIDGE_DIR)/Makefile || (echo "bridge/ is empty. Run: git submodule update --init --recursive" && exit 1)
	@test -d $(ROOT_DIR)/planner_msgs || (echo "planner_msgs/ is missing. Run: make bootstrap" && exit 1)
	@# ros1_bridge generates its conversion factories at build time, so the ROS 1
	@# planner_msgs sources have to be inside the bridge build context before the
	@# image is built. Docker will not follow a symlink out of the context, so copy.
	@echo "Staging planner_msgs into bridge/custom_msgs/custom_msgs_ros1_ws/src/..."
	@rm -rf $(BRIDGE_DIR)/custom_msgs/custom_msgs_ros1_ws/src/planner_msgs
	@cp -r $(ROOT_DIR)/planner_msgs $(BRIDGE_DIR)/custom_msgs/custom_msgs_ros1_ws/src/planner_msgs
	@$(MAKE) -C $(BRIDGE_DIR) build

build-all: ## Build everything: gbplanner image, workspace bootstrap, bridge image
	@$(MAKE) build
	@$(MAKE) bootstrap
	@$(MAKE) build-bridge

bootstrap: ## One-shot: clone and build the full workspace into bootstrap/
	@echo "Bootstrapping gbplanner:$(ROS_DISTRO)-$(GBPLANNER3_VERSION) workspace..."
	@chmod +x $(ROOT_DIR)/docker/bootstrap.sh
	@$(COMPOSE) run --rm bootstrap

run-dev: ## Interactive dev shell over the bootstrapped workspace
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
rebuild: ## Rebuild one package (REBUILD_PKG=$(REBUILD_PKG)) in the bootstrapped workspace
	@test -f $(ROOT_DIR)/bootstrap/install/setup_all.bash || (echo "workspace not bootstrapped. Run: make bootstrap" && exit 1)
	@echo "Rebuilding $(REBUILD_PKG) in gbplanner3_ws..."
	@$(COMPOSE) run --rm --no-deps rebuild
endif

run-sim: ## Launch gbplanner + ros1_bridge for one namespace: make run-sim robot0 [rebuild]
	@test -n "$$SSH_AUTH_SOCK" || (echo "SSH_AUTH_SOCK is not set. Start ssh-agent and run ssh-add before make run-sim." && exit 1)
	@test -n "$(BRIDGE_VERSION)" || (echo "BRIDGE_VERSION is empty - bridge/ is not checked out. Run: git submodule update --init --recursive" && exit 1)
ifneq ($(RUN_SIM_REBUILD),)
	@$(MAKE) rebuild
endif
	@xhost +SI:localuser:root >/dev/null
	@echo "Launching gbplanner + bridge for namespace '$(NAMESPACE)' on $(ROS_MASTER_URI) (use_sim_time:=true, rviz:=$(RVIZ))..."
	@# -p isolates each robot into its own compose project, so a second
	@# `make run-sim robot1` adds containers instead of recreating robot0's.
	@$(COMPOSE) -p gbplanner-$(NAMESPACE) up --abort-on-container-exit --remove-orphans run-sim bridge

stop-sim: ## Stop the sim stack for one namespace: make stop-sim NAMESPACE=robot0
	@$(COMPOSE) -p gbplanner-$(NAMESPACE) down

enter-dev: ## Attach a shell to the running dev container
	@echo "Entering $(CONTAINER_NAME)-dev container..."
	@docker exec -it $(CONTAINER_NAME)-dev bash

stop: ## Stop and remove any running gbplanner containers
	@$(COMPOSE) down

clean: stop ## Stop containers and remove the built image
	@echo "Cleaning up gbplanner:$(ROS_DISTRO)-$(GBPLANNER3_VERSION) container image..."
	@docker rmi gbplanner:$(ROS_DISTRO)-$(GBPLANNER3_VERSION) || true

help: ## Show this help message
	@echo "Usage: make [target]"
	@echo ""
	@echo "Targets:"
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | sort | awk 'BEGIN {FS = ":.*?## "} {printf "  \033[36m%-20s\033[0m %s\n", $$1, $$2}'

.PHONY: build build-bridge build-all bootstrap rebuild run-dev run run-sim stop-sim enter-dev stop clean help
