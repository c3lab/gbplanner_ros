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
export IMAGE_NAME
export ROS_DISTRO
export GBPLANNER3_VERSION
export LAUNCH_FILE
export NAMESPACE
export REBUILD_PKG
export ROS_DOMAIN_ID
export RVIZ

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

run-sim: ## Launch gbplanner + gz sim for one namespace: make run-sim robot0 [rebuild]
	@test -n "$$SSH_AUTH_SOCK" || (echo "SSH_AUTH_SOCK is not set. Start ssh-agent and run ssh-add before make run-sim." && exit 1)
ifneq ($(RUN_SIM_REBUILD),)
	@$(MAKE) rebuild
endif
	@xhost +SI:localuser:root >/dev/null
	@echo "Launching gbplanner + gz sim for namespace '$(NAMESPACE)' on ROS_DOMAIN_ID=$(ROS_DOMAIN_ID) (use_sim_time:=true, rviz:=$(RVIZ))..."
	@# -p isolates each robot into its own compose project, so a second
	@# `make run-sim robot1` adds containers instead of recreating robot0's.
	@$(COMPOSE) -p gbplanner-$(NAMESPACE) up --abort-on-container-exit --remove-orphans run-sim

stop-sim: ## Stop the sim stack for one namespace: make stop-sim NAMESPACE=robot0
	@$(COMPOSE) -p gbplanner-$(NAMESPACE) down

enter-dev: ## Attach a shell to the running dev container
	@echo "Entering $(CONTAINER_NAME)-dev container..."
	@docker exec -it $(CONTAINER_NAME)-dev bash

stop: ## Stop and remove any running gbplanner containers
	@$(COMPOSE) down

clean: stop ## Stop containers and remove the built image
	@echo "Cleaning up $(IMAGE_NAME):$(ROS_DISTRO)-$(GBPLANNER3_VERSION) container image..."
	@docker rmi $(IMAGE_NAME):$(ROS_DISTRO)-$(GBPLANNER3_VERSION) || true

help: ## Show this help message
	@echo "Usage: make [target]"
	@echo ""
	@echo "Targets:"
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | sort | awk 'BEGIN {FS = ":.*?## "} {printf "  \033[36m%-20s\033[0m %s\n", $$1, $$2}'

.PHONY: build bootstrap build-ws build-all rebuild run-dev run run-sim stop-sim enter-dev stop clean help
