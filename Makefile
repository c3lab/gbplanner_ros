ROS_DISTRO := noetic
GBPLANNER3_VERSION := 3.0.0
ROOT_DIR := $(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))

# Launch file passed to `roslaunch gbplanner`. File name only, no directory:
# roslaunch matches launch files by base name (roslib.packages.find_resource
# walks the package), so a path like "uav/gz/x.launch" never resolves.
# Override on the command line: make run LAUNCH_FILE=ugv_gzc_urban_exploration.launch
LAUNCH_FILE ?= uav_gz_cave_exploration.launch

# Extra "name:=value" pairs appended to the roslaunch line by the `run` service.
LAUNCH_ARGS ?=

# TF frame prefix this robot gets in the fleet graph (make run-robot /
# run-robot-bridge). Separate from NAMESPACE-as-ROS-namespace: the planner keeps
# answering on /anymal/... locally while its frames become <prefix>/... globally.
TF_PREFIX ?= $(NAMESPACE)

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

# ROS namespace the planner answers on, on the real robot. Kept at the
# gbplanner2 value so operator tooling calling /anymal/... keeps working.
ROBOT_NAME ?= anymal

# DDS domain the whole fleet shares.
ROS_DOMAIN_ID ?= 0

# Address the containers advertise to ROS 1 (run-robot and run-robot-bridge
# both). Only needed when the master is on another machine, or when it
# registered itself under a hostname: ROS 1 hands a node's own URI to its
# peers, so with this unset they get the container's hostname, which they
# usually cannot resolve, and the connection dies with no useful error. Set it
# to this host's IP on the robot's network:
#   make run-robot NAMESPACE=robot0 ROS_MASTER_URI=http://192.168.5.108:11311 ROS_IP=192.168.5.108
ROS_IP ?=

# Passed only when non-empty. `-e ROS_IP=` would *set* the variable to the
# empty string, and rosgraph.network.get_address_override() tests for the
# variable's presence rather than its value - so every node would advertise
# itself at an empty address. Same reason ROS_HOSTNAME is never passed here:
# leaving it absent is what makes ROS 1 fall back to its own detection.
ROS_IP_ARG := $(if $(ROS_IP),-e ROS_IP=$(ROS_IP),)

export ROOT_DIR
export ROS_DISTRO
export GBPLANNER3_VERSION
export LAUNCH_FILE
export LAUNCH_ARGS
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

# ---------------------------------------------------------------------------
# Real robot (`make run-robot` / `make run-robot-bridge`)
#
# Deliberately two targets, not one: after run-robot the whole TF picture is
# inspectable on the robot's own ROS 1 master with no ROS 2 in the way, so a
# problem there cannot be confused with a bridge or DDS problem. Bring the
# bridge up only once /tf and /$(TF_PREFIX)/tf both look right.
#
# Neither target calls xhost or requires ssh-agent, unlike run/run-sim: a robot
# is usually headless, and there `xhost` fails and aborts the recipe.
#
# Both attach to the roscore the host stack already brought up, so
# ROS_MASTER_URI must point at it (default localhost:11311, via network_mode:
# host).
# ---------------------------------------------------------------------------
run-robot: ## Planner + TF prefixing on the robot: make run-robot NAMESPACE=robot0
	@echo "gbplanner + tf prefix '$(TF_PREFIX)' on $(ROS_MASTER_URI) (ns /$(ROBOT_NAME))..."
	@LAUNCH_FILE=anymal_robot.launch \
	 LAUNCH_ARGS="robot_name:=$(ROBOT_NAME) prefix:=$(TF_PREFIX)" \
	 $(COMPOSE) run --rm --no-deps --name gbplanner-robot-$(NAMESPACE) \
	   -e ROS_MASTER_URI=$(ROS_MASTER_URI) $(ROS_IP_ARG) run

run-robot-bridge: ## ROS1<->ROS2 bridge for the robot: make run-robot-bridge NAMESPACE=robot0
	@test -n "$(BRIDGE_VERSION)" || (echo "BRIDGE_VERSION is empty - bridge/ is not checked out. Run: git submodule update --init --recursive" && exit 1)
	@# ppa:ros-for-jammy/noble publishes amd64 binaries only, so on an arm64
	@# robot the bridge image rebuilds ROS 1 from that ppa's source packages
	@# instead - see bridge/docker/build-noetic-from-source.sh. Running this
	@# from an x86 host against the robot's master also works; that case needs
	@# ROS_IP so the master's nodes can reach back.
	@case "$(ROS_MASTER_URI)" in \
	  *localhost*|*127.0.0.1*) ;; \
	  *) test -n "$(ROS_IP)" || (echo "ROS_MASTER_URI is remote but ROS_IP is unset. The robot's ROS 1 nodes would get this host's hostname and fail to connect back. Pass ROS_IP=<this host's IP on the robot network>." && exit 1) ;; \
	esac
	@echo "bridge for '$(NAMESPACE)' on $(ROS_MASTER_URI), domain $(ROS_DOMAIN_ID), ROS_IP=$(ROS_IP), fleet TF on..."
	@# BRIDGE_TF_FLEET picks tf_fleet_relay over tf_static_repeater, and
	@# sim_time off keeps the ROS 1 side on wall clock. bridge_topics.yaml has
	@# to bridge /{ns}/tf and /{ns}/tf_static for this to carry anything.
	@$(COMPOSE) run --rm --no-deps --name ros1-bridge-robot-$(NAMESPACE) \
	   -e BRIDGE_NAMESPACE=$(NAMESPACE) \
	   -e BRIDGE_USE_SIM_TIME=false \
	   -e BRIDGE_TF_FLEET=true \
	   -e ROS_DOMAIN_ID=$(ROS_DOMAIN_ID) \
	   -e ROS_MASTER_URI=$(ROS_MASTER_URI) $(ROS_IP_ARG) bridge

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

.PHONY: build build-bridge build-all bootstrap rebuild run-dev run run-sim run-robot run-robot-bridge stop-sim enter-dev stop clean help
