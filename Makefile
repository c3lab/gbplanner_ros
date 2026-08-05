ROS_DISTRO := noetic
GBPLANNER3_VERSION := 3.0.0
ROOT_DIR := $(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))

# Launch file passed to `roslaunch gbplanner`, relative to gbplanner/launch/.
# Override on the command line: make run LAUNCH_FILE=ugv/gzc/ugv_gzc_urban_exploration.launch
LAUNCH_FILE ?= uav/gz/uav_gz_cave_exploration.launch

export ROOT_DIR
export ROS_DISTRO
export GBPLANNER3_VERSION
export LAUNCH_FILE

COMPOSE := docker compose
CONTAINER_NAME := gbplanner_ros1

default: help

build: ## Build release container image
	@echo "Building gbplanner:$(ROS_DISTRO)-$(GBPLANNER3_VERSION) container image..."
	@$(COMPOSE) build base

bootstrap: ## One-shot: clone and build the full workspace into bootstrap/
	@echo "Bootstrapping gbplanner:$(ROS_DISTRO)-$(GBPLANNER3_VERSION) workspace..."
	@chmod +x $(ROOT_DIR)/docker/bootstrap.sh
	@$(COMPOSE) run --rm bootstrap

run-dev: ## Interactive dev shell over the bootstrapped workspace
	@test -n "$$SSH_AUTH_SOCK" || (echo "SSH_AUTH_SOCK is not set. Start ssh-agent and run ssh-add before make run-dev." && exit 1)
	@xhost +SI:localuser:root >/dev/null
	@$(COMPOSE) run --rm dev

run: ## Launch gbplanner (LAUNCH_FILE=$(LAUNCH_FILE))
	@test -n "$$SSH_AUTH_SOCK" || (echo "SSH_AUTH_SOCK is not set. Start ssh-agent and run ssh-add before make run." && exit 1)
	@xhost +SI:localuser:root >/dev/null
	@echo "Launching gbplanner $(LAUNCH_FILE)..."
	@$(COMPOSE) run --rm run

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

.PHONY: build bootstrap run-dev run enter-dev stop clean help
