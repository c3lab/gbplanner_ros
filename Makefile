ROS_DISTRO := noetic
GBPLANNER3_VERSION := 3.0.0

CONTAINER_IMAGE := gbplanner:$(ROS_DISTRO)-$(GBPLANNER3_VERSION)
CONTAINER_NAME := gbplanner_ros1


PERCENT := %
ROOT_DIR := $(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))

default: help


build: ## Build release container
	@echo "Building $(CONTAINER_IMAGE) container image..."
	@docker build \
		--tag $(CONTAINER_IMAGE) \
		--file docker/Dockerfile.base \
		--build-arg ROS_DISTRO=$(ROS_DISTRO) \
		.

bootstrap: ## Build development container
	@echo "Bootstrapping $(CONTAINER_IMAGE) container image for development..."
	@chmod +x $(ROOT_DIR)/docker/bootstrap.sh
	@docker run \
		--interactive \
		--tty \
		--rm \
		--runtime nvidia \
		--gpus all \
		--privileged \
		--net host \
		--ipc host \
		--name ${CONTAINER_NAME}-bootstrap \
		--volume /tmp/.X11-unix:/tmp/.X11-unix \
		--volume ~/.Xauthority:/root/.Xauthority \
		--env DISPLAY=$$DISPLAY \
		--env XAUTHORITY=$$XAUTHORITY \
		--env SSH_AUTH_SOCK=/ssh-agent \
		--volume "$$SSH_AUTH_SOCK:/ssh-agent" \
		--volume $(ROOT_DIR):/workspace/src/ \
		--volume $(ROOT_DIR)/docker/bootstrap.sh:/workspace/bootstrap.sh \
		--volume $(ROOT_DIR)/bootstrap/gazebo_garden_ws:/workspace/gazebo_garden_ws \
		--volume $(ROOT_DIR)/bootstrap/ros_gz_bridge_ws:/workspace/ros_gz_bridge_ws \
		--volume $(ROOT_DIR)/bootstrap/exploration/BehaviorTree.CPP:/workspace/gbplanner3_ws/src/exploration/BehaviorTree.CPP \
		--volume $(ROOT_DIR)/bootstrap/exploration/adaptive_obb_ros:/workspace/gbplanner3_ws/src/exploration/adaptive_obb_ros \
		--volume $(ROOT_DIR)/bootstrap/exploration/gbplanner_ros:/workspace/gbplanner3_ws/src/exploration/gbplanner_ros \
		--volume $(ROOT_DIR)/bootstrap/exploration/manhole_detector_ros:/workspace/gbplanner3_ws/src/exploration/manhole_detector_ros \
		--volume $(ROOT_DIR)/bootstrap/exploration/pci_general:/workspace/gbplanner3_ws/src/exploration/pci_general \
		--volume $(ROOT_DIR)/bootstrap/mapping/voxblox:/workspace/gbplanner3_ws/src/mapping/voxblox \
		--volume $(ROOT_DIR)/bootstrap/misc/catkin_boost_python_buildtool:/workspace/gbplanner3_ws/src/misc/catkin_boost_python_buildtool \
		--volume $(ROOT_DIR)/bootstrap/misc/catkin_simple:/workspace/gbplanner3_ws/src/misc/catkin_simple \
		--volume $(ROOT_DIR)/bootstrap/misc/eigen_catkin:/workspace/gbplanner3_ws/src/misc/eigen_catkin \
		--volume $(ROOT_DIR)/bootstrap/misc/eigen_checks:/workspace/gbplanner3_ws/src/misc/eigen_checks \
		--volume $(ROOT_DIR)/bootstrap/misc/gflags_catkin:/workspace/gbplanner3_ws/src/misc/gflags_catkin \
		--volume $(ROOT_DIR)/bootstrap/misc/glog_catkin:/workspace/gbplanner3_ws/src/misc/glog_catkin \
		--volume $(ROOT_DIR)/bootstrap/misc/kindr:/workspace/gbplanner3_ws/src/misc/kindr \
		--volume $(ROOT_DIR)/bootstrap/misc/mav_comm:/workspace/gbplanner3_ws/src/misc/mav_comm \
		--volume $(ROOT_DIR)/bootstrap/misc/minkindr:/workspace/gbplanner3_ws/src/misc/minkindr \
		--volume $(ROOT_DIR)/bootstrap/misc/minkindr_ros:/workspace/gbplanner3_ws/src/misc/minkindr_ros \
		--volume $(ROOT_DIR)/bootstrap/misc/multi_dof_joint_trajectory_rviz_plugins:/workspace/gbplanner3_ws/src/misc/multi_dof_joint_trajectory_rviz_plugins \
		--volume $(ROOT_DIR)/bootstrap/misc/numpy_eigen:/workspace/gbplanner3_ws/src/misc/numpy_eigen \
		--volume $(ROOT_DIR)/bootstrap/misc/protobuf_catkin:/workspace/gbplanner3_ws/src/misc/protobuf_catkin \
		--volume $(ROOT_DIR)/bootstrap/sim/arl_gazebo_sim_ros:/workspace/gbplanner3_ws/src/sim/arl_gazebo_sim_ros \
		--volume $(ROOT_DIR)/bootstrap/sim/lidar_simulator:/workspace/gbplanner3_ws/src/sim/lidar_simulator \
		--volume $(ROOT_DIR)/bootstrap/sim/rotors_simulator:/workspace/gbplanner3_ws/src/sim/rotors_simulator \
		--volume $(ROOT_DIR)/bootstrap/sim/smb_simulator:/workspace/gbplanner3_ws/src/sim/smb_simulator \
		--volume $(ROOT_DIR)/bootstrap/sim/subt_cave_sim:/workspace/gbplanner3_ws/src/sim/subt_cave_sim \
		--volume $(ROOT_DIR)/gbplanner:/workspace/gbplanner3_ws/src/exploration/gbplanner \
		--volume $(ROOT_DIR)/gbplanner_ui:/workspace/gbplanner3_ws/src/exploration/gbplanner_ui \
		--volume $(ROOT_DIR)/kdtree:/workspace/gbplanner3_ws/src/exploration/kdtree \
		--volume $(ROOT_DIR)/map_manager:/workspace/gbplanner3_ws/src/exploration/map_manager \
		--volume $(ROOT_DIR)/planner_common:/workspace/gbplanner3_ws/src/exploration/planner_common \
		--volume $(ROOT_DIR)/planner_control_interface:/workspace/gbplanner3_ws/src/exploration/planner_control_interface \
		--volume $(ROOT_DIR)/planner_gazebo_sim:/workspace/gbplanner3_ws/src/exploration/planner_gazebo_sim \
		--volume $(ROOT_DIR)/planner_msgs:/workspace/gbplanner3_ws/src/exploration/planner_msgs \
		--volume $(ROOT_DIR)/planner_semantic_msgs:/workspace/gbplanner3_ws/src/exploration/planner_semantic_msgs \
		--volume $(ROOT_DIR)/ugv_simulator:/workspace/gbplanner3_ws/src/exploration/ugv_simulator \
		--volume $(ROOT_DIR)/vcstool:/workspace/gbplanner3_ws/src/exploration/vcstool \
		--volume $(ROOT_DIR)/bootstrap/gbplanner3_ws/build:/workspace/gbplanner3_ws/build \
		--volume $(ROOT_DIR)/bootstrap/gbplanner3_ws/logs:/workspace/gbplanner3_ws/log \
		--volume $(ROOT_DIR)/bootstrap/gbplanner3_ws/install:/workspace/gbplanner3_ws/install \
		--volume $(ROOT_DIR)/bootstrap/gbplanner3_ws/devel:/workspace/gbplanner3_ws/devel \
		--volume $(ROOT_DIR)/bootstrap/install:/workspace/install \
		$(CONTAINER_IMAGE) \
		bash -ci "/workspace/bootstrap.sh"

run-dev: ## Run container in development mode
	@echo "Running $(CONTAINER_IMAGE) container in development mode..."
	@test -n "$$SSH_AUTH_SOCK" || (echo "SSH_AUTH_SOCK is not set. Start ssh-agent and run ssh-add before make run-dev." && exit 1)
	@xhost +
	@docker run \
		--interactive \
		--tty \
		--rm \
		--runtime nvidia \
		--gpus all \
		--privileged \
		--net host \
		--ipc host \
		--name ${CONTAINER_NAME}-dev \
		--volume /tmp/.X11-unix:/tmp/.X11-unix \
		--volume ~/.Xauthority:/root/.Xauthority \
		--env DISPLAY=$$DISPLAY \
		--env XAUTHORITY=$$XAUTHORITY \
		--env SSH_AUTH_SOCK=/ssh-agent \
		--volume "$$SSH_AUTH_SOCK:/ssh-agent" \
		--volume $(ROOT_DIR)/bootstrap/gazebo_garden_ws:/workspace/gazebo_garden_ws \
		--volume $(ROOT_DIR)/bootstrap/ros_gz_bridge_ws:/workspace/ros_gz_bridge_ws \
		--volume $(ROOT_DIR)/bootstrap/exploration/BehaviorTree.CPP:/workspace/gbplanner3_ws/src/exploration/BehaviorTree.CPP \
		--volume $(ROOT_DIR)/bootstrap/exploration/adaptive_obb_ros:/workspace/gbplanner3_ws/src/exploration/adaptive_obb_ros \
		--volume $(ROOT_DIR)/bootstrap/exploration/gbplanner_ros:/workspace/gbplanner3_ws/src/exploration/gbplanner_ros \
		--volume $(ROOT_DIR)/bootstrap/exploration/manhole_detector_ros:/workspace/gbplanner3_ws/src/exploration/manhole_detector_ros \
		--volume $(ROOT_DIR)/bootstrap/exploration/pci_general:/workspace/gbplanner3_ws/src/exploration/pci_general \
		--volume $(ROOT_DIR)/bootstrap/mapping/voxblox:/workspace/gbplanner3_ws/src/mapping/voxblox \
		--volume $(ROOT_DIR)/bootstrap/misc/catkin_boost_python_buildtool:/workspace/gbplanner3_ws/src/misc/catkin_boost_python_buildtool \
		--volume $(ROOT_DIR)/bootstrap/misc/catkin_simple:/workspace/gbplanner3_ws/src/misc/catkin_simple \
		--volume $(ROOT_DIR)/bootstrap/misc/eigen_catkin:/workspace/gbplanner3_ws/src/misc/eigen_catkin \
		--volume $(ROOT_DIR)/bootstrap/misc/eigen_checks:/workspace/gbplanner3_ws/src/misc/eigen_checks \
		--volume $(ROOT_DIR)/bootstrap/misc/gflags_catkin:/workspace/gbplanner3_ws/src/misc/gflags_catkin \
		--volume $(ROOT_DIR)/bootstrap/misc/glog_catkin:/workspace/gbplanner3_ws/src/misc/glog_catkin \
		--volume $(ROOT_DIR)/bootstrap/misc/kindr:/workspace/gbplanner3_ws/src/misc/kindr \
		--volume $(ROOT_DIR)/bootstrap/misc/mav_comm:/workspace/gbplanner3_ws/src/misc/mav_comm \
		--volume $(ROOT_DIR)/bootstrap/misc/minkindr:/workspace/gbplanner3_ws/src/misc/minkindr \
		--volume $(ROOT_DIR)/bootstrap/misc/minkindr_ros:/workspace/gbplanner3_ws/src/misc/minkindr_ros \
		--volume $(ROOT_DIR)/bootstrap/misc/multi_dof_joint_trajectory_rviz_plugins:/workspace/gbplanner3_ws/src/misc/multi_dof_joint_trajectory_rviz_plugins \
		--volume $(ROOT_DIR)/bootstrap/misc/numpy_eigen:/workspace/gbplanner3_ws/src/misc/numpy_eigen \
		--volume $(ROOT_DIR)/bootstrap/misc/protobuf_catkin:/workspace/gbplanner3_ws/src/misc/protobuf_catkin \
		--volume $(ROOT_DIR)/bootstrap/sim/arl_gazebo_sim_ros:/workspace/gbplanner3_ws/src/sim/arl_gazebo_sim_ros \
		--volume $(ROOT_DIR)/bootstrap/sim/lidar_simulator:/workspace/gbplanner3_ws/src/sim/lidar_simulator \
		--volume $(ROOT_DIR)/bootstrap/sim/rotors_simulator:/workspace/gbplanner3_ws/src/sim/rotors_simulator \
		--volume $(ROOT_DIR)/bootstrap/sim/smb_simulator:/workspace/gbplanner3_ws/src/sim/smb_simulator \
		--volume $(ROOT_DIR)/bootstrap/sim/subt_cave_sim:/workspace/gbplanner3_ws/src/sim/subt_cave_sim \
		--volume $(ROOT_DIR)/gbplanner:/workspace/gbplanner3_ws/src/exploration/gbplanner \
		--volume $(ROOT_DIR)/gbplanner_ui:/workspace/gbplanner3_ws/src/exploration/gbplanner_ui \
		--volume $(ROOT_DIR)/kdtree:/workspace/gbplanner3_ws/src/exploration/kdtree \
		--volume $(ROOT_DIR)/map_manager:/workspace/gbplanner3_ws/src/exploration/map_manager \
		--volume $(ROOT_DIR)/planner_common:/workspace/gbplanner3_ws/src/exploration/planner_common \
		--volume $(ROOT_DIR)/planner_control_interface:/workspace/gbplanner3_ws/src/exploration/planner_control_interface \
		--volume $(ROOT_DIR)/planner_gazebo_sim:/workspace/gbplanner3_ws/src/exploration/planner_gazebo_sim \
		--volume $(ROOT_DIR)/planner_msgs:/workspace/gbplanner3_ws/src/exploration/planner_msgs \
		--volume $(ROOT_DIR)/planner_semantic_msgs:/workspace/gbplanner3_ws/src/exploration/planner_semantic_msgs \
		--volume $(ROOT_DIR)/ugv_simulator:/workspace/gbplanner3_ws/src/exploration/ugv_simulator \
		--volume $(ROOT_DIR)/vcstool:/workspace/gbplanner3_ws/src/exploration/vcstool \
		--volume $(ROOT_DIR)/bootstrap/gbplanner3_ws/build:/workspace/gbplanner3_ws/build \
		--volume $(ROOT_DIR)/bootstrap/gbplanner3_ws/logs:/workspace/gbplanner3_ws/log \
		--volume $(ROOT_DIR)/bootstrap/gbplanner3_ws/install:/workspace/gbplanner3_ws/install \
		--volume $(ROOT_DIR)/bootstrap/gbplanner3_ws/devel:/workspace/gbplanner3_ws/devel \
		--volume $(ROOT_DIR)/bootstrap/install:/workspace/install \
		$(CONTAINER_IMAGE) \
		bash

enter-dev: ## Enter running container in development mode
	@echo "Entering $(CONTAINER_IMAGE) container..."
	@docker exec -it ${CONTAINER_NAME}-dev bash

clean: ## Clean up container image
	@echo "Cleaning up $(CONTAINER_IMAGE) container image..."
	@docker rmi $(CONTAINER_IMAGE) || true

help: ## Show this help message
	@echo "Usage: make [target]"
	@echo ""
	@echo "Targets:"
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | sort | awk 'BEGIN {FS = ":.*?## "} {printf "  \033[36m%-20s\033[0m %s\n", $$1, $$2}'

.PHONY: build bootstrap run-dev enter-dev clean help
