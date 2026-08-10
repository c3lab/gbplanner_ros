# GBPlanner 3.0: Universal Exploration and Inspection Path Planning across Robot Morphologies (aka OmniPlanner)

![swag](img/cerberus_subt_winners.png)
> **_NOTE:_** In CERBERUS, during the DARPA Subterranean Challenge, an older version - GBPlanner 2.0 - was used.

We present the State-Of-The-Art Graph-Based Exploration and Inspection Path Planner: GBPlanner3. 
> **_NOTE:_** The full OmniPlanner codebase, along with usage examples, will be provided.

For an extensive documentation, installation instructions, and demos please visit the documentation page of the repository here: [**Documetation**](https://github.com/ntnu-arl/gbplanner3_wiki/wiki).

## Installation

Everything is built and run inside Docker, driven by the `Makefile` at the root
of the repository. Nothing is installed on the host, and the upstream manual
workspace setup (Gazebo Garden from source, `ros_gz` bridge, catkin workspace)
is reproduced by `docker/bootstrap.sh` instead.

### Host requirements

* Docker with the Compose plugin
* NVIDIA Container Toolkit — without it Gazebo and RViz silently fall back to
  CPU software rendering
* An `ssh-agent` holding a key with access to the cloned repositories: the
  bootstrap step clones over SSH
* `git` and `git-lfs`
* An X server, for RViz

### Clone

```bash
git clone git@github.com:c3lab/gbplanner_ros.git -b gbplanner3-dghost
cd gbplanner_ros
git submodule update --init --recursive
```

`--recursive` is not optional: `bridge/` carries a nested submodule of its own
(the ROS 2 `planner_msgs`), and the bridge image cannot be built without it.

### Build

```bash
eval $(ssh-agent) && ssh-add
make build-all
```

`build-all` runs the three build steps in order. Each is also available on its
own:

| Target | What it does |
| --- | --- |
| `make build` | Builds the `gbplanner:noetic-3.0.0` container image only. |
| `make bootstrap` | Clones and builds the full workspace into `bootstrap/` — Gazebo Garden, the `ros_gz` bridge and `gbplanner3_ws`. This is the long one. The Gazebo Garden sources come from `docker/collection-garden.lock.yaml`, which pins every repository to a commit: Garden is EOL but its branches still move, and a moving `gz-common5` stops compiling against the assimp 5.0.1 that Ubuntu 20.04 ships. Bump the pins there, not by re-pointing at an upstream collection file. |
| `make build-bridge` | Builds the ROS 1 ↔ ROS 2 `ros1_bridge` image from the `bridge/` submodule. Copies the ROS 1 `planner_msgs` into the bridge build context first: `ros1_bridge` generates its conversion factories at build time, so the message definitions have to be present before the image is built. |

`make rebuild` does an incremental catkin build of a single package
(`REBUILD_PKG`, default `gbplanner`) over the already bootstrapped workspace.
Only C++ and message changes need it — launch files and YAML configs are read
from the bind-mounted source tree at run time.

## Usage

### Simulation

Use `make run-sim <namespace>`. It brings up the namespaced planner and the
`ros1_bridge` together, with `use_sim_time` enabled and the sim topic layout
(`sensor_measurements/odom`, `sensor_measurements/lidar/points`):

```bash
make run-sim robot0
```

Two containers start: `gbplanner_ros1-robot0`, running
`sim_gbplanner.launch` (which brings up its own `roscore`), and
`ros1-bridge-robot0`, bridging that namespace. The bridge waits for the ROS 1
master, so start order does not matter. `Ctrl-C` stops both.

Append `rebuild` to rebuild the `gbplanner` package before launching:

```bash
make run-sim robot0 rebuild
```

Each namespace runs in its own Compose project, so a second robot adds
containers rather than replacing the first one's. Give each robot its own
master: `ROS_MASTER_URI` is passed to both containers of a pair, the planner's
`roslaunch` starts its `roscore` on that port and the bridge attaches to the
same one. Reusing one port instead makes the second robot join the first one's
graph rather than getting a master of its own.

```bash
make run-sim robot0 ROS_MASTER_URI=http://localhost:11311
make run-sim robot1 ROS_MASTER_URI=http://localhost:11312
make stop-sim NAMESPACE=robot0
```

So N robots means 2N containers: N planners on N masters, and one bridge
attached to each. `RVIZ=false` drops RViz from a robot, which is usually what
you want for every robot you are not watching:

```bash
make run-sim robot1 RVIZ=false ROS_MASTER_URI=http://localhost:11312
```

The planner runs on `use_sim_time`, so it stays frozen at time zero until
something on the ROS 2 side publishes `/clock` through the bridge.

Bridged topics and services live in `bridge/entrypoint/bridge_topics.yaml`,
which is bind-mounted into the container — editing it needs a restart, not a
rebuild.

### Standalone demos

The self-contained Gazebo scenarios shipped upstream are launched with
`make run`:

```bash
make run                                              # uav_gz_cave_exploration.launch
make run LAUNCH_FILE=ugv_gzc_urban_exploration.launch
```

`LAUNCH_FILE` is a file name without any directory: `roslaunch` resolves launch
files by base name, so a path like `uav/gz/uav_gz_cave_exploration.launch` does
not resolve.

### Development

```bash
make run-dev     # interactive shell over the bootstrapped workspace
make enter-dev   # attach another shell to the running dev container
make stop        # stop and remove the containers
make clean       # stop the containers and drop the image
```

`make help` lists every target.

## Robots using GBPlanner, GBPlanner2, GBPlanner3:
![robots](img/gbplanner3_robots.png)


If you use this work in your research, please cite the following publications:

**Graph-based subterranean exploration path planning using aerial and legged robots**
```
@article{dang2020graph,
  title={Graph-based subterranean exploration path planning using aerial and legged robots},
  author={Dang, Tung and Tranzatto, Marco and Khattak, Shehryar and Mascarich, Frank and Alexis, Kostas and Hutter, Marco},
  journal={Journal of Field Robotics},
  volume = {37},
  number = {8},
  pages = {1363-1388},  
  year={2020},
  note={Wiley Online Library}
}
```
**Autonomous Teamed Exploration of Subterranean Environments using Legged and Aerial Robots**
```
@INPROCEEDINGS{9812401,
  author={Kulkarni, Mihir and Dharmadhikari, Mihir and Tranzatto, Marco and Zimmermann, Samuel and Reijgwart, Victor and De Petris, Paolo and Nguyen, Huan and Khedekar, Nikhil and Papachristos, Christos and Ott, Lionel and Siegwart, Roland and Hutter, Marco and Alexis, Kostas},
  booktitle={2022 International Conference on Robotics and Automation (ICRA)}, 
  title={Autonomous Teamed Exploration of Subterranean Environments using Legged and Aerial Robots}, 
  year={2022},
  volume={},
  number={},
  pages={3306-3313},
  doi={10.1109/ICRA46639.2022.9812401}}
```

**OmniPlanner: Universal Exploration and Inspection Path Planning across Robot Morphologies**
```
@article{zacharia2026omniplanner,
  title   = {OmniPlanner: Universal Exploration and Inspection Path Planning across Robot Morphologies},
  author  = {Zacharia, Angelos and Dharmadhikari, Mihir and Singh, Mohit and Alexis, Kostas},
  journal = {arXiv preprint arXiv:2603.04284},
  year    = {2026},
  url     = {https://arxiv.org/abs/2603.04284}
}
```

You can contact us for any question:
* [Tung Dang](mailto:tung.dang@nevada.unr.edu)
* [Mihir Dharmadhikari](mailto:mihir.dharmadhikari@ntnu.no)
* [Angelos Zacharia](mailto:angelos.zacharia@ntnu.no)
* [Kostas Alexis](mailto:konstantinos.alexis@ntnu.no)

## Acknowledgements 
This work was developed throughout multiple research activities funded by DARPA (under Agreement No. HR00111820045), the Research Council of Norway (Proj. Number: 321435), and Horizon Europe (101070405, 101120732, 101121321, 101119774). The presented content and ideas are solely those of the authors.
This code is intended for civilian use only. It is provided under the license found in [LICENSE](https://github.com/ntnu-arl/gbplanner_ros/blob/gbplanner3/LICENSE).
