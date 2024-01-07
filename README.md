# RFN3D Planner for MRS simulator
This is the University of Virginia's Robust Fast Navigation (3D) planner, modified to run on top of CTU's [MRS UAV System](https://github.com/ctu-mrs/mrs_uav_system)

## Installation
This package has the following dependencies:
* Apt repositories: FCL, Octomap, PCL
  ```bash
  sudo apt-get install libfcl-dev libpcl-dev ros-<ros_version>-octomap*
  ```
* [OMPL](https://ompl.kavrakilab.org/installation.html)
* [Gurobi](https://www.gurobi.com/downloads/gurobi-software/)

## Running
Consists of two nodes, the `planner` and the `local_octomap`. The `planner` takes care of collision-avoidant trajectories based on the octomap generated by `local_octomap`. The planner should already be setup to run with the UAS_MAPPING_CORE 
simulation. It can be launched via

```bash
roslaunch rfn3d planner.launch
```

## Parameters
The system has parameters to adjust the octomap, trajectory generation, and planner logistics. These parameters and their descriptions can be found in `./params/params.yaml`