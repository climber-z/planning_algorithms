# How to use this package?
1. Create a directory named "catkin_ws" (or any other names you like).
2. Copy the ros/src directory into "catkin_ws".
3. Then `catkin_make` and `source devel/setup.bash`.
4. Run roscore and rviz (configure file is 'ros/src/rviz_plugins/config/lattice_graph_demo.rviz').
5. Run the demo node with command: `roslaunch grid_path_searcher demo.launch`.

## lattice_graph plan result
![](https://github.com/climber-z/planning_algorithms/blob/main/lattice_graph/ros/img/lattice_graph_1.png)
![](https://github.com/climber-z/planning_algorithms/blob/main/lattice_graph/ros/img/lattice_graph_2.png)

Note: the paths in red are not collision-free, in blue are collision-free but not optimal, in green is the optimal.

This is a local planner which use the method of sampling in control space, and the optimal method is solving the OBVP problem of local path's end point to the target point.
For more details of OBVP's mathematical derivation, please see the "obvp_derivation.pdf" document in this directory.
