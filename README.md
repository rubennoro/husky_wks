This directory is a ROS2 package for Husky, a robot capable of walking and flight. Within the package contains the path planner for Husky, as well as nodes to publish data in the given environment that can be visualized in Rviz2.

To Use:

1. Run: colcon build
2. Run: source install/setup.bash 
Open several terminal tabs, and source your workspace in each of them.

3. Run: rviz2
4. Run: ros2 run husky_planner build_env_node
5. Run: ros2 run husky_planner path_planner

build_env_node is a publishing node that publishes the environment migrated from the Matlab code to Rviz2.

path_planner is the publishing node that runs path planning and publishes the connections between waypoints around the given environment obstacles.
