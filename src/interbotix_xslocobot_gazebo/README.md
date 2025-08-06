# interbotix_xslocobot_gazebo

[![View Documentation](https://docs.trossenrobotics.com/docs_button.svg)](https://docs.trossenrobotics.com/interbotix_xslocobots_docs/ros1_packages/gazebo_simulation_configuration.html)

## Overview

This package contains the necessary config files to get any rover in our X-Series Interbotix LoCoBot Family simulated in Gazebo. Specifically, it contains the [locobot_configs.gazebo](config/locobot_configs.gazebo) file which allows the black texture of the robot to display properly (following the method explained [here](http://answers.gazebosim.org/question/16280/how-to-use-custom-textures-on-urdf-models-in-gazebo/)) and starts various plugins. It also contains YAML files with tuned PID gains for the arm, gripper, and pan/tilt joints so that ros_control can control the robot effectively. This package can either be used in conjunction with MoveIt via the FollowJointTrajectory interface or by itself via the JointPositionController interface.
