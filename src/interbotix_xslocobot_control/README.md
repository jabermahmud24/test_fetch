# interbotix_xslocobot_control

[![View Documentation](https://docs.trossenrobotics.com/docs_button.svg)](https://docs.trossenrobotics.com/interbotix_xslocobots_docs/ros1_packages/locobot_control.html)

## Overview

This package contains the configuration and launch files necessary to easily start the various components of the X-Series LoCoBot platform. This includes launching the **xs_sdk** node responsible for driving the DYNAMIXEL motors on the robot, loading the URDF to the `robot_description` parameter, starting the base, and activating the RealSense depth camera and RPLidar 2D laser scanner. Essentially, this package is what all 'downstream' ROS packages should reference to get the robot up and running.
