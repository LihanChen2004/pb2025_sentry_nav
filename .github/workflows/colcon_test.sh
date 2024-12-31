#!/bin/bash
source /opt/ros/humble/setup.sh
cd ros_ws
colcon test --packages-up-to "$1" --event-handlers console_cohesion+ --return-code-on-test-failure
