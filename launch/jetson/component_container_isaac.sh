#!/bin/bash
export LD_PRELOAD=$(ros2 pkg prefix isaac_ros_gxf)/share/isaac_ros_gxf/gxf/lib/core/libgxf_core.so
exec $(ros2 pkg prefix rclcpp_components)/lib/rclcpp_components/component_container_mt "$@"