cc_library(
  NAME ros
  #DEPS rclcpp geometry_msgs tf2 tf2_ros
  #INCS ${catkin_INCLUDE_DIRS}
  INTERFACE)
ament_target_dependencies(sv_ros INTERFACE
    rclcpp
    cv_bridge
    geometry_msgs
    sensor_msgs
    nav_msgs
    visualization_msgs
    tf2
    tf2_ros
)

cc_library(
  NAME ros_msg_conv
  SRCS "msg_conv.cpp"
  DEPS sv_base sv_ros Eigen3::Eigen Sophus::Sophus
  INCS ${OpenCV_INCLUDE_DIRS}
)
