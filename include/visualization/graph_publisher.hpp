#ifndef GRAPH_PUBLISHER_HPP
#define GRAPH_PUBLISHER_HPP


#include <math.h>
#include <pcl_conversions/pcl_conversions.h>
#include <boost/format.hpp>
#include <cmath>
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "situational_graphs_reasoning_msgs/msg/graph.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <tf2_eigen/tf2_eigen.hpp>

// ORB-SLAM3
#include "System.h"
#include "ImuTypes.h"
#include "Types/SystemParams.h"
#include "Semantic/Room.h"

namespace GraphPublisher {

/**
 * @brief Build and return a graph message from an ORB-SLAM3 map.
 *
 * @param pCurrentMap   Pointer to the current ORB-SLAM3 map
 * @param graph_type    "Prior" or "Online"
 * @param tfStampedSE   Transform from SE frame to world frame
 * @return situational_graphs_reasoning_msgs::msg::Graph
 */
situational_graphs_reasoning_msgs::msg::Graph publish_graph(
    ORB_SLAM3::Map* pCurrentMap,
    std::string graph_type,
    geometry_msgs::msg::TransformStamped tfStampedSE,geometry_msgs::msg::TransformStamped tfStampedBC);

}  // namespace GraphPublisher

#endif  // GRAPH_PUBLISHER_HPP