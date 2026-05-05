/*
Copyright (c) 2023, University of Luxembourg
All rights reserved.

Redistributions and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 'AS IS'
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*/

#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <include/visualization/graph_publisher.hpp>
#include <vector>
constexpr int PLANE_ID_OFFSET = 1000000;
namespace{

  void _extract_room_wall_edges(ORB_SLAM3::Room *const room, situational_graphs_reasoning_msgs::msg::Edge &graph_edge, situational_graphs_reasoning_msgs::msg::Attribute &edge_attribute, std::vector<situational_graphs_reasoning_msgs::msg::Attribute> &edge_att_vec, std::vector<situational_graphs_reasoning_msgs::msg::Edge, std::allocator<situational_graphs_reasoning_msgs::msg::Edge>> &edges_vec)
  {
    /*Function to create the edges connecting room and walls leveraging Atlas Functions*/
    for (auto wall : room->getWalls())
    {
      if (wall->isBad())continue;
      graph_edge.origin_node = room->getId();
      graph_edge.target_node = wall->getId()+PLANE_ID_OFFSET;
      edge_attribute.name = "EdgeRoom4Planes";
      edge_att_vec.push_back(edge_attribute);
      graph_edge.attributes = edge_att_vec;
      edges_vec.push_back(graph_edge);
      edge_att_vec.clear();
    }
  }
  void _extract_plane_nodes(std::vector<ORB_SLAM3::Plane *> &planes, std::vector<situational_graphs_reasoning_msgs::msg::Node, std::allocator<situational_graphs_reasoning_msgs::msg::Node>> &nodes_vec,geometry_msgs::msg::TransformStamped tfStampedBC)
  {

    for (const auto plane : planes)
    {
      if(plane->isBad()) continue;
      if (plane->getExpectedPlaneType() != ORB_SLAM3::Plane::WALL) continue; //Removing non vertical planes
      situational_graphs_reasoning_msgs::msg::Node graph_node;
      situational_graphs_reasoning_msgs::msg::Attribute node_attribute;

      graph_node.id = plane->getId()+PLANE_ID_OFFSET;
      graph_node.type = "Plane";
      node_attribute.name = "Geometric_info";
      Eigen::Vector4d plane_coeffs = plane->getGlobalEquation().coeffs();
      Eigen::Vector4d plane_coeffs_tr;
      
      try {


          
          tf2::Transform transform;
          tf2::fromMsg(tfStampedBC.transform, transform);
          
          Eigen::Isometry3d T = tf2::transformToEigen(tfStampedBC);
          plane_coeffs_tr = T.matrix().inverse().transpose() * plane_coeffs;

          // Rotazione fissa 90° attorno a X: ORB-SLAM3 → ROS
          Eigen::Matrix4d R90 = Eigen::Matrix4d::Identity();
          R90(1,1) =  0; R90(1,2) = 1;
          R90(2,1) = -1; R90(2,2) = 0;

          plane_coeffs_tr = R90.inverse().transpose() * plane_coeffs_tr;
      }
      catch (tf2::TransformException &ex) {
          RCLCPP_WARN(rclcpp::get_logger("visual_sgraphs"),
              "Wall plane transform failed: %s", ex.what());
          plane_coeffs_tr = plane_coeffs;
      }
      node_attribute.fl_value = {plane_coeffs_tr(0), plane_coeffs_tr(1), plane_coeffs_tr(2), plane_coeffs_tr(3)};
      std::vector<situational_graphs_reasoning_msgs::msg::Attribute> node_att_vec = {node_attribute};
      graph_node.attributes = node_att_vec;
      nodes_vec.push_back(graph_node);
      node_attribute.fl_value.clear();
      node_att_vec.clear();
    }
  }
}
namespace GraphPublisher {

  situational_graphs_reasoning_msgs::msg::Graph publish_graph(ORB_SLAM3::Map* pCurrentMap,std::string graph_type,geometry_msgs::msg::TransformStamped tfStampedSE,geometry_msgs::msg::TransformStamped tfStampedBC){

    std::vector<situational_graphs_reasoning_msgs::msg::Edge> edges_vec;
    std::vector<situational_graphs_reasoning_msgs::msg::Node> nodes_vec;
    situational_graphs_reasoning_msgs::msg::Graph graph_msg;
    std::vector<situational_graphs_reasoning_msgs::msg::Attribute> edge_att_vec;
    std::vector<situational_graphs_reasoning_msgs::msg::Attribute> node_att_vec;

    auto planes = pCurrentMap->GetAllPlanes();
    auto rooms = pCurrentMap->GetAllRooms();
    if (graph_type == "Prior") {
      graph_msg.name = "Prior";

      _extract_plane_nodes(planes, nodes_vec,tfStampedBC);

      
      for (const auto room:rooms){
        situational_graphs_reasoning_msgs::msg::Edge graph_edge;
        situational_graphs_reasoning_msgs::msg::Node graph_node;
        situational_graphs_reasoning_msgs::msg::Attribute edge_attribute;
        situational_graphs_reasoning_msgs::msg::Attribute node_attribute;
        graph_node.id = room->getId();
        if(room->getRoomVariant()!=ORB_SLAM3::Room::roomVariant::UNDEFINED){
          graph_node.type = "Finite Room";
        }else{
          graph_node.type = "Infinite Room";
        }
        
        node_attribute.name = "Geometric_info";


        
        geometry_msgs::msg::PointStamped roomPoint, roomPointTr;
      
        roomPoint.point.x = room->getCentroid().x();
        roomPoint.point.y = room->getCentroid().y();
        roomPoint.point.z = room->getCentroid().z();
        try{
            tf2::doTransform(roomPoint, roomPointTr, tfStampedSE);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(rclcpp::get_logger("visual_sgraphs"), "Room  centroid transform failed during creation of MapInfo msg: %s", ex.what());
            roomPointTr = roomPoint;
        }          
        
        node_attribute.fl_value.push_back(roomPointTr.point.x);
        node_attribute.fl_value.push_back(roomPointTr.point.y);
        node_attribute.fl_value.push_back(0.0);

        node_att_vec.push_back(node_attribute);
        graph_node.attributes = node_att_vec;
        nodes_vec.push_back(graph_node);
        node_attribute.fl_value.clear();
        node_att_vec.clear();
        _extract_room_wall_edges(room, graph_edge, edge_attribute, edge_att_vec, edges_vec);
      }



    }else{
      graph_msg.name = "Online";
      _extract_plane_nodes(planes, nodes_vec,tfStampedBC);
      for (auto room : pCurrentMap->GetAllRooms()) {
        if (room->isBad()) continue; 

        situational_graphs_reasoning_msgs::msg::Edge graph_edge;
        situational_graphs_reasoning_msgs::msg::Node graph_node;
        situational_graphs_reasoning_msgs::msg::Attribute edge_attribute;
        situational_graphs_reasoning_msgs::msg::Attribute node_attribute;

        graph_node.id = room->getId();
        
        graph_node.type = "Finite Room";
        node_attribute.name = "Geometric_info";
    
        geometry_msgs::msg::PointStamped roomPoint, roomPointTr;
      
        roomPoint.point.x = room->getCentroid().x();
        roomPoint.point.y = room->getCentroid().y();
        roomPoint.point.z = room->getCentroid().z();
        try{
            tf2::doTransform(roomPoint, roomPointTr, tfStampedSE);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(rclcpp::get_logger("visual_sgraphs"), "Wall centroid transform failed during creation of MapInfo msg: %s", ex.what());
            roomPointTr = roomPoint;
        }          
        
        node_attribute.fl_value.push_back(roomPointTr.point.x);
        node_attribute.fl_value.push_back(roomPointTr.point.y);
        node_attribute.fl_value.push_back(0.0);




        node_att_vec.push_back(node_attribute);
        graph_node.attributes = node_att_vec;
        nodes_vec.push_back(graph_node);
        node_attribute.fl_value.clear();
        node_att_vec.clear();
        _extract_room_wall_edges(room, graph_edge, edge_attribute, edge_att_vec, edges_vec);

      }
    }
    graph_msg.edges = edges_vec;
    graph_msg.nodes = nodes_vec;
    edges_vec.clear();
    nodes_vec.clear();
    return graph_msg;
  }


} // namespace GraphPublisher