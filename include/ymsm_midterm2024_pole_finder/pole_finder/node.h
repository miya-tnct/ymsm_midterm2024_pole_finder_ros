#ifndef YMSM_MIDTERM2024_PLANNNER_POLE_FINDER_NODE_H_
#define YMSM_MIDTERM2024_PLANNNER_POLE_FINDER_NODE_H_

#include "geometry_msgs/PointStamped.h"
#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/LaserScan.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"


namespace ymsm_midterm2024_planner::pole_finder
{

class Node : public ros::NodeHandle
{
public:
  Node();

private:
  bool is_in_map(const tf2::Vector3 & point);

  void initialize_map(nav_msgs::OccupancyGrid::ConstPtr map_msg);

  void update_map(nav_msgs::OccupancyGrid::ConstPtr map_msg);

  void convert(const sensor_msgs::LaserScan::ConstPtr& scan_msg);

  double pole_diameter_;
  double range_min_;

  geometry_msgs::PointStamped pole_msg_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;
  ros::Publisher pole_left_publisher_, pole_right_publisher_;
  ros::Subscriber map_subscriber_;

  // ここからはmap受診時に初期化
  nav_msgs::OccupancyGrid::ConstPtr map_msg_;
  tf2::Transform map_origin_, map_origin_inv_;
  tf2::Vector3 map_point_max_;

  ros::Subscriber scan_subscriber_;
};

}

#endif