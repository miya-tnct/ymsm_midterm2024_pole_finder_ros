#ifndef YMSM_MIDTERM2024_PLANNNER_POLE_FINDER_NODE_H_
#define YMSM_MIDTERM2024_PLANNNER_POLE_FINDER_NODE_H_

#include <array>

#include "geometry_msgs/PointStamped.h"
#include "ros/node_handle.h"
#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/LaserScan.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"


namespace ymsm_midterm2024_planner::pole_finder
{

class Node : public ros::NodeHandle
{
public:
  Node();

private:
  static auto square(double val) { return val * val; }

  bool is_in_map(const tf2::Vector3 & vec);

  void initialize_map(nav_msgs::OccupancyGrid::ConstPtr map_msg);

  void update_map(nav_msgs::OccupancyGrid::ConstPtr map_msg);

  void convert(const sensor_msgs::LaserScan::ConstPtr& scan_msg);

  ros::NodeHandle pnh_;

  double range_min_;
  double cluster_threshold_distance2_;
  double pole_diameter_, pole_diameter_error_threshold2_;

  geometry_msgs::PointStamped pole_msg_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::array<ros::Publisher, 2> pole_publishers_;
  ros::Subscriber map_subscriber_;

  // ここからはmap受診時に初期化
  nav_msgs::OccupancyGrid::ConstPtr map_msg_;
  tf2::Transform map_origin_tf_, map_origin_inv_tf_;
  tf2::Vector3 map_max_vec_;

  ros::Subscriber scan_subscriber_;
};

}

#endif