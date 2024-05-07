#include "ymsm_midterm2024_pole_finder/pole_finder/node.h"

#include <limits>

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace ymsm_midterm2024_planner::pole_finder
{

Node::Node() :
  ros::NodeHandle(),
  pnh_("~"),
  range_min_(pnh_.param("range_min", 0.8)),
  cluster_threshold_(pnh_.param("cluster_threshold", 0.3)),
  cluster_threshold2_(cluster_threshold_ * cluster_threshold_),
  pole_diameter_(pnh_.param("pole_diameter", 0.1)),
  pole_msg_(),
  tf_buffer_(),
  tf_listener_(tf_buffer_),
  static_tf_broadcaster_(),
  pole_left_publisher_(this->advertise<geometry_msgs::PointStamped>("pole_left", 1, true)),
  pole_right_publisher_(this->advertise<geometry_msgs::PointStamped>("pole_right", 1, true)),
  map_subscriber_(this->subscribe("map", 1, &Node::initialize_map, this))
{
}

bool Node::is_in_map(const tf2::Vector3 & point)
{
  return 0 <= point.x() && point.x() < map_point_max_.x()
    && 0 <= point.y() && point.y() < map_point_max_.y();
}

void Node::initialize_map(nav_msgs::OccupancyGrid::ConstPtr map_msg)
{
  this->update_map(std::move(map_msg));
  map_subscriber_ = this->subscribe("map", 1, &Node::update_map, this);
  scan_subscriber_ = this->subscribe("scan", 1, &Node::convert, this);
}

void Node::update_map(nav_msgs::OccupancyGrid::ConstPtr map_msg)
{
  map_msg_ = std::move(map_msg);
  tf2::fromMsg(map_msg_->info.origin, map_origin_);
  map_origin_inv_ = map_origin_.inverse();
  map_point_max_ = tf2::Vector3(
    map_msg_->info.resolution * map_msg_->info.width,
    map_msg_->info.resolution * map_msg_->info.height,
    std::numeric_limits<double>::infinity()
  );
  pole_msg_.header.frame_id = map_msg->header.frame_id;
}

void Node::convert(
  const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
  geometry_msgs::TransformStamped scanner_tf_msg;
  try {
    scanner_tf_msg = tf_buffer_.lookupTransform(
      pole_msg_.header.frame_id, scan_msg->header.frame_id, ros::Time(0));
  }
  catch (...) {
    return;
  }
  tf2::Transform scanner_tf;
  tf2::fromMsg(scanner_tf_msg.transform, scanner_tf);
  scanner_tf = map_origin_inv_ * scanner_tf;

  // scanからpointに変換する
  // このときに近すぎる値とNaNとmap外の値は無視する
  std::vector<tf2::Vector3> points;
  auto angle_min_harf = 0.5 * scan_msg->angle_min;
  auto laser_tf =  tf2::Transform(scanner_tf.getRotation() * tf2::Quaternion(0, 0, std::sin(angle_min_harf), std::cos(angle_min_harf)), scanner_tf.getOrigin());
  auto angle_increment_harf = 0.5 * scan_msg->angle_increment;
  auto laser_increment_quat = tf2::Quaternion(0, 0, std::sin(angle_increment_harf), std::cos(angle_increment_harf));
  for (const auto & range : scan_msg->ranges) {
    if (std::isnormal(range) && range >= range_min_) {
      auto point = laser_tf * tf2::Vector3(range, 0, 0);
      if (this->is_in_map(point)) {
        points.emplace_back(std::move(point));
      }
    }
    laser_tf.setRotation(laser_tf.getRotation() * laser_increment_quat);
  }

  // 距離が一定の範囲内の値をpoint_clusterとして、その集合point_clustersを作成する
  std::vector<std::vector<tf2::Vector3>> point_clusters;
  std::vector<tf2::Vector3> point_cluster;
  auto point_last = tf2::Vector3(0, 0, 0);
  for (const auto & point : points) {
    if (point.distance2(point_last) > cluster_threshold2_) {
      point_clusters.push_back(point_cluster);
      point_cluster.clear();
    }

    point_cluster.push_back(point);
    point_last = point;
  }
  point_clusters.push_back(point_cluster);

  // point_clusterの端のpointの距離とポール直径二乗誤差をキーとしてpoleのpoint集合を作る
  // 要素数0のclusterは無視する
  std::multimap<double, tf2::Vector3> poles;
  for (const auto & point_cluster : point_clusters) {
    if (point_cluster.empty()) {
      continue;
    }

    auto dist = point_cluster.front().distance(point_cluster.back());
    auto error = dist - pole_diameter_;
    auto error2 = error * error;
    if (error2 <= 0.1 * 0.1) {
      poles.emplace(error * error, 0.5 * (point_cluster.front() + point_cluster.back()));
    }
  }

  if(poles.size() < 2) {
    return;
  }
  
  // y座標をもとにpublish
  auto pole_itr = poles.begin();
  auto pole_0 = std::next(pole_itr, 0);
  auto pole_1 = std::next(pole_itr, 1);
  auto to_msg = [](auto vec3) {
    geometry_msgs::Point point;
    point.x = vec3.x();
    point.y = vec3.y();
    point.z = vec3.z();
    return point;
  };
  pole_msg_.header.stamp = ros::Time::now();
  if (pole_0->second.y() >= pole_1->second.y()) {
     pole_msg_.point = to_msg(map_origin_ * pole_0->second);
     pole_left_publisher_.publish(pole_msg_);
     pole_msg_.point = to_msg(map_origin_ * pole_1->second);
     pole_right_publisher_.publish(pole_msg_);
  }
  else {
     pole_msg_.point = to_msg(map_origin_ * pole_1->second);
     pole_left_publisher_.publish(pole_msg_);
     pole_msg_.point = to_msg(map_origin_ * pole_0->second);
     pole_right_publisher_.publish(pole_msg_);
  }
  ++pole_msg_.header.seq;
}

}