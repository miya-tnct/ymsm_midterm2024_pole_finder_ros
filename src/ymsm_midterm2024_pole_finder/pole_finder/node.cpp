#include "ymsm_midterm2024_pole_finder/pole_finder/node.h"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <map>
#include <set>
#include <utility>
#include <vector>

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "ymsm_midterm2024_pole_finder/pole_finder/vec_y_compare.h"

namespace ymsm_midterm2024_planner::pole_finder
{

Node::Node() :
  ros::NodeHandle(),
  pnh_("~"),
  range_min_(pnh_.param("range_min", 0.8)),
  cluster_threshold2_(square(pnh_.param("cluster_threshold", 0.3))),
  pole_diameter_(pnh_.param("pole_diameter", 0.1)),
  pole_diameter_error_threshold2_(square(pnh_.param("pole_diameter_error_threshold", 0.1))),
  pole_msg_(),
  tf_buffer_(),
  tf_listener_(tf_buffer_),
  pole_publishers_({
    this->advertise<geometry_msgs::PointStamped>("pole_right", 1, true),
    this->advertise<geometry_msgs::PointStamped>("pole_left", 1, true)
  }),
  map_subscriber_(this->subscribe("map", 1, &Node::initialize_map, this))
{
}

bool Node::is_in_map(const tf2::Vector3 & vec)
{
  return 0 <= vec.x() && vec.x() < map_max_vec_.x()
    && 0 <= vec.y() && vec.y() < map_max_vec_.y();
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
  tf2::fromMsg(map_msg_->info.origin, map_origin_tf_);
  map_origin_inv_tf_ = map_origin_tf_.inverse();
  map_max_vec_ = tf2::Vector3(
    map_msg_->info.resolution * map_msg_->info.width,
    map_msg_->info.resolution * map_msg_->info.height,
    std::numeric_limits<double>::infinity()
  );
  pole_msg_.header.frame_id = map_msg->header.frame_id;
}

void Node::convert(
  const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
  geometry_msgs::TransformStamped scanner_from_map_tf_msg;
  try {
    scanner_from_map_tf_msg = tf_buffer_.lookupTransform(
      pole_msg_.header.frame_id, scan_msg->header.frame_id, ros::Time(0));
  }
  catch (...) {
    return;
  }
  tf2::Transform scanner_from_map_tf;
  tf2::fromMsg(scanner_from_map_tf_msg.transform, scanner_from_map_tf);
  const auto scanner_tf = map_origin_inv_tf_ * scanner_from_map_tf; // map(frame_idではなく地図の)の原点

  // scanからpointに変換する
  // このときに近すぎる値とNaNとmap外の値は無視する
  std::vector<tf2::Vector3> scan_vecs;
  auto angle_min_harf = 0.5 * scan_msg->angle_min;
  auto laser_tf = tf2::Transform(scanner_tf.getRotation() * tf2::Quaternion(0, 0, std::sin(angle_min_harf), std::cos(angle_min_harf)), scanner_tf.getOrigin());
  auto angle_increment_harf = 0.5 * scan_msg->angle_increment;
  auto laser_increment_quat = tf2::Quaternion(0, 0, std::sin(angle_increment_harf), std::cos(angle_increment_harf));
  for (const auto & range : scan_msg->ranges) {
    if (std::isnormal(range) && range >= range_min_) {
      auto vec = laser_tf * tf2::Vector3(range, 0, 0);
      if (this->is_in_map(vec)) {
        scan_vecs.emplace_back(std::move(vec));
      }
    }
    laser_tf.setRotation(laser_tf.getRotation() * laser_increment_quat);
  }

  // 距離が一定の範囲内の値をpoint_clusterとして、その集合point_clustersを作成する
  std::vector<std::vector<tf2::Vector3>> point_clusters;
  std::vector<tf2::Vector3> point_cluster;
  auto scan_vec_last = tf2::Vector3(0, 0, 0);
  for (const auto & vec : scan_vecs) {
    if (vec.distance2(scan_vec_last) > cluster_threshold2_) {
      point_clusters.push_back(point_cluster);
      point_cluster.clear();
    }

    point_cluster.push_back(vec);
    scan_vec_last = vec;
  }
  point_clusters.push_back(point_cluster);

  // point_clusterの端のpointの距離とポール直径二乗誤差をキーとしてpoleのpoint集合を作る
  // 要素数0のclusterは無視する
  std::multimap<double, tf2::Vector3> pole_vecs;
  for (const auto & point_cluster : point_clusters) {
    if (point_cluster.empty()) {
      continue;
    }

    auto dist = point_cluster.front().distance(point_cluster.back());
    auto error = dist - pole_diameter_;
    auto error2 = error * error;
    if (error2 <= pole_diameter_error_threshold2_) {
      pole_vecs.emplace(error * error, 0.5 * (point_cluster.front() + point_cluster.back()));
    }
  }

  // 見つかったポールの数が少なかったら終了
  if(pole_vecs.size() < pole_publishers_.size()) {
    return;
  }
  
  // poleをy座標をもとにソート
  std::set<tf2::Vector3, VecYCompare> pole_set;
  std::for_each(pole_vecs.begin(), std::next(pole_vecs.begin(), pole_publishers_.size()), 
    [&pole_set](const auto & pole_vec_error_pair) {
      pole_set.emplace(pole_vec_error_pair.second);
    });
  
  // publish
  auto publisher_itr = pole_publishers_.begin();
  for (auto & pole : pole_set) {
    pole_msg_.point.x = pole.x();
    pole_msg_.point.y = pole.y();
    pole_msg_.point.z = pole.z();
    publisher_itr->publish(pole_msg_);
  }
  ++pole_msg_.header.seq;
}

}