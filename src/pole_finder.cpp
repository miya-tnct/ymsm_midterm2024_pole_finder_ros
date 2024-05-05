#include "ymsm_midterm2024_pole_finder/pole_finder/node.h"

int main(int argc, char **argv)
{
  //ノードの初期化
  ros::init(argc, argv, "pole_finder");
  ymsm_midterm2024_planner::pole_finder::Node node;
  ros::spin();
  return 0;
}