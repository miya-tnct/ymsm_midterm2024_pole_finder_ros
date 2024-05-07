#ifndef YMSM_MIDTERM2024_PLANNNER_VEC_Y_COMPARE_H_
#define YMSM_MIDTERM2024_PLANNNER_VEC_Y_COMPARE_H_

#include "tf2/LinearMath/Vector3.h"


namespace ymsm_midterm2024_planner::pole_finder
{

class VecYCompare {
public:
  auto operator()(const tf2::Vector3 & v0, const tf2::Vector3 & v1) const
  {
    return v0.y() < v1.y();
  }
};

}

#endif