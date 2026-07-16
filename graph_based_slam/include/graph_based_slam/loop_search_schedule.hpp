// Copyright 2026 Sasaki
// All rights reserved.
//
// Software License Agreement (BSD 2-Clause Simplified License)

#ifndef GRAPH_BASED_SLAM__LOOP_SEARCH_SCHEDULE_HPP_
#define GRAPH_BASED_SLAM__LOOP_SEARCH_SCHEDULE_HPP_

namespace graphslam::loop_search_schedule
{

inline bool shouldSearch(int zero_based_query_index, int query_stride)
{
  if (zero_based_query_index < 0) {
    return false;
  }
  const int safe_stride = query_stride < 1 ? 1 : query_stride;
  return safe_stride == 1 || ((zero_based_query_index + 1) % safe_stride) == 0;
}

}  // namespace graphslam::loop_search_schedule

#endif  // GRAPH_BASED_SLAM__LOOP_SEARCH_SCHEDULE_HPP_
