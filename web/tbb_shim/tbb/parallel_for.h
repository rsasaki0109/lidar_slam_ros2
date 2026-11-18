#pragma once
#include "blocked_range.h"
namespace tbb {
template <typename Range, typename Body>
void parallel_for(const Range& range, const Body& body) {
  if (!range.empty()) body(range);
}
}  // namespace tbb
