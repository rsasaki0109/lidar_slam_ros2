#pragma once
#include <cstddef>
namespace tbb {
class global_control {
 public:
  enum parameter { max_allowed_parallelism, thread_stack_size };
  global_control(parameter, std::size_t) {}
};
}  // namespace tbb
