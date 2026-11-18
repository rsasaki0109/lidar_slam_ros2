#pragma once
namespace tbb {
namespace this_task_arena {
inline int max_concurrency() { return 1; }
}  // namespace this_task_arena
}  // namespace tbb
