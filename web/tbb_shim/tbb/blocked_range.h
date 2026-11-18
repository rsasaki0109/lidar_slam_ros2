// Serial stand-in for the oneTBB headers RKO-LIO uses, for single-threaded
// WebAssembly builds. Splitting follows oneTBB's blocked_range (halve while the
// size exceeds the grain), so deterministic reductions combine in the same tree.
#pragma once
#include <cstddef>
#include <iterator>
namespace tbb {
struct split {};
template <typename Value>
class blocked_range {
 public:
  using const_iterator = Value;
  using size_type = std::size_t;
  blocked_range(Value begin, Value end, size_type grainsize = 1)
      : end_(end), begin_(begin), grainsize_(grainsize) {}
  // end_ is declared (and so initialised) before begin_: do_split shortens r first.
  blocked_range(blocked_range& r, split)
      : end_(r.end_), begin_(do_split(r)), grainsize_(r.grainsize_) {}
  Value begin() const { return begin_; }
  Value end() const { return end_; }
  size_type size() const { return size_type(end_ - begin_); }
  size_type grainsize() const { return grainsize_; }
  bool empty() const { return !(begin_ < end_); }
  bool is_divisible() const { return grainsize_ < size(); }

 private:
  static Value do_split(blocked_range& r) {
    Value middle = r.begin_ + (r.end_ - r.begin_) / 2u;
    r.end_ = middle;
    return middle;
  }
  Value end_;
  Value begin_;
  size_type grainsize_;
};
}  // namespace tbb
