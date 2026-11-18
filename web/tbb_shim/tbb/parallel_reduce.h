#pragma once
#include <utility>
#include "blocked_range.h"
namespace tbb {
namespace detail_serial {
template <typename Range, typename Value, typename RealBody, typename Reduction>
Value deterministic(Range range, const Value& identity, const RealBody& body,
                    const Reduction& reduction) {
  if (range.empty()) return identity;
  if (!range.is_divisible()) return body(range, identity);
  Range right(range, split{});
  Value left_value = deterministic(range, identity, body, reduction);
  Value right_value = deterministic(right, identity, body, reduction);
  return reduction(left_value, right_value);
}
template <typename Range, typename Body>
void deterministic_body(Range range, Body& body) {
  if (range.empty()) return;
  if (!range.is_divisible()) {
    body(range);
    return;
  }
  Range right(range, split{});
  deterministic_body(range, body);
  Body right_body(body, split{});
  deterministic_body(right, right_body);
  body.join(right_body);
}
}  // namespace detail_serial
template <typename Range, typename Value, typename RealBody, typename Reduction>
Value parallel_reduce(const Range& range, const Value& identity, const RealBody& body,
                      const Reduction& reduction) {
  return range.empty() ? identity : body(range, identity);
}
template <typename Range, typename Value, typename RealBody, typename Reduction>
Value parallel_deterministic_reduce(const Range& range, const Value& identity,
                                    const RealBody& body, const Reduction& reduction) {
  return detail_serial::deterministic(range, identity, body, reduction);
}
template <typename Range, typename Body>
void parallel_deterministic_reduce(const Range& range, Body& body) {
  detail_serial::deterministic_body(range, body);
}
}  // namespace tbb
