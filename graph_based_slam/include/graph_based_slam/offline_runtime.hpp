// Copyright 2026 Sasaki
// All rights reserved.
//
// Software License Agreement (BSD 2-Clause Simplified License)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in
//    the documentation and/or other materials provided with the
//    distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// Parameters and logging of the offline backend without ROS.
//
// Parameters holds typed values by name, with the type rules of rclcpp::Parameter: a value is read
// only as its own type (an integer is never read as a double, nor a double as an integer), and
// integral and floating types take a cast of the stored int64 / double. The ROS 2 runner fills it
// from the node's parameter overrides and the Python package from a parameter file, so both read
// the same values the same way.

#ifndef GRAPH_BASED_SLAM__OFFLINE_RUNTIME_HPP_
#define GRAPH_BASED_SLAM__OFFLINE_RUNTIME_HPP_

#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <map>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

namespace graphslam
{
namespace offline
{

using ParameterValue = std::variant<
  bool, std::int64_t, double, std::string, std::vector<std::uint8_t>, std::vector<bool>,
  std::vector<std::int64_t>, std::vector<double>, std::vector<std::string>>;

class ParameterTypeError : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

inline const char * parameterTypeName(const ParameterValue & value)
{
  static const char * const kNames[] = {
    "bool", "integer", "double", "string", "byte_array", "bool_array", "integer_array",
    "double_array", "string_array"};
  return kNames[value.index()];
}

class Parameters
{
public:
  void set(const std::string & name, ParameterValue value)
  {
    values_[name] = std::move(value);
  }

  bool has(const std::string & name) const {return values_.count(name) != 0U;}

  const std::map<std::string, ParameterValue> & values() const {return values_;}

  // rclcpp::Node::get_parameter_or: the stored value when the parameter is set, else the
  // alternative. Returns whether the parameter was set; throws ParameterTypeError when its
  // stored type cannot be read as T.
  template<typename T>
  bool get_parameter_or(const std::string & name, T & value, const T & alternative) const
  {
    const auto it = values_.find(name);
    if (it == values_.end()) {
      value = alternative;
      return false;
    }
    value = read<T>(name, it->second);
    return true;
  }

private:
  template<typename Stored>
  static const Stored & expect(const std::string & name, const ParameterValue & value)
  {
    const Stored * stored = std::get_if<Stored>(&value);
    if (stored == nullptr) {
      throw ParameterTypeError(
              "parameter '" + name + "' has type " + parameterTypeName(value) +
              ", which cannot be read as the requested type");
    }
    return *stored;
  }

  template<typename T>
  static T read(const std::string & name, const ParameterValue & value)
  {
    if constexpr (std::is_same_v<T, bool>) {
      return expect<bool>(name, value);
    } else if constexpr (std::is_integral_v<T>) {
      return static_cast<T>(expect<std::int64_t>(name, value));
    } else if constexpr (std::is_floating_point_v<T>) {
      return static_cast<T>(expect<double>(name, value));
    } else if constexpr (std::is_same_v<T, std::string>) {
      return expect<std::string>(name, value);
    } else if constexpr (std::is_same_v<T, std::vector<std::uint8_t>>) {
      return expect<std::vector<std::uint8_t>>(name, value);
    } else if constexpr (std::is_same_v<T, std::vector<bool>>) {
      return expect<std::vector<bool>>(name, value);
    } else if constexpr (std::is_same_v<T, std::vector<std::string>>) {
      return expect<std::vector<std::string>>(name, value);
    } else if constexpr (std::is_same_v<T, std::vector<std::int64_t>>) {
      return expect<std::vector<std::int64_t>>(name, value);
    } else if constexpr (std::is_same_v<T, std::vector<double>>) {
      return expect<std::vector<double>>(name, value);
    } else if constexpr (std::is_same_v<T, std::vector<int>>) {
      const auto & stored = expect<std::vector<std::int64_t>>(name, value);
      return T(stored.begin(), stored.end());
    } else if constexpr (std::is_same_v<T, std::vector<float>>) {
      const auto & stored = expect<std::vector<double>>(name, value);
      return T(stored.begin(), stored.end());
    } else {
      static_assert(!sizeof(T), "unsupported parameter type");
    }
  }

  std::map<std::string, ParameterValue> values_;
};

// Where the backend's messages go: the ROS 2 runner logs through rclcpp, the Python package
// through its own handler.
class Log
{
public:
  virtual ~Log() = default;
  virtual void info(const std::string & message) const = 0;
  virtual void warn(const std::string & message) const = 0;
  virtual void error(const std::string & message) const = 0;
};

// Logs to stderr in rclcpp's console layout (no time stamp).
class StderrLog : public Log
{
public:
  explicit StderrLog(std::string name = "graph_based_slam")
  : name_(std::move(name)) {}
  void info(const std::string & message) const override {write("INFO", message);}
  void warn(const std::string & message) const override {write("WARN", message);}
  void error(const std::string & message) const override {write("ERROR", message);}

private:
  void write(const char * level, const std::string & message) const
  {
    std::fprintf(stderr, "[%s] [%s]: %s\n", level, name_.c_str(), message.c_str());
  }
  std::string name_;
};

#if defined(__GNUC__)
__attribute__((format(printf, 1, 2)))
#endif
inline std::string formatLog(const char * format, ...)
{
  va_list args;
  va_start(args, format);
  va_list copy;
  va_copy(copy, args);
  const int size = std::vsnprintf(nullptr, 0, format, copy);
  va_end(copy);
  std::string text;
  if (size > 0) {
    text.resize(static_cast<std::size_t>(size) + 1U);
    std::vsnprintf(text.data(), text.size(), format, args);
    text.resize(static_cast<std::size_t>(size));
  }
  va_end(args);
  return text;
}

}  // namespace offline
}  // namespace graphslam

#define GRAPHSLAM_LOG_INFO(log, ...) (log).info(::graphslam::offline::formatLog(__VA_ARGS__))
#define GRAPHSLAM_LOG_WARN(log, ...) (log).warn(::graphslam::offline::formatLog(__VA_ARGS__))
#define GRAPHSLAM_LOG_ERROR(log, ...) (log).error(::graphslam::offline::formatLog(__VA_ARGS__))

#endif  // GRAPH_BASED_SLAM__OFFLINE_RUNTIME_HPP_
