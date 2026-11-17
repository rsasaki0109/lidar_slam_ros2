// The few ROS 2 types and helpers velodyne_pointcloud's RawData and Calibration use, without ROS 2.
//
// lidarslam: the vendored rawdata.cpp and calibration.cpp keep their arithmetic unchanged; this
// header stands in for rclcpp::Time / Duration (the same nanosecond difference and the same
// double seconds), velodyne_msgs/VelodynePacket, angles::from_degrees, the RCLCPP logging macros
// (dropped) and the scalar conversions of yaml-cpp (std::stringstream, as yaml-cpp 0.8 does).
// Calibration files are read by the caller into a CalibrationDocument of scalar strings.

#ifndef VELODYNE_POINTCLOUD__ROS_FREE_HPP_
#define VELODYNE_POINTCLOUD__ROS_FREE_HPP_

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <ios>
#include <limits>
#include <locale>
#include <map>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace builtin_interfaces::msg
{
struct Time
{
  int32_t sec{0};
  uint32_t nanosec{0};
};
}  // namespace builtin_interfaces::msg

namespace velodyne_msgs::msg
{
struct VelodynePacket
{
  builtin_interfaces::msg::Time stamp;
  std::array<uint8_t, 1206> data{};
};
}  // namespace velodyne_msgs::msg

namespace rclcpp
{
class Duration
{
public:
  explicit Duration(int64_t nanoseconds)
  : nanoseconds_(nanoseconds) {}
  // as rclcpp::Duration::seconds()
  double seconds() const
  {
    return std::chrono::duration<double>(std::chrono::nanoseconds(nanoseconds_)).count();
  }

private:
  int64_t nanoseconds_;
};

class Time
{
public:
  // as rclcpp::Time(const builtin_interfaces::msg::Time &): sec * 1e9 + nanosec
  Time(const builtin_interfaces::msg::Time & stamp)  // NOLINT(runtime/explicit)
  : nanoseconds_(static_cast<int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec) {}
  int64_t nanoseconds() const {return nanoseconds_;}
  Duration operator-(const Time & rhs) const {return Duration(nanoseconds_ - rhs.nanoseconds_);}

private:
  int64_t nanoseconds_;
};

struct Clock {};
}  // namespace rclcpp

// velodyne_pointcloud only logs through these; the package does not print from the decoder.
#define RCLCPP_DEBUG(...) ((void)0)
#define RCLCPP_INFO(...) ((void)0)
#define RCLCPP_WARN(...) ((void)0)
#define RCLCPP_WARN_STREAM_THROTTLE(...) ((void)0)

namespace angles
{
// angles a96224f9: degrees * M_PI / 180.0
static inline double from_degrees(double degrees) {return degrees * M_PI / 180.0;}
}  // namespace angles

namespace velodyne_pointcloud
{
/// A calibration file read into scalar strings: top-level keys and one map per laser.
struct CalibrationDocument
{
  std::map<std::string, std::string> scalars;
  std::vector<std::map<std::string, std::string>> lasers;
};

namespace yaml_scalar
{
// yaml-cpp 0.8 convert<T>::decode for numbers: a classic-locale stringstream with the base
// prefixes accepted (unsetf(dec)), the whole scalar consumed, and .inf / .nan spelled the YAML way.
template<typename T>
T as(const std::string & input, const std::string & key)
{
  std::stringstream stream(input);
  stream.imbue(std::locale::classic());
  stream.unsetf(std::ios::dec);
  T value{};
  if ((stream >> std::noskipws >> value) && (stream >> std::ws).eof()) {
    return value;
  }
  if constexpr (std::numeric_limits<T>::has_infinity) {
    if (input == ".inf" || input == ".Inf" || input == ".INF" || input == "+.inf" ||
      input == "+.Inf" || input == "+.INF")
    {
      return std::numeric_limits<T>::infinity();
    }
    if (input == "-.inf" || input == "-.Inf" || input == "-.INF") {
      return -std::numeric_limits<T>::infinity();
    }
  }
  if constexpr (std::numeric_limits<T>::has_quiet_NaN) {
    if (input == ".nan" || input == ".NaN" || input == ".NAN") {
      return std::numeric_limits<T>::quiet_NaN();
    }
  }
  throw std::runtime_error("calibration: bad value for " + key + ": " + input);
}

// yaml-cpp 0.8 convert<bool>::decode
inline bool as_bool(const std::string & input, const std::string & key)
{
  static const char * const true_names[] = {"y", "Y", "yes", "Yes", "YES", "true", "True",
    "TRUE", "on", "On", "ON"};
  static const char * const false_names[] = {"n", "N", "no", "No", "NO", "false", "False",
    "FALSE", "off", "Off", "OFF"};
  for (const char * name : true_names) {
    if (input == name) {return true;}
  }
  for (const char * name : false_names) {
    if (input == name) {return false;}
  }
  throw std::runtime_error("calibration: bad value for " + key + ": " + input);
}

inline std::optional<std::string> find(
  const std::map<std::string, std::string> & node, const char * key)
{
  const auto it = node.find(key);
  if (it == node.end()) {
    return std::nullopt;
  }
  return it->second;
}

inline const std::string & require(
  const std::map<std::string, std::string> & node, const char * key)
{
  const auto it = node.find(key);
  if (it == node.end()) {
    throw std::runtime_error(std::string("calibration: missing ") + key);
  }
  return it->second;
}
}  // namespace yaml_scalar
}  // namespace velodyne_pointcloud

#endif  // VELODYNE_POINTCLOUD__ROS_FREE_HPP_
