#pragma once

#include <iostream>
#include <fstream>
#include <Eigen/Core>

namespace pciof {
template <typename T>
using Vector3 = Eigen::Matrix<T, 3, 1>;

template <typename T>
std::vector<Vector3<T>> read_pcd(const std::string& path) {
  std::ifstream file(path, std::ios::binary);
  if (!file.is_open()) {
    std::cout << "Failed to open file: " << path << std::endl;
    return {};
  }

  int points_size = 0;
  Eigen::Vector3i xyz_cols = Eigen::Vector3i::Zero();

  std::vector<std::string> fields;
  std::vector<size_t> sizes;
  std::vector<std::string> types;
  std::vector<size_t> counts;

  // Skip the first 11 lines
  for (int i = 0; i < 11; ++i) {
    std::string line;
    std::getline(file, line);

    size_t found = line.find("POINTS");
    if (found != std::string::npos) {
      // Extract the number after "POINTS"
      points_size = std::stoi(line.substr(found + 7));
    }

    size_t found1 = line.find("FIELDS");
    if (found1 != std::string::npos) {
      std::istringstream iss(line);
      std::string field;
      while (iss >> field) {
        if (field != "FIELDS" && field != "SIZE" && field != "TYPE" && field != "COUNT") {
          fields.emplace_back(field);
        }
      }

      for (int j = 0; j < fields.size(); ++j) {
        if (fields[j] == "x")
          xyz_cols.x() = j;
        else if (fields[j] == "y")
          xyz_cols.y() = j;
        else if (fields[j] == "z")
          xyz_cols.z() = j;
      }
    }

    size_t found2 = line.find("SIZE");
    if (found2 != std::string::npos) {
      std::istringstream iss(line);
      std::string size;
      while (iss >> size) {
        if (size != "SIZE" && size != "TYPE" && size != "COUNT") {
          sizes.emplace_back(std::stoi(size));
        }
      }
    }

    size_t found3 = line.find("TYPE");
    if (found3 != std::string::npos) {
      std::istringstream iss(line);
      std::string type;
      while (iss >> type) {
        if (type != "TYPE" && type != "COUNT") {
          types.emplace_back(type);
        }
      }
    }

    size_t found4 = line.find("COUNT");
    if (found4 != std::string::npos) {
      std::istringstream iss(line);
      std::string count;
      while (iss >> count) {
        if (count != "COUNT") {
          counts.emplace_back(std::stoi(count));
        }
      }
    }
  }

  std::vector<Vector3<T>> point_cloud;
  point_cloud.reserve(points_size);

  // check if T and x y z sizez are the same
  if (sizeof(T) != sizes[xyz_cols.x()] || sizeof(T) != sizes[xyz_cols.y()] || sizeof(T) != sizes[xyz_cols.z()]) {
    std::cout << "T and x y z sizez are not the same" << std::endl;
    return {};
  }

  size_t ignore_size_before_xyz = 0;
  for (int i = 0; i < xyz_cols.x(); ++i) {
    size_t count_temp = counts[i];
    ignore_size_before_xyz += sizes[i] * count_temp;
  }

  size_t ignore_size_after_xyz = 0;
  for (int i = xyz_cols.z() + 1; i < sizes.size(); ++i) {
    size_t count_temp = counts[i];
    ignore_size_after_xyz += sizes[i] * count_temp;
  }

  T x, y, z;
  for (int i = 0; i < points_size; ++i) {
    file.ignore(ignore_size_before_xyz);
    file.read(reinterpret_cast<char*>(&x), sizeof(T));
    file.read(reinterpret_cast<char*>(&y), sizeof(T));
    file.read(reinterpret_cast<char*>(&z), sizeof(T));
    file.ignore(ignore_size_after_xyz);

    Vector3<T> point(x, y, z);
    point_cloud.emplace_back(point);
  }
  file.close();

  return point_cloud;
}

template <typename T>
bool save_pcd(const std::string& path, const std::vector<Vector3<T>>& point_cloud) {
  std::ofstream file(path, std::ios::binary);
  if (!file.is_open()) {
    std::cout << "Failed to create file: " << path << std::endl;
    return false;
  }

  // Write PCD header
  file << "# .PCD v.7 - Point Cloud Data file format\n";
  file << "VERSION .7\n";
  file << "FIELDS x y z\n";
  file << "SIZE " << sizeof(T) << " " << sizeof(T) << " " << sizeof(T) << "\n";

  if (std::is_floating_point<T>::value) {
    file << "TYPE F F F\n";
  } else if (std::is_signed<T>::value) {
    file << "TYPE I I I\n";
  } else if (std::is_same<T, double>::value) {
    file << "TYPE D D D\n";
  } else {
    file << "TYPE U U U\n";
  }

  file << "COUNT 1 1 1\n";
  file << "WIDTH " << point_cloud.size() << "\n";
  file << "HEIGHT 1\n";
  file << "VIEWPOINT 0 0 0 1 0 0 0\n";
  file << "POINTS " << point_cloud.size() << "\n";
  file << "DATA binary\n";

  for (const auto& point : point_cloud) {
    file.write(reinterpret_cast<const char*>(&point[0]), sizeof(T));
    file.write(reinterpret_cast<const char*>(&point[1]), sizeof(T));
    file.write(reinterpret_cast<const char*>(&point[2]), sizeof(T));
  }

  file.close();

  return true;
}

}  // namespace pciof
