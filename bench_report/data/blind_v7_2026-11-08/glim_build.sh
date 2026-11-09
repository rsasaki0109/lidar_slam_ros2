#!/usr/bin/env bash
# GLIM CPU build from source at pinned tags (GTSAM 4.3a0, gtsam_points / glim / glim_ros2 v1.2.2), installed under
# /home/user/rivals/glim_install (nothing goes to the system).
set -euo pipefail
B=/home/user/rivals/glim_build; P=/home/user/rivals/glim_install; J=3
mkdir -p "$P"
cmake -S $B/gtsam -B $B/gtsam/build -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$P -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
  -DGTSAM_BUILD_TESTS=OFF -DGTSAM_WITH_TBB=OFF -DGTSAM_USE_SYSTEM_EIGEN=ON -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
  -DGTSAM_BUILD_PYTHON=OFF
cmake --build $B/gtsam/build -j$J && cmake --install $B/gtsam/build
cmake -S $B/gtsam_points -B $B/gtsam_points/build -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$P -DCMAKE_PREFIX_PATH=$P \
  -DBUILD_WITH_CUDA=OFF -DBUILD_WITH_MARCH_NATIVE=OFF
cmake --build $B/gtsam_points/build -j$J && cmake --install $B/gtsam_points/build
set +u; source /opt/ros/jazzy/setup.bash; set -u
cd $B/ws && CMAKE_PREFIX_PATH=$P:${CMAKE_PREFIX_PATH:-} LD_LIBRARY_PATH=$P/lib:${LD_LIBRARY_PATH:-} colcon build \
  --parallel-workers 1 --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_WITH_CUDA=OFF -DBUILD_WITH_VIEWER=OFF \
  -DBUILD_WITH_MARCH_NATIVE=OFF -DCMAKE_PREFIX_PATH=$P
echo BUILD_DONE
