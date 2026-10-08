# FAST-LIO2 and Point-LIO at the degenerate_lio_sota_v3 rival revisions, for the
# ENWIDE comparison (docs/research/enwide-photometric-registration-2026-10.md).
# Both need livox_ros_driver's messages even for Ouster input.
#
#   docker build -f docker/enwide_rivals_ros1.Dockerfile -t enwide-rivals-ros1 docker
#   scripts/run_enwide_rival_ros1.sh fast_lio /path/to/enwide/field_d OUTPUT_DIR
FROM ros:noetic-perception
SHELL ["/bin/bash", "-c"]
RUN apt-get update && apt-get install -y --no-install-recommends \
      git cmake build-essential libeigen3-dev libpcl-dev ros-noetic-pcl-ros \
      ros-noetic-eigen-conversions libgoogle-glog-dev && rm -rf /var/lib/apt/lists/*
RUN git clone https://github.com/Livox-SDK/Livox-SDK.git /opt/Livox-SDK && \
    cd /opt/Livox-SDK && git checkout 9306596a2bf15c1343bc023b497465ed0a32909d && \
    mkdir -p build && cd build && cmake .. && make -j8 && make install
RUN mkdir -p /root/rivals_ws/src && cd /root/rivals_ws/src && \
    git clone https://github.com/Livox-SDK/livox_ros_driver.git && \
    git -C livox_ros_driver checkout 3d240d5666129e1a3052e78ee8487a04b08fdda3 && \
    git clone https://github.com/hku-mars/FAST_LIO.git && \
    git -C FAST_LIO checkout 7cc4175de6f8ba2edf34bab02a42195b141027e9 && \
    git -C FAST_LIO submodule update --init --recursive && \
    git clone https://github.com/hku-mars/Point-LIO.git && \
    git -C Point-LIO checkout 4b86a469eb5572e70ed575af25b5f15dd06e8e3c
# The packages do not order their message generation; a second pass finishes it.
RUN source /opt/ros/noetic/setup.bash && cd /root/rivals_ws && \
    (catkin_make -DCMAKE_BUILD_TYPE=Release -j8 || catkin_make -DCMAKE_BUILD_TYPE=Release -j8)
