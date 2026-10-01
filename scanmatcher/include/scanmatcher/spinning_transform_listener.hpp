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
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
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

#ifndef SCANMATCHER_SPINNING_TRANSFORM_LISTENER_HPP_
#define SCANMATCHER_SPINNING_TRANSFORM_LISTENER_HPP_

#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace graphslam
{

// TransformListener on its own node and executor, spun by a thread that polls a
// stop flag.  tf2_ros' built-in dedicated thread stops by Executor::cancel();
// when the listener is destroyed right after construction, that cancel can be
// issued before spin() starts and is then lost, so the destructor's join blocks
// until an unrelated DDS event wakes the executor, or forever.  Polling a flag
// with spin_once() cannot lose the stop request.
class SpinningTransformListener
{
public:
  SpinningTransformListener(tf2_ros::Buffer & buffer, rclcpp::Node & owner)
  : context_(owner.get_node_base_interface()->get_context())
  {
    rclcpp::NodeOptions options;
    // Same as tf2_ros' dedicated node: keep global remaps (e.g. of /tf) but
    // give the helper node its own name.
    options.context(context_)
    .start_parameter_services(false)
    .start_parameter_event_publisher(false)
    .arguments({"--ros-args", "-r", "__node:=" + std::string(owner.get_name()) + "_tf_listener"});
    node_ = std::make_shared<rclcpp::Node>(
      std::string(owner.get_name()) + "_tf_listener", owner.get_namespace(), options);
    listener_ = std::make_unique<tf2_ros::TransformListener>(buffer, node_, false);
    rclcpp::ExecutorOptions executor_options;
    executor_options.context = context_;
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(executor_options);
    executor_->add_node(node_);
    thread_ = std::thread([this]() {
        while (running_.load() && context_->is_valid()) {
          try {
            executor_->spin_once(std::chrono::milliseconds(50));
          } catch (const std::exception &) {
            break;  // context shut down underneath us
          }
        }
      });
  }

  ~SpinningTransformListener()
  {
    running_.store(false);
    if (thread_.joinable()) {
      thread_.join();
    }
    executor_->remove_node(node_);
  }

  SpinningTransformListener(const SpinningTransformListener &) = delete;
  SpinningTransformListener & operator=(const SpinningTransformListener &) = delete;

private:
  rclcpp::Context::SharedPtr context_;
  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<tf2_ros::TransformListener> listener_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::atomic<bool> running_{true};
  std::thread thread_;
};

}  // namespace graphslam

#endif  // SCANMATCHER_SPINNING_TRANSFORM_LISTENER_HPP_
