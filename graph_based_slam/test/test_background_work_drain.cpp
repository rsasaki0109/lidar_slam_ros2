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
//    copyright notice, this list of conditions and the following disclaimer
//    in the documentation and/or other materials provided with the
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

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <stdexcept>
#include <thread>

#include "graph_based_slam/background_work_drain.hpp"

using graphslam::scheduling::BackgroundWorkDrain;

namespace
{
// A drain cycle that reports when it starts and then blocks until released.
class GatedCycles
{
public:
  void enter()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    ++started_;
    cv_.notify_all();
    cv_.wait(lock, [this]() {return released_ >= started_;});
  }

  void waitStarted(int count)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait(lock, [this, count]() {return started_ >= count;});
  }

  void releaseAll()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    released_ = 1 << 20;
    cv_.notify_all();
  }

  int started()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return started_;
  }

private:
  std::mutex mutex_;
  std::condition_variable cv_;
  int started_ {0};
  int released_ {0};
};
}  // namespace

TEST(BackgroundWorkDrain, RequestReturnsWhileTheDrainRunsOnTheWorker)
{
  GatedCycles cycles;
  std::thread::id drain_thread;
  BackgroundWorkDrain drain(
    [&]() {
      drain_thread = std::this_thread::get_id();
      cycles.enter();
    });

  drain.request();
  cycles.waitStarted(1);

  EXPECT_NE(drain_thread, std::this_thread::get_id());
  cycles.releaseAll();
}

TEST(BackgroundWorkDrain, RequestsDuringADrainCoalesceIntoOneMoreCycle)
{
  GatedCycles cycles;
  BackgroundWorkDrain drain([&cycles]() {cycles.enter();});

  drain.request();
  cycles.waitStarted(1);
  drain.request();
  drain.request();
  drain.request();
  cycles.releaseAll();
  // runSerialized() runs behind the coalesced cycle when it is queued after it.
  cycles.waitStarted(2);
  ASSERT_TRUE(drain.runSerialized([]() {}));

  EXPECT_EQ(cycles.started(), 2);
}

TEST(BackgroundWorkDrain, SerializedTaskWaitsForTheRunningDrainAndNeverOverlapsIt)
{
  GatedCycles cycles;
  std::atomic<int> active {0};
  std::atomic<int> maximum_active {0};
  const auto enter = [&]() {
      const int now = ++active;
      int previous = maximum_active.load();
      while (now > previous && !maximum_active.compare_exchange_weak(previous, now)) {}
    };
  BackgroundWorkDrain drain(
    [&]() {
      enter();
      cycles.enter();
      --active;
    });

  drain.request();
  cycles.waitStarted(1);
  std::atomic<bool> task_ran {false};
  std::thread caller(
    [&]() {
      EXPECT_TRUE(
        drain.runSerialized(
          [&]() {
            enter();
            task_ran = true;
            --active;
          }));
      EXPECT_TRUE(task_ran.load());
    });
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  EXPECT_FALSE(task_ran.load());
  cycles.releaseAll();
  caller.join();

  EXPECT_TRUE(task_ran.load());
  EXPECT_EQ(maximum_active.load(), 1);
}

TEST(BackgroundWorkDrain, SerializedTaskRunsOnTheWorkerAndRethrowsItsException)
{
  BackgroundWorkDrain drain([]() {});
  std::thread::id task_thread;

  ASSERT_TRUE(drain.runSerialized([&task_thread]() {task_thread = std::this_thread::get_id();}));
  EXPECT_NE(task_thread, std::this_thread::get_id());
  EXPECT_THROW(
    drain.runSerialized([]() {throw std::runtime_error("task failed");}),
    std::runtime_error);
  EXPECT_TRUE(drain.runSerialized([]() {}));
}

TEST(BackgroundWorkDrain, DrainExceptionIsRethrownByTheNextRequestAndTheRequestRetries)
{
  std::atomic<int> attempts {0};
  BackgroundWorkDrain drain(
    [&attempts]() {
      if (++attempts == 1) {
        throw std::runtime_error("drain failed");
      }
    });

  drain.request();
  // Until the failing cycle has finished, a request only coalesces into the
  // next cycle; afterwards the first request reports the failure.
  bool rethrown = false;
  for (int i = 0; i < 1000000 && !rethrown; ++i) {
    try {
      drain.request();
    } catch (const std::runtime_error &) {
      rethrown = true;
    }
    std::this_thread::yield();
  }
  ASSERT_TRUE(rethrown);
  while (attempts.load() < 2) {
    std::this_thread::yield();
  }

  EXPECT_NO_THROW(drain.request());
  EXPECT_TRUE(drain.runSerialized([]() {}));
}

TEST(BackgroundWorkDrain, SerializedCallReportsARecordedDrainFailureOnce)
{
  std::atomic<int> attempts {0};
  BackgroundWorkDrain drain(
    [&attempts]() {
      if (++attempts == 1) {
        throw std::runtime_error("drain failed");
      }
    });

  drain.request();
  bool rethrown = false;
  for (int i = 0; i < 1000000 && !rethrown; ++i) {
    try {
      EXPECT_TRUE(drain.runSerialized([]() {}));
    } catch (const std::runtime_error &) {
      rethrown = true;
    }
  }
  ASSERT_TRUE(rethrown);

  EXPECT_TRUE(drain.runSerialized([]() {}));
}

TEST(BackgroundWorkDrain, StopLetsTheRunningDrainFinishAndDiscardsPendingWork)
{
  GatedCycles cycles;
  std::atomic<bool> saw_stop {false};
  BackgroundWorkDrain drain(
    [&]() {
      cycles.enter();
      saw_stop = drain.stopRequested();
    });

  drain.request();
  cycles.waitStarted(1);
  drain.request();
  std::thread stopper([&drain]() {drain.stop();});
  while (!drain.stopRequested()) {
    std::this_thread::yield();
  }
  cycles.releaseAll();
  stopper.join();

  EXPECT_TRUE(saw_stop.load());
  EXPECT_EQ(cycles.started(), 1);
  bool task_ran = false;
  EXPECT_FALSE(drain.runSerialized([&task_ran]() {task_ran = true;}));
  EXPECT_FALSE(task_ran);
  drain.request();
  EXPECT_EQ(cycles.started(), 1);
}
