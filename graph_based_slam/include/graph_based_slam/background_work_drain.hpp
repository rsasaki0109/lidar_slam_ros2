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

#ifndef GRAPH_BASED_SLAM__BACKGROUND_WORK_DRAIN_HPP_
#define GRAPH_BASED_SLAM__BACKGROUND_WORK_DRAIN_HPP_

#include <atomic>
#include <condition_variable>
#include <deque>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <utility>

namespace graphslam
{
namespace scheduling
{

// Runs one drain operation on a dedicated worker thread. request() only marks
// a drain pending and returns, so the requesting callback never waits for the
// drain; requests that arrive while a drain runs coalesce into one more cycle.
// runSerialized() runs a task on the same worker, never concurrently with the
// drain, and blocks its caller until the task has returned. Queued tasks run
// before a pending drain.
//
// The drain and the tasks only ever run on the worker thread, so the state they
// share needs no further synchronization. An exception thrown by the drain is
// rethrown by the next request() or runSerialized() call (the drain stays
// requestable, so a later request retries); an exception thrown by a task is
// rethrown by its own runSerialized() call.
//
// stop() and the destructor let a running drain or task finish, discard
// pending work and join the worker. A long drain should poll stopRequested().
// Neither stop() nor runSerialized() may be called from the worker itself, and
// stop() must not be called concurrently with itself.
class BackgroundWorkDrain
{
public:
  explicit BackgroundWorkDrain(std::function<void()> drain)
  : drain_(std::move(drain)),
    worker_([this]() {run();})
  {
  }

  ~BackgroundWorkDrain()
  {
    stop();
  }

  BackgroundWorkDrain(const BackgroundWorkDrain &) = delete;
  BackgroundWorkDrain & operator=(const BackgroundWorkDrain &) = delete;

  void request()
  {
    std::exception_ptr drain_error;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!stopping_) {
        drain_pending_ = true;
      }
      std::swap(drain_error, drain_error_);
    }
    work_cv_.notify_one();
    if (drain_error) {
      std::rethrow_exception(drain_error);
    }
  }

  // Returns false without running the task once the worker is stopping,
  // including when it starts stopping while the task is still queued.
  bool runSerialized(std::function<void()> task)
  {
    const auto job = std::make_shared<Job>();
    job->task = std::move(task);
    {
      std::unique_lock<std::mutex> lock(mutex_);
      if (drain_error_) {
        std::exception_ptr drain_error;
        std::swap(drain_error, drain_error_);
        std::rethrow_exception(drain_error);
      }
      if (stopping_) {
        return false;
      }
      jobs_.push_back(job);
      work_cv_.notify_one();
      done_cv_.wait(lock, [&job]() {return job->finished;});
    }
    if (job->error) {
      std::rethrow_exception(job->error);
    }
    return job->ran;
  }

  bool stopRequested() const
  {
    return stop_requested_.load();
  }

  void stop()
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopping_ = true;
      stop_requested_.store(true);
    }
    work_cv_.notify_one();
    if (worker_.joinable()) {
      worker_.join();
    }
  }

private:
  struct Job
  {
    std::function<void()> task;
    bool finished {false};
    bool ran {false};
    std::exception_ptr error;
  };

  void run()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    while (true) {
      work_cv_.wait(lock, [this]() {return stopping_ || drain_pending_ || !jobs_.empty();});
      if (stopping_) {
        break;
      }
      if (!jobs_.empty()) {
        const std::shared_ptr<Job> job = jobs_.front();
        jobs_.pop_front();
        lock.unlock();
        try {
          job->task();
        } catch (...) {
          job->error = std::current_exception();
        }
        lock.lock();
        job->ran = true;
        job->finished = true;
        done_cv_.notify_all();
        continue;
      }
      drain_pending_ = false;
      lock.unlock();
      std::exception_ptr drain_error;
      try {
        drain_();
      } catch (...) {
        drain_error = std::current_exception();
      }
      lock.lock();
      if (drain_error && !drain_error_) {
        drain_error_ = drain_error;
      }
    }
    for (const auto & job : jobs_) {
      job->finished = true;
    }
    jobs_.clear();
    done_cv_.notify_all();
  }

  std::function<void()> drain_;
  std::mutex mutex_;
  std::condition_variable work_cv_;
  std::condition_variable done_cv_;
  bool stopping_ {false};
  bool drain_pending_ {false};
  std::deque<std::shared_ptr<Job>> jobs_;
  std::exception_ptr drain_error_;
  std::atomic<bool> stop_requested_ {false};
  // Declared last: the worker starts only after every member it uses exists.
  std::thread worker_;
};

}  // namespace scheduling
}  // namespace graphslam

#endif  // GRAPH_BASED_SLAM__BACKGROUND_WORK_DRAIN_HPP_
