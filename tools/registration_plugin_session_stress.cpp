// Copyright 2026 Sasaki
// SPDX-License-Identifier: BSD-2-Clause

// Bounded, ROS-free stress executable for RegistrationPluginSession.  The
// loader itself is a C++17 shell because pluginlib/Jazzy use filesystem APIs;
// this consumer deliberately remains C++14 to exercise the installed SDK
// boundary.  It uses host fake providers so no bag, DSO, or benchmark input is
// involved.

#include "lidarslam_registration_loader/registration_plugin_loader.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {

namespace registration = lidarslam::plugins::registration;
namespace shell = lidarslam::plugins::registration::shell;

#ifndef LIDARSLAM_REGISTRATION_STRESS_SOURCE_SHA256
#define LIDARSLAM_REGISTRATION_STRESS_SOURCE_SHA256 "unbound"
#endif

#if defined(__SANITIZE_THREAD__)
constexpr const char * kThreadSanitizer = "enabled";
#elif defined(__clang__) && __has_feature(thread_sanitizer)
constexpr const char * kThreadSanitizer = "enabled";
#else
constexpr const char * kThreadSanitizer = "disabled";
#endif

struct CheckState final
{
  std::atomic<unsigned int> failures{0U};
  std::mutex mutex;
  std::vector<std::string> messages;

  void require(const bool condition, const std::string & message)
  {
    if (condition) {
      return;
    }
    failures.fetch_add(1U);
    std::lock_guard<std::mutex> lock(mutex);
    messages.push_back(message);
  }
};

template<typename Predicate>
bool waitFor(Predicate predicate, const std::chrono::milliseconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (predicate()) {
      return true;
    }
    std::this_thread::yield();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return predicate();
}

registration::PointCloud::Ptr makeCloud()
{
  registration::PointCloud::Ptr cloud(new registration::PointCloud());
  registration::PointT point;
  point.x = 1.0F;
  point.y = 2.0F;
  point.z = 3.0F;
  point.intensity = 1.0F;
  cloud->push_back(point);
  return cloud;
}

class StressRegistration final
  : public registration::RegistrationPlugin,
    public registration::RegistrationPluginDescriptorProvider
{
public:
  StressRegistration(
    const std::string & class_id,
    const bool cooperative,
    const bool reentrant,
    const bool throw_once,
    const bool descriptor_corrupt,
    const bool descriptor_throw,
    const unsigned int work_steps)
  : class_id_(class_id),
    cooperative_(cooperative),
    reentrant_(reentrant),
    throw_once_(throw_once),
    descriptor_corrupt_(descriptor_corrupt),
    descriptor_throw_(descriptor_throw),
    work_steps_(work_steps) {}

  ~StressRegistration() override
  {
    destroyed_.fetch_add(1U);
  }

  registration::PluginMetadata metadata() const override
  {
    registration::PluginMetadata metadata;
    metadata.class_id = class_id_;
    metadata.implementation_version = "stress-1";
    metadata.license = "BSD-2-Clause";
    metadata.api_version = registration::kHostApiVersion;
    return metadata;
  }

  registration::Capabilities capabilities() const override
  {
    registration::Capabilities capabilities;
    capabilities
    .add(registration::Capability::kInitialGuess)
    .add(registration::Capability::kAlignedSource)
    .add(registration::Capability::kDeterministic)
    .setTargetPolicy(registration::TargetPolicy::kRequiresRawTarget)
    .setCorrespondenceMetric(registration::CorrespondenceMetric::kMeanDistance)
    .setThreadModel(reentrant_ ? registration::ThreadModel::kReentrant :
      registration::ThreadModel::kSerializedOwner)
    .setCancellationModel(cooperative_ ?
      registration::CancellationModel::kCooperativeCancel :
      registration::CancellationModel::kNonInterruptibleAlign);
    return capabilities;
  }

  registration::RegistrationRuntimeDescriptor registrationDescriptor() const override
  {
    if (descriptor_throw_) {
      throw std::runtime_error("synthetic descriptor provider failure");
    }
    const registration::Capabilities current = capabilities();
    registration::RegistrationRuntimeDescriptor descriptor =
      registration::makeRegistrationRuntimeDescriptor(
      metadata(), current, current.bits(), 0U,
      registration::registrationConfigSchemaForClassId(class_id_));
    if (descriptor_corrupt_) {
      descriptor.toolchain_tag += ".drift";
    }
    return descriptor;
  }

  bool configure(const registration::ParameterMap &, std::string *) override
  {
    configured_.store(true);
    return true;
  }

  bool setInputTarget(
    const registration::PointCloudConstPtr & target, std::string * error) override
  {
    if (!configured_.load()) {
      if (error != nullptr) {
        *error = "stress provider is not configured";
      }
      return false;
    }
    if (!target || target->empty()) {
      if (error != nullptr) {
        *error = "stress provider target is empty";
      }
      return false;
    }
    if (reset_seen_.load()) {
      callback_after_reset_.store(true);
    }
    std::atomic_store(&target_, target);
    return true;
  }

  registration::AlignmentResult align(
    const registration::AlignmentRequest & request) override
  {
    struct ActiveGuard final
    {
      explicit ActiveGuard(StressRegistration * owner)
      : owner_(owner) {owner_->active_calls_.fetch_add(1U);}
      ~ActiveGuard()
      {
        owner_->active_calls_.fetch_sub(1U);
      }
      StressRegistration * owner_;
    } active(this);

    align_calls_.fetch_add(1U);
    if (reset_seen_.load()) {
      callback_after_reset_.store(true);
    }
    if (!std::atomic_load(&target_) || !configured_.load()) {
      registration::AlignmentResult result;
      result.failure = registration::FailureCode::kInvalidInput;
      result.diagnostics.detail = "stress provider target/configuration missing";
      return result;
    }
    if (!request.source || request.source->empty()) {
      registration::AlignmentResult result;
      result.failure = registration::FailureCode::kInvalidInput;
      result.diagnostics.detail = "stress provider source missing";
      return result;
    }
    if (throw_once_ && !throw_consumed_.exchange(true)) {
      throw std::runtime_error("synthetic align provider failure");
    }

    started_.store(true);
    for (unsigned int step = 0U; step < work_steps_; ++step) {
      if (cooperative_ && cancel_requested_.load()) {
        cancellation_observed_.store(true);
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    if (reset_seen_.load()) {
      callback_after_reset_.store(true);
    }

    registration::AlignmentResult result;
    result.converged = true;
    result.final_transformation = request.initial_guess;
    result.aligned_source.reset(new registration::PointCloud(*request.source));
    result.fitness_score = 0.0;
    result.failure = registration::FailureCode::kNone;
    result.diagnostics.mean_correspondence_distance_valid = true;
    result.diagnostics.mean_correspondence_distance = 0.0;
    return result;
  }

  void requestCancel() noexcept override
  {
    cancel_requests_.fetch_add(1U);
    cancel_requested_.store(true);
  }

  void reset() noexcept override
  {
    if (active_calls_.load() != 0U) {
      reset_during_callback_.store(true);
    }
    reset_calls_.fetch_add(1U);
    reset_seen_.store(true);
    configured_.store(false);
    std::atomic_store(&target_, registration::PointCloudConstPtr());
  }

  void markStartedForTest() {started_.store(true);}
  bool started() const {return started_.load();}
  bool cancellationObserved() const {return cancellation_observed_.load();}
  bool callbackAfterReset() const {return callback_after_reset_.load();}
  bool resetDuringCallback() const {return reset_during_callback_.load();}
  unsigned int alignCalls() const {return align_calls_.load();}
  unsigned int cancelRequests() const {return cancel_requests_.load();}
  unsigned int resetCalls() const {return reset_calls_.load();}
  unsigned int activeCalls() const {return active_calls_.load();}

  static unsigned int destroyed() {return destroyed_.load();}

private:
  std::string class_id_;
  bool cooperative_{false};
  bool reentrant_{false};
  bool throw_once_{false};
  bool descriptor_corrupt_{false};
  bool descriptor_throw_{false};
  unsigned int work_steps_{0U};
  std::atomic<bool> configured_{false};
  std::shared_ptr<const registration::PointCloud> target_;
  std::atomic<bool> cancel_requested_{false};
  std::atomic<bool> cancellation_observed_{false};
  std::atomic<bool> started_{false};
  std::atomic<bool> reset_seen_{false};
  std::atomic<bool> callback_after_reset_{false};
  std::atomic<bool> reset_during_callback_{false};
  std::atomic<bool> throw_consumed_{false};
  std::atomic<unsigned int> align_calls_{0U};
  std::atomic<unsigned int> cancel_requests_{0U};
  std::atomic<unsigned int> reset_calls_{0U};
  std::atomic<unsigned int> active_calls_{0U};
  static std::atomic<unsigned int> destroyed_;
};

std::atomic<unsigned int> StressRegistration::destroyed_{0U};

shell::LoadRequest makeRequest(const std::string & class_id, const bool cooperative)
{
  shell::LoadRequest request;
  request.class_id = class_id;
  request.capabilities.require_initial_guess = true;
  request.capabilities.require_aligned_source = true;
  request.capabilities.require_deterministic = true;
  request.capabilities.require_target_policy = true;
  request.capabilities.target_policy = registration::TargetPolicy::kRequiresRawTarget;
  request.capabilities.require_correspondence_metric = true;
  request.capabilities.correspondence_metric = registration::CorrespondenceMetric::kMeanDistance;
  request.capabilities.require_cooperative_cancel = cooperative;
  return request;
}

std::shared_ptr<shell::RegistrationPluginSession> createSession(
  const std::shared_ptr<StressRegistration> & provider,
  const bool cooperative,
  shell::LoadFailure * failure)
{
  return shell::RegistrationPluginSession::createHostSession(
    provider, makeRequest(provider->metadata().class_id, cooperative), "", true, failure);
}

registration::AlignmentRequest makeRequestForAlignment(
  const registration::PointCloudConstPtr & cloud)
{
  registration::AlignmentRequest request;
  request.source = cloud;
  request.initial_guess_enabled = false;
  return request;
}

void checkSessionCreated(
  CheckState * checks,
  const std::shared_ptr<shell::RegistrationPluginSession> & session,
  const shell::LoadFailure & failure,
  const std::string & label)
{
  checks->require(session != nullptr && failure.ok(), label + ": " + failure.message);
}

void runHighContention(CheckState * checks)
{
  const std::shared_ptr<StressRegistration> provider(new StressRegistration(
      "lidarslam_builtin/StressSerialized", true, false, false, false, false, 0U));
  shell::LoadFailure failure;
  const std::shared_ptr<shell::RegistrationPluginSession> session =
    createSession(provider, true, &failure);
  checkSessionCreated(checks, session, failure, "high-contention session");
  if (!session) {
    return;
  }
  shell::RegistrationPluginSessionAdapter adapter(*session);
  const registration::PointCloud::Ptr cloud = makeCloud();
  std::string target_error;
  checks->require(adapter.setInputTarget(cloud, &target_error),
    "high-contention target setup: " + target_error);

  std::atomic<unsigned int> call_failures{0U};
  std::vector<std::thread> workers;
  for (unsigned int worker = 0U; worker < 8U; ++worker) {
    workers.emplace_back([&adapter, &cloud, &call_failures]() {
        for (unsigned int iteration = 0U; iteration < 32U; ++iteration) {
          const registration::AlignmentResult result = adapter.align(
            makeRequestForAlignment(cloud));
          if (result.failure != registration::FailureCode::kNone) {
            call_failures.fetch_add(1U);
          }
        }
      });
  }
  for (auto & worker : workers) {
    worker.join();
  }
  checks->require(call_failures.load() == 0U,
    "serialized high-contention calls returned an unexpected failure");
  std::vector<std::thread> shutdown_workers;
  for (unsigned int worker = 0U; worker < 6U; ++worker) {
    shutdown_workers.emplace_back([&session]() {session->shutdown();});
  }
  for (auto & worker : shutdown_workers) {
    worker.join();
  }
  checks->require(provider->resetCalls() == 1U,
    "idempotent shutdown called reset more than once");
  checks->require(provider->activeCalls() == 0U,
    "high-contention session retained an active provider callback");
  checks->require(!provider->resetDuringCallback() && !provider->callbackAfterReset(),
    "provider callback overlapped or followed shutdown reset");
}

void runCancellation(CheckState * checks)
{
  {
    const std::shared_ptr<StressRegistration> provider(new StressRegistration(
        "lidarslam_builtin/StressCancelBefore", true, false, false, false, false, 4U));
    shell::LoadFailure failure;
    const auto session = createSession(provider, true, &failure);
    checkSessionCreated(checks, session, failure, "cancel-before session");
    if (session) {
      session->cancel();
      const auto result = session->align(makeRequestForAlignment(makeCloud()));
      checks->require(result.failure == registration::FailureCode::kCancelled &&
        provider->alignCalls() == 0U, "cancel-before did not block provider admission");
      session->shutdown();
    }
  }

  {
    const std::shared_ptr<StressRegistration> provider(new StressRegistration(
        "lidarslam_builtin/StressCooperative", true, true, false, false, false, 250U));
    shell::LoadFailure failure;
    const auto session = createSession(provider, true, &failure);
    checkSessionCreated(checks, session, failure, "cooperative cancel session");
    if (!session) {
      return;
    }
    std::string target_error;
    checks->require(session->setInputTarget(makeCloud(), &target_error),
      "cooperative cancel target setup: " + target_error);
    const auto cloud = makeCloud();
    std::atomic<bool> done{false};
    registration::AlignmentResult result;
    std::thread worker([&]() {
        result = session->align(makeRequestForAlignment(cloud));
        done.store(true);
      });
    checks->require(waitFor([&provider]() {return provider->started();},
      std::chrono::milliseconds(500)), "cooperative provider did not start");
    session->cancel();
    std::vector<std::thread> shutdown_workers;
    for (unsigned int index = 0U; index < 5U; ++index) {
      shutdown_workers.emplace_back([&session]() {session->shutdown();});
    }
    checks->require(waitFor([&done]() {return done.load();},
      std::chrono::milliseconds(2000)), "cooperative cancellation/shutdown deadlocked");
    worker.join();
    for (auto & shutdown_worker : shutdown_workers) {
      shutdown_worker.join();
    }
    checks->require(result.failure == registration::FailureCode::kCancelled,
      "cooperative cancel-during did not return kCancelled");
    checks->require(provider->cancelRequests() > 0U && provider->cancellationObserved(),
      "cooperative provider did not receive/observe requestCancel");
    checks->require(provider->resetCalls() == 1U && provider->activeCalls() == 0U &&
      !provider->resetDuringCallback() && !provider->callbackAfterReset(),
      "cooperative shutdown violated the quiescence boundary");
  }

  {
    const std::shared_ptr<StressRegistration> provider(new StressRegistration(
        "lidarslam_builtin/StressNonInterruptible", false, true, false, false, false, 35U));
    shell::LoadFailure failure;
    const auto session = createSession(provider, false, &failure);
    checkSessionCreated(checks, session, failure, "non-interruptible session");
    if (!session) {
      return;
    }
    std::string target_error;
    checks->require(session->setInputTarget(makeCloud(), &target_error),
      "non-interruptible target setup: " + target_error);
    const auto cloud = makeCloud();
    std::atomic<bool> done{false};
    registration::AlignmentResult result;
    std::thread worker([&]() {
        result = session->align(makeRequestForAlignment(cloud));
        done.store(true);
      });
    checks->require(waitFor([&provider]() {return provider->started();},
      std::chrono::milliseconds(500)), "non-interruptible provider did not start");
    session->cancel();
    std::thread shutdown_worker([&session]() {session->shutdown();});
    checks->require(waitFor([&done]() {return done.load();},
      std::chrono::milliseconds(2000)), "non-interruptible shutdown did not quiesce");
    worker.join();
    shutdown_worker.join();
    checks->require(result.failure == registration::FailureCode::kCancelled,
      "non-interruptible cancel-during did not post-check cancellation");
    checks->require(provider->cancelRequests() == 0U,
      "non-interruptible provider was falsely advertised as cooperative");
    checks->require(!provider->resetDuringCallback() && !provider->callbackAfterReset(),
      "non-interruptible shutdown reset raced an active provider callback");
  }

  {
    const std::shared_ptr<StressRegistration> provider(new StressRegistration(
        "lidarslam_builtin/StressCancelAfter", true, false, false, false, false, 0U));
    shell::LoadFailure failure;
    const auto session = createSession(provider, true, &failure);
    checkSessionCreated(checks, session, failure, "cancel-after session");
    if (session) {
      std::string target_error;
      checks->require(session->setInputTarget(makeCloud(), &target_error),
        "cancel-after target setup: " + target_error);
      const auto result = session->align(makeRequestForAlignment(makeCloud()));
      checks->require(result.failure == registration::FailureCode::kNone,
        "cancel-after baseline alignment failed");
      session->cancel();
      const auto cancelled = session->align(makeRequestForAlignment(makeCloud()));
      checks->require(cancelled.failure == registration::FailureCode::kCancelled,
        "cancel-after did not block the next operation");
      session->shutdown();
    }
  }
}

void runFaultAndActivation(CheckState * checks)
{
  {
    const std::shared_ptr<StressRegistration> provider(new StressRegistration(
        "lidarslam_builtin/StressThrow", false, false, true, false, false, 0U));
    shell::LoadFailure failure;
    const auto session = createSession(provider, false, &failure);
    checkSessionCreated(checks, session, failure, "fault-latch session");
    if (session) {
      std::string target_error;
      checks->require(session->setInputTarget(makeCloud(), &target_error),
        "fault-latch target setup: " + target_error);
      const auto first = session->align(makeRequestForAlignment(makeCloud()));
      const unsigned int calls_after_throw = provider->alignCalls();
      const auto second = session->align(makeRequestForAlignment(makeCloud()));
      checks->require(first.failure == registration::FailureCode::kInternalError &&
        second.failure == registration::FailureCode::kInternalError && session->faulted() &&
        provider->alignCalls() == calls_after_throw,
        "provider exception did not latch the session fault boundary");
      session->shutdown();
    }
  }

  {
    const std::shared_ptr<StressRegistration> corrupt(new StressRegistration(
        "lidarslam_builtin/StressDescriptorCorrupt", false, false, false, true, false, 0U));
    shell::LoadFailure failure;
    const auto session = createSession(corrupt, false, &failure);
    checks->require(!session && failure.code == shell::LoadFailureCode::kDescriptorMismatch,
      "descriptor drift was not rejected before activation");
  }
  {
    const std::shared_ptr<StressRegistration> throwing(new StressRegistration(
        "lidarslam_builtin/StressDescriptorThrow", false, false, false, false, true, 0U));
    shell::LoadFailure failure;
    const auto session = createSession(throwing, false, &failure);
    checks->require(!session && failure.code == shell::LoadFailureCode::kPluginException,
      "throwing descriptor provider was not rejected fail-closed");
  }

  const std::shared_ptr<StressRegistration> previous_provider(new StressRegistration(
      "lidarslam_builtin/StressActivationPrevious", true, false, false, false, false, 0U));
  const std::shared_ptr<StressRegistration> candidate_provider(new StressRegistration(
      "lidarslam_builtin/StressActivationCandidate", true, false, false, false, false, 0U));
  shell::LoadFailure previous_failure;
  shell::LoadFailure candidate_failure;
  const auto previous = createSession(previous_provider, true, &previous_failure);
  const auto candidate = createSession(candidate_provider, true, &candidate_failure);
  checkSessionCreated(checks, previous, previous_failure, "activation previous session");
  checkSessionCreated(checks, candidate, candidate_failure, "activation candidate session");
  if (previous && candidate) {
    std::shared_ptr<shell::RegistrationPluginSession> active_session = previous;
    std::shared_ptr<registration::RegistrationPlugin> active_plugin = previous->plugin();
    shell::RegistrationActivationSlots slots{&active_session, &active_plugin, nullptr, nullptr, nullptr};
    shell::RegistrationActivationTransaction transaction(slots);
    shell::LoadFailure failure;
    checks->require(transaction.prepare(candidate, &failure),
      "activation transaction prepare failed unexpectedly");
    checks->require(!transaction.validate(
      [](const shell::RegistrationActivationSnapshot &) {
        return shell::LoadFailure{shell::LoadFailureCode::kDescriptorMismatch,
          "synthetic activation descriptor rejection"};
      }, &failure) && failure.code == shell::LoadFailureCode::kDescriptorMismatch,
      "activation descriptor failure was not rejected");
    checks->require(active_session == previous && active_plugin == previous->plugin(),
      "failed activation mutated the active host pair");
    previous->shutdown();
    candidate->shutdown();
  }
}

void runRepeatedLifetime(CheckState * checks)
{
  const unsigned int destroyed_before = StressRegistration::destroyed();
  for (unsigned int iteration = 0U; iteration < 24U; ++iteration) {
    std::weak_ptr<registration::RegistrationPlugin> weak_plugin;
    {
      const std::shared_ptr<StressRegistration> provider(new StressRegistration(
          "lidarslam_builtin/StressLifetime", true, false, false, false, false, 0U));
      weak_plugin = provider;
      shell::LoadFailure failure;
      auto session = createSession(provider, true, &failure);
      checkSessionCreated(checks, session, failure, "repeated lifetime session");
      if (!session) {
        continue;
      }
      shell::RegistrationPluginSessionAdapter adapter(*session);
      std::string target_error;
      checks->require(adapter.setInputTarget(makeCloud(), &target_error),
        "repeated lifetime target setup: " + target_error);
      checks->require(adapter.align(makeRequestForAlignment(makeCloud())).failure ==
        registration::FailureCode::kNone, "repeated lifetime alignment failed");
      session->shutdown();
      session.reset();
      checks->require(!weak_plugin.expired(),
        "caller-owned provider disappeared before its final shared reference");
    }
    checks->require(weak_plugin.expired(),
      "provider instance remained alive after session and caller release");
  }
  checks->require(StressRegistration::destroyed() >= destroyed_before + 24U,
    "repeated session create/destroy did not release provider instances");
}

void writeReceipt(const std::string & path, const CheckState & checks)
{
  if (path.empty()) {
    return;
  }
  std::ofstream output(path.c_str());
  output << "{\n"
    << "  \"schema\": \"registration-plugin-session-stress-v1\",\n"
    << "  \"source_sha256\": \"" << LIDARSLAM_REGISTRATION_STRESS_SOURCE_SHA256 << "\",\n"
    << "  \"cxx_standard\": 14,\n"
    << "  \"thread_sanitizer\": \"" << kThreadSanitizer << "\",\n"
#if defined(__GNUC__)
    << "  \"compiler\": \"gcc-" << __GNUC__ << "." << __GNUC_MINOR__ << "\",\n"
#else
    << "  \"compiler\": \"unknown\",\n"
#endif
    << "  \"hardware_threads\": " << std::thread::hardware_concurrency() << ",\n"
    << "  \"bounded_iterations\": {\"workers\": 8, \"calls_per_worker\": 32, \"lifetime_runs\": 24},\n"
    << "  \"failures\": " << checks.failures.load() << ",\n"
    << "  \"status\": \"" << (checks.failures.load() == 0U ? "PASS" : "FAIL") << "\"\n}\n";
}

}  // namespace

int main(int argc, char ** argv)
{
  std::string receipt_path;
  for (int index = 1; index < argc; ++index) {
    const std::string argument(argv[index]);
    if (argument == "--receipt" && index + 1 < argc) {
      receipt_path = argv[++index];
    } else if (argument == "--help") {
      std::cout << "usage: registration_plugin_session_stress [--receipt PATH]\n";
      return 0;
    }
  }

  CheckState checks;
  runHighContention(&checks);
  runCancellation(&checks);
  runFaultAndActivation(&checks);
  runRepeatedLifetime(&checks);
  writeReceipt(receipt_path, checks);

  if (checks.failures.load() != 0U) {
    std::lock_guard<std::mutex> lock(checks.mutex);
    for (const auto & message : checks.messages) {
      std::cerr << "FAIL: " << message << "\n";
    }
    return 1;
  }
  std::cout << "registration plugin session stress PASS (TSAN " << kThreadSanitizer << ")\n";
  return 0;
}
