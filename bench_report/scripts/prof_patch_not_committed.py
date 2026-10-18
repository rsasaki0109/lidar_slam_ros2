# Temporary profiling instrumentation (never committed): per-candidate NDT timing in the live
# PclRegistrationAdapter. Usage: prof_patch.py apply|revert
import sys
p = '/home/user/lidar_slam_ros2/graph_based_slam/include/graph_based_slam/registration_plugin_adapter.hpp'
s = open(p).read()
marker = '// PROF-PATCH'
if sys.argv[1] == 'revert':
    import subprocess
    subprocess.check_call(['git', '-C', '/home/user/lidar_slam_ros2', 'checkout', '--',
                           'graph_based_slam/include/graph_based_slam/registration_plugin_adapter.hpp'])
    print('reverted'); sys.exit(0)
assert marker not in s
def rep(old, new):
    global s
    assert s.count(old) == 1, old
    s = s.replace(old, new)
rep('#include <pcl/registration/registration.h>  // NOLINT(build/include_order)\n',
    '#include <pcl/registration/registration.h>  // NOLINT(build/include_order)\n'
    '#include <chrono>  ' + marker + '\n#include <cstdio>\n#include <pclomp/ndt_omp.h>\n')
rep('''    try {
      registration_->setInputTarget(target);
      target_ = target;
      return true;''', '''    try {
      const auto prof_t0 = std::chrono::steady_clock::now();  ''' + marker + '''
      registration_->setInputTarget(target);
      prof_set_ms_ = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - prof_t0).count();
      target_ = target;
      return true;''')
rep('''      registration_->setInputSource(request.source);
      PointCloud aligned;
      if (request.initial_guess_enabled) {
        registration_->align(aligned, request.initial_guess);
      } else {
        registration_->align(aligned);
      }
      result.converged = registration_->hasConverged();
      result.final_transformation = registration_->getFinalTransformation();
      result.fitness_score = registration_->getFitnessScore();''', '''      registration_->setInputSource(request.source);
      PointCloud aligned;
      const auto prof_t0 = std::chrono::steady_clock::now();  ''' + marker + '''
      if (request.initial_guess_enabled) {
        registration_->align(aligned, request.initial_guess);
      } else {
        registration_->align(aligned);
      }
      const auto prof_t1 = std::chrono::steady_clock::now();
      result.converged = registration_->hasConverged();
      result.final_transformation = registration_->getFinalTransformation();
      result.fitness_score = registration_->getFitnessScore();
      const auto prof_t2 = std::chrono::steady_clock::now();
      int prof_iterations = -1;
      if (const auto * ndt = dynamic_cast<
          pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> *>(registration_))
      {
        prof_iterations = ndt->getFinalNumIteration();
      }
      std::fprintf(
        stderr, "PROF src=%zu tgt=%zu set_ms=%.1f align_ms=%.1f fit_ms=%.1f iters=%d fitness=%.5f\\n",
        request.source->size(), target_->size(), prof_set_ms_,
        std::chrono::duration<double, std::milli>(prof_t1 - prof_t0).count(),
        std::chrono::duration<double, std::milli>(prof_t2 - prof_t1).count(),
        prof_iterations, result.fitness_score);''')
# member
idx = s.rindex('};\n\n}  // namespace backend_registration')
s = s[:idx] + '  double prof_set_ms_ {0.0};  ' + marker + '\n' + s[idx:]
open(p, 'w').write(s)
print('applied')
