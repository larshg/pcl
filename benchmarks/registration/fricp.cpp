#include <pcl/filters/filter.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/fricp.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>

#if defined(__has_include)
#if __has_include(<pcl/visualization/pcl_visualizer.h>)
#include <pcl/visualization/pcl_visualizer.h>
#define PCL_BENCH_HAS_VISUALIZER 1
#endif
#endif
#ifndef PCL_BENCH_HAS_VISUALIZER
#define PCL_BENCH_HAS_VISUALIZER 0
#endif

#include <benchmark/benchmark.h>

#include <Eigen/Geometry>

#include <iomanip>
#include <iostream>
#include <random>
#include <sstream>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace {
using PointT = pcl::PointXYZ;
using Cloud = pcl::PointCloud<PointT>;
using CloudPtr = Cloud::Ptr;

struct FRICPResult {
  bool converged{false};
  Eigen::Matrix4f transform{Eigen::Matrix4f::Identity()};
  double fitness{0.0};
  int convergence_state{-1};
  std::string convergence_criteria{"not-available"};
  std::string native_convergence_trigger{"not-available"};
};

using RegistrationResult = FRICPResult;

template <typename T, typename = void>
struct HasGetConvergeCriteria : std::false_type {};

template <typename T>
struct HasGetConvergeCriteria<
    T,
    std::void_t<decltype(std::declval<T&>().getConvergeCriteria())>>
    : std::true_type {};

template <typename T, typename = void>
struct HasGetLastConvergenceTrigger : std::false_type {};

template <typename T>
struct HasGetLastConvergenceTrigger<
  T,
  std::void_t<decltype(std::declval<T&>().getLastConvergenceTrigger())>>
  : std::true_type {};

const char*
convergenceStateToString(int state)
{
  switch (state) {
    case 0:
      return "NOT_CONVERGED";
    case 1:
      return "ITERATIONS";
    case 2:
      return "TRANSFORM";
    case 3:
      return "ABS_MSE";
    case 4:
      return "REL_MSE";
    case 5:
      return "NO_CORRESPONDENCES";
    case 6:
      return "FAILURE_AFTER_MAX_ITERATIONS";
    default:
      return "UNKNOWN";
  }
}

const char*
fricpNativeTriggerToString(int trigger)
{
  switch (trigger) {
    case 0:
      return "NONE";
    case 1:
      return "DEFAULT_CRITERIA";
    case 2:
      return "FRICP_STOP_THRESHOLD";
    case 3:
      return "ITERATION_LIMIT";
    case 4:
      return "NO_CORRESPONDENCES";
    default:
      return "UNKNOWN";
  }
}

template <typename RegistrationT>
int
getConvergenceStateSafe(RegistrationT& reg)
{
  if constexpr (HasGetConvergeCriteria<RegistrationT>::value) {
    const auto criteria = reg.getConvergeCriteria();
    if (criteria) {
      return static_cast<int>(criteria->getConvergenceState());
    }
    return -2;
  }
  return -1;
}

template <typename RegistrationT>
std::string
getNativeConvergenceTriggerSafe(RegistrationT& reg)
{
  if constexpr (HasGetLastConvergenceTrigger<RegistrationT>::value) {
    const int trigger = static_cast<int>(reg.getLastConvergenceTrigger());
    std::ostringstream ss;
    ss << fricpNativeTriggerToString(trigger) << "(" << trigger << ")";
    return ss.str();
  }
  return "not-available";
}

template <typename RegistrationT>
std::string
getConvergenceCriteriaSummarySafe(RegistrationT& reg)
{
  if constexpr (HasGetConvergeCriteria<RegistrationT>::value) {
    const auto criteria = reg.getConvergeCriteria();
    if (!criteria) {
      return "available-but-null";
    }

    const int state = static_cast<int>(criteria->getConvergenceState());
    std::ostringstream ss;
    ss << "state=" << convergenceStateToString(state) << "(" << state << ")"
       << ", max_iter=" << criteria->getMaximumIterations()
       << ", rot_thr=" << criteria->getRotationThreshold()
       << ", trans_thr=" << criteria->getTranslationThreshold()
       << ", rel_mse_thr=" << criteria->getRelativeMSE()
       << ", abs_mse_thr=" << criteria->getAbsoluteMSE();
    return ss.str();
  }

  return "not-available";
}

CloudPtr
loadCloud(const std::string& file)
{
  CloudPtr cloud(new Cloud);
  if (pcl::io::loadPCDFile(file, *cloud) < 0) {
    return CloudPtr();
  }

  CloudPtr filtered(new Cloud);
  pcl::Indices indices;
  pcl::removeNaNFromPointCloud(*cloud, *filtered, indices);
  return filtered;
}

CloudPtr
makeTransformedTarget(const CloudPtr& target, const Eigen::Matrix4f& transform)
{
  CloudPtr transformed(new Cloud);
  pcl::transformPointCloud(*target, *transformed, transform);

  transformed->width = static_cast<std::uint32_t>(transformed->size());
  transformed->height = 1;
  transformed->is_dense = target->is_dense;

  return transformed;
}

CloudPtr
makeNoisyTarget(const CloudPtr& transformed_target)
{
  CloudPtr transformed(new Cloud(*transformed_target));

  std::mt19937 rng(1337u);
  std::normal_distribution<float> gaussian(0.0f, 0.004f);
  for (auto& p : *transformed) {
    p.x += gaussian(rng);
    p.y += gaussian(rng);
    p.z += gaussian(rng);
  }

  std::uniform_real_distribution<float> uniform(-0.4f, 0.4f);
  for (int i = 0; i < 20; ++i) {
    PointT outlier;
    outlier.x = uniform(rng);
    outlier.y = uniform(rng);
    outlier.z = uniform(rng) + 0.4f;
    transformed->push_back(outlier);
  }

  transformed->width = static_cast<std::uint32_t>(transformed->size());
  transformed->height = 1;
  transformed->is_dense = false;

  return transformed;
}

template <typename RegistrationT>
void
runRegistrationBenchmark(benchmark::State& state,
                         const CloudPtr& source,
                         const CloudPtr& target,
                         bool use_guess,
                         const Eigen::Matrix4f& guess,
                         const char* algo_name)
{
  double last_fitness = 0.0;
  double converged_counter = 1.0;
  double last_convergence_state = -1.0;

  for (auto _ : state) {
    state.PauseTiming();
    RegistrationT reg;
    reg.setInputSource(source);
    reg.setInputTarget(target);
    reg.setMaximumIterations(100);
    reg.setTransformationEpsilon(1e-8);
    Cloud output;
    state.ResumeTiming();

    if (use_guess) {
      reg.align(output, guess);
    }
    else {
      reg.align(output);
    }

    if (!reg.hasConverged()) {
      converged_counter = 0.0;
      state.SkipWithError(algo_name);
      break;
    }

    last_fitness = reg.getFitnessScore();
    last_convergence_state = static_cast<double>(getConvergenceStateSafe(reg));

    benchmark::DoNotOptimize(output.size());
    benchmark::ClobberMemory();
  }

  state.counters["converged"] = converged_counter;
  state.counters["fitness"] = last_fitness;
  state.counters["convergence_state"] = last_convergence_state;
}

template <typename RegistrationT>
RegistrationResult
runRegistrationOnce(const CloudPtr& source,
                    const CloudPtr& target,
                    bool use_guess,
                    const Eigen::Matrix4f& guess)
{
  RegistrationT reg;
  reg.setInputSource(source);
  reg.setInputTarget(target);
  reg.setMaximumIterations(100);
  reg.setTransformationEpsilon(1e-8);

  Cloud output;
  if (use_guess) {
    reg.align(output, guess);
  }
  else {
    reg.align(output);
  }

  RegistrationResult result;
  result.converged = reg.hasConverged();
  result.transform = reg.getFinalTransformation();
  result.fitness = reg.getFitnessScore();
  result.convergence_state = getConvergenceStateSafe(reg);
  result.convergence_criteria = getConvergenceCriteriaSummarySafe(reg);
  result.native_convergence_trigger = getNativeConvergenceTriggerSafe(reg);
  return result;
}

void
printResult(const std::string& label, const RegistrationResult& result)
{
  std::cout << label << " converged="
            << (result.converged ? "true" : "false")
            << ", fitness=" << result.fitness << '\n'
            << "convergence_state=" << result.convergence_state << " ("
            << convergenceStateToString(result.convergence_state) << ")\n"
            << "convergence_criteria=" << result.convergence_criteria << '\n'
            << "native_convergence_trigger=" << result.native_convergence_trigger
            << '\n'
            << result.transform << "\n\n";
}

  void
  visualizeAllResults(const CloudPtr& source,
                      const CloudPtr& clean_target,
                      const CloudPtr& noisy_target,
                      const RegistrationResult& fricp_clean_result,
                      const RegistrationResult& fricp_noisy_result,
                      const RegistrationResult& icp_clean_result,
                      const RegistrationResult& icp_noisy_result,
                      const RegistrationResult& gicp_clean_result,
                      const RegistrationResult& gicp_noisy_result)
  {
  #if PCL_BENCH_HAS_VISUALIZER
      CloudPtr fricp_clean_aligned(new Cloud);
      CloudPtr fricp_noisy_aligned(new Cloud);
      CloudPtr icp_clean_aligned(new Cloud);
      CloudPtr icp_noisy_aligned(new Cloud);
      CloudPtr gicp_clean_aligned(new Cloud);
      CloudPtr gicp_noisy_aligned(new Cloud);

      pcl::transformPointCloud(*source, *fricp_clean_aligned, fricp_clean_result.transform);
      pcl::transformPointCloud(*source, *fricp_noisy_aligned, fricp_noisy_result.transform);
      pcl::transformPointCloud(*source, *icp_clean_aligned, icp_clean_result.transform);
      pcl::transformPointCloud(*source, *icp_noisy_aligned, icp_noisy_result.transform);
      pcl::transformPointCloud(*source, *gicp_clean_aligned, gicp_clean_result.transform);
      pcl::transformPointCloud(*source, *gicp_noisy_aligned, gicp_noisy_result.transform);

      pcl::visualization::PCLVisualizer viewer("Registration clean vs noisy comparison");
    int left_view = 0;
    int right_view = 1;
    viewer.createViewPort(0.0, 0.0, 0.5, 1.0, left_view);
    viewer.createViewPort(0.5, 0.0, 1.0, 1.0, right_view);

    viewer.setBackgroundColor(0.08, 0.08, 0.08, left_view);
    viewer.setBackgroundColor(0.08, 0.08, 0.08, right_view);
      viewer.addText("Clean target / no initial guess", 10, 10, 14, 1.0, 1.0, 1.0,
                     "left_label", left_view);
      viewer.addText("Noisy target / with initial guess", 10, 10, 14, 1.0, 1.0, 1.0,
                     "right_label", right_view);
      viewer.addText("Target: green", 10, 32, 12, 0.6, 1.0, 0.6, "legend_target_l",
                     left_view);
      viewer.addText("FRICP: red", 10, 50, 12, 1.0, 0.4, 0.4, "legend_fricp_l",
                     left_view);
      viewer.addText("ICP: blue", 10, 68, 12, 0.5, 0.7, 1.0, "legend_icp_l",
                     left_view);
      viewer.addText("GICP: yellow", 10, 86, 12, 1.0, 0.9, 0.3, "legend_gicp_l",
                     left_view);

      viewer.addText("Target: green", 10, 32, 12, 0.6, 1.0, 0.6, "legend_target_r",
                     right_view);
      viewer.addText("FRICP: red", 10, 50, 12, 1.0, 0.4, 0.4, "legend_fricp_r",
                     right_view);
      viewer.addText("ICP: blue", 10, 68, 12, 0.5, 0.7, 1.0, "legend_icp_r",
                     right_view);
      viewer.addText("GICP: yellow", 10, 86, 12, 1.0, 0.9, 0.3, "legend_gicp_r",
                     right_view);

    pcl::visualization::PointCloudColorHandlerCustom<PointT> clean_target_color(
        clean_target, 80, 220, 80);
      pcl::visualization::PointCloudColorHandlerCustom<PointT> fricp_clean_color(
        fricp_clean_aligned, 220, 80, 80);
      pcl::visualization::PointCloudColorHandlerCustom<PointT> icp_clean_color(
        icp_clean_aligned, 90, 130, 255);
      pcl::visualization::PointCloudColorHandlerCustom<PointT> gicp_clean_color(
        gicp_clean_aligned, 255, 220, 80);
      viewer.addPointCloud<PointT>(clean_target, clean_target_color, "clean_target", left_view);
      viewer.addPointCloud<PointT>(
        fricp_clean_aligned, fricp_clean_color, "fricp_clean_aligned", left_view);
      viewer.addPointCloud<PointT>(
        icp_clean_aligned, icp_clean_color, "icp_clean_aligned", left_view);
      viewer.addPointCloud<PointT>(
        gicp_clean_aligned, gicp_clean_color, "gicp_clean_aligned", left_view);
    viewer.setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2.0, "clean_target", left_view);
    viewer.setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2.0, "fricp_clean_aligned", left_view);
      viewer.setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2.0, "icp_clean_aligned", left_view);
      viewer.setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2.0, "gicp_clean_aligned", left_view);

    pcl::visualization::PointCloudColorHandlerCustom<PointT> noisy_target_color(
      noisy_target, 80, 220, 80);
      pcl::visualization::PointCloudColorHandlerCustom<PointT> fricp_noisy_color(
        fricp_noisy_aligned, 220, 80, 80);
      pcl::visualization::PointCloudColorHandlerCustom<PointT> icp_noisy_color(
        icp_noisy_aligned, 90, 130, 255);
      pcl::visualization::PointCloudColorHandlerCustom<PointT> gicp_noisy_color(
        gicp_noisy_aligned, 255, 220, 80);
    viewer.addPointCloud<PointT>(
      noisy_target, noisy_target_color, "noisy_target", right_view);
    viewer.addPointCloud<PointT>(
        fricp_noisy_aligned, fricp_noisy_color, "fricp_noisy_aligned", right_view);
      viewer.addPointCloud<PointT>(
        icp_noisy_aligned, icp_noisy_color, "icp_noisy_aligned", right_view);
      viewer.addPointCloud<PointT>(
        gicp_noisy_aligned, gicp_noisy_color, "gicp_noisy_aligned", right_view);
    viewer.setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2.0, "noisy_target", right_view);
    viewer.setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2.0, "fricp_noisy_aligned", right_view);
      viewer.setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2.0, "icp_noisy_aligned", right_view);
      viewer.setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2.0, "gicp_noisy_aligned", right_view);

    viewer.addCoordinateSystem(0.1);
    while (!viewer.wasStopped()) {
      viewer.spinOnce(16);
    }
  #else
    (void)source;
    (void)clean_target;
    (void)noisy_target;
    (void)fricp_clean_result;
    (void)fricp_noisy_result;
    (void)icp_clean_result;
    (void)icp_noisy_result;
    (void)gicp_clean_result;
    (void)gicp_noisy_result;
    std::cerr << "Visualization requested, but pcl_visualization is not available in this build."
        << std::endl;
  #endif
}

void
BM_FRICP_Clean_NoInitialGuess(benchmark::State& state,
                              const CloudPtr& source,
                              const CloudPtr& target,
                              const Eigen::Matrix4f& identity)
{
  runRegistrationBenchmark<pcl::FastRobustIterativeClosestPoint<PointT, PointT>>(
      state, source, target, false, identity, "FRICP did not converge");
}

void
BM_FRICP_NoisyTarget_WithInitialGuess(benchmark::State& state,
                                      const CloudPtr& source,
                                      const CloudPtr& noisy_target,
                                      const Eigen::Matrix4f& guess)
{
  runRegistrationBenchmark<pcl::FastRobustIterativeClosestPoint<PointT, PointT>>(
      state, source, noisy_target, true, guess, "FRICP did not converge");
}

void
BM_ICP_CleanTarget_NoInitialGuess(benchmark::State& state,
                                  const CloudPtr& source,
                                  const CloudPtr& target,
                                  const Eigen::Matrix4f& identity)
{
  runRegistrationBenchmark<pcl::IterativeClosestPoint<PointT, PointT>>(
      state, source, target, false, identity, "ICP did not converge");
}

void
BM_ICP_NoisyTarget_WithInitialGuess(benchmark::State& state,
                                    const CloudPtr& source,
                                    const CloudPtr& noisy_target,
                                    const Eigen::Matrix4f& guess)
{
  runRegistrationBenchmark<pcl::IterativeClosestPoint<PointT, PointT>>(
      state, source, noisy_target, true, guess, "ICP did not converge");
}

void
BM_GICP_CleanTarget_NoInitialGuess(benchmark::State& state,
                                   const CloudPtr& source,
                                   const CloudPtr& target,
                                   const Eigen::Matrix4f& identity)
{
  runRegistrationBenchmark<pcl::GeneralizedIterativeClosestPoint<PointT, PointT>>(
      state, source, target, false, identity, "GICP did not converge");
}

void
BM_GICP_NoisyTarget_WithInitialGuess(benchmark::State& state,
                                     const CloudPtr& source,
                                     const CloudPtr& noisy_target,
                                     const Eigen::Matrix4f& guess)
{
  runRegistrationBenchmark<pcl::GeneralizedIterativeClosestPoint<PointT, PointT>>(
      state, source, noisy_target, true, guess, "GICP did not converge");
}

} // namespace

int
main(int argc, char** argv)
{
  bool visualize = false;
  std::vector<char*> benchmark_args;
  benchmark_args.reserve(static_cast<std::size_t>(argc));
  benchmark_args.push_back(argv[0]);
  for (int i = 1; i < argc; ++i) {
    if (std::string(argv[i]) == "--visualize") {
      visualize = true;
      continue;
    }
    benchmark_args.push_back(argv[i]);
  }
  int benchmark_argc = static_cast<int>(benchmark_args.size());

  if (benchmark_argc < 3) {
    std::cerr << "No test files given. Please download `bun0.pcd` and `bun4.pcd` and "
                 "pass their paths to the benchmark."
              << std::endl;
    return -1;
  }

  CloudPtr source = loadCloud(benchmark_args[1]);
  CloudPtr target = loadCloud(benchmark_args[2]);
  if (!source || !target) {
    std::cerr << "Failed to read test files. Please pass valid paths for `bun0.pcd` "
                 "and `bun4.pcd`."
              << std::endl;
    return -1;
  }

  const Eigen::Matrix4f identity = Eigen::Matrix4f::Identity();
  const Eigen::Matrix4f robust_guess =
      (Eigen::Isometry3f(Eigen::AngleAxisf(-0.15f, Eigen::Vector3f::UnitX()) *
                         Eigen::AngleAxisf(0.25f, Eigen::Vector3f::UnitY()) *
                         Eigen::AngleAxisf(-0.30f, Eigen::Vector3f::UnitZ()))
           .translate(Eigen::Vector3f(0.08f, -0.05f, 0.12f)))
          .matrix();

    CloudPtr clean_target = makeTransformedTarget(target, robust_guess);
    CloudPtr noisy_target = makeNoisyTarget(clean_target);

    const RegistrationResult fricp_clean_result =
      runRegistrationOnce<pcl::FastRobustIterativeClosestPoint<PointT, PointT>>(
        source, clean_target, false, identity);
    const RegistrationResult fricp_noisy_result =
      runRegistrationOnce<pcl::FastRobustIterativeClosestPoint<PointT, PointT>>(
        source, noisy_target, true, robust_guess);

    const RegistrationResult icp_clean_result =
      runRegistrationOnce<pcl::IterativeClosestPoint<PointT, PointT>>(
        source, clean_target, false, identity);
    const RegistrationResult icp_noisy_result =
      runRegistrationOnce<pcl::IterativeClosestPoint<PointT, PointT>>(
        source, noisy_target, true, robust_guess);

    const RegistrationResult gicp_clean_result =
      runRegistrationOnce<pcl::GeneralizedIterativeClosestPoint<PointT, PointT>>(
        source, clean_target, false, identity);
    const RegistrationResult gicp_noisy_result =
      runRegistrationOnce<pcl::GeneralizedIterativeClosestPoint<PointT, PointT>>(
        source, noisy_target, true, robust_guess);

  std::cout << std::fixed << std::setprecision(6);
    printResult("FRICP_CleanTarget_NoInitialGuess", fricp_clean_result);
    printResult("FRICP_NoisyTarget_WithInitialGuess", fricp_noisy_result);
    printResult("ICP_CleanTarget_NoInitialGuess", icp_clean_result);
    printResult("ICP_NoisyTarget_WithInitialGuess", icp_noisy_result);
    printResult("GICP_CleanTarget_NoInitialGuess", gicp_clean_result);
    printResult("GICP_NoisyTarget_WithInitialGuess", gicp_noisy_result);

  if (visualize) {
    visualizeAllResults(source,
                        clean_target,
                        noisy_target,
                        fricp_clean_result,
                        fricp_noisy_result,
                        icp_clean_result,
                        icp_noisy_result,
                        gicp_clean_result,
                        gicp_noisy_result);
  }

  benchmark::RegisterBenchmark(
      "FRICP_CleanTarget_NoInitialGuess",
      &BM_FRICP_Clean_NoInitialGuess,
      source,
      clean_target,
      identity)
      ->Unit(benchmark::kMillisecond);

  benchmark::RegisterBenchmark("FRICP_NoisyTarget_WithInitialGuess",
                               &BM_FRICP_NoisyTarget_WithInitialGuess,
                               source,
                               noisy_target,
                               robust_guess)
      ->Unit(benchmark::kMillisecond);

    benchmark::RegisterBenchmark("ICP_CleanTarget_NoInitialGuess",
                   &BM_ICP_CleanTarget_NoInitialGuess,
                   source,
                   clean_target,
                   identity)
      ->Unit(benchmark::kMillisecond);

    benchmark::RegisterBenchmark("ICP_NoisyTarget_WithInitialGuess",
                   &BM_ICP_NoisyTarget_WithInitialGuess,
                   source,
                   noisy_target,
                   robust_guess)
      ->Unit(benchmark::kMillisecond);

    benchmark::RegisterBenchmark("GICP_CleanTarget_NoInitialGuess",
                   &BM_GICP_CleanTarget_NoInitialGuess,
                   source,
                   clean_target,
                   identity)
      ->Unit(benchmark::kMillisecond);

    benchmark::RegisterBenchmark("GICP_NoisyTarget_WithInitialGuess",
                   &BM_GICP_NoisyTarget_WithInitialGuess,
                   source,
                   noisy_target,
                   robust_guess)
      ->Unit(benchmark::kMillisecond);

  benchmark::Initialize(&benchmark_argc, benchmark_args.data());
  benchmark::RunSpecifiedBenchmarks();

  return 0;
}
