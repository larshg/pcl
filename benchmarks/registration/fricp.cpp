#include <pcl/filters/filter.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/fricp.h>

#include <benchmark/benchmark.h>

#include <Eigen/Geometry>

#include <random>
#include <string>

namespace {
using PointT = pcl::PointXYZ;
using Cloud = pcl::PointCloud<PointT>;
using CloudPtr = Cloud::Ptr;

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
makeNoisyTarget(const CloudPtr& target, const Eigen::Matrix4f& transform)
{
  CloudPtr transformed(new Cloud);
  pcl::transformPointCloud(*target, *transformed, transform);

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

void
runFRICP(benchmark::State& state,
         const CloudPtr& source,
         const CloudPtr& target,
         bool use_guess,
         const Eigen::Matrix4f& guess)
{
  for (auto _ : state) {
    state.PauseTiming();
    pcl::FastRobustIterativeClosestPoint<PointT, PointT> reg;
    reg.setInputSource(source);
    reg.setInputTarget(target);
    reg.setMaximumIterations(60);
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
      state.SkipWithError("FRICP did not converge");
      break;
    }

    benchmark::DoNotOptimize(output.size());
    benchmark::ClobberMemory();
  }
}

void
BM_FRICP_Clean(benchmark::State& state,
               const CloudPtr& source,
               const CloudPtr& target,
               const Eigen::Matrix4f& identity)
{
  runFRICP(state, source, target, false, identity);
}

void
BM_FRICP_RobustWithGuess(benchmark::State& state,
                         const CloudPtr& source,
                         const CloudPtr& noisy_target,
                         const Eigen::Matrix4f& guess)
{
  runFRICP(state, source, noisy_target, true, guess);
}

} // namespace

int
main(int argc, char** argv)
{
  if (argc < 3) {
    std::cerr << "No test files given. Please download `bun0.pcd` and `bun4.pcd` and "
                 "pass their paths to the benchmark."
              << std::endl;
    return -1;
  }

  CloudPtr source = loadCloud(argv[1]);
  CloudPtr target = loadCloud(argv[2]);
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

  CloudPtr noisy_target = makeNoisyTarget(target, robust_guess);

  benchmark::RegisterBenchmark(
      "BM_FRICP_Clean", &BM_FRICP_Clean, source, target, identity)
      ->Unit(benchmark::kMillisecond);

  benchmark::RegisterBenchmark("BM_FRICP_RobustWithGuess",
                               &BM_FRICP_RobustWithGuess,
                               source,
                               noisy_target,
                               robust_guess)
      ->Unit(benchmark::kMillisecond);

  benchmark::Initialize(&argc, argv);
  benchmark::RunSpecifiedBenchmarks();

  return 0;
}
