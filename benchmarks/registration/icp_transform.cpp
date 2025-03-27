#include <pcl/registration/icp.h>
#include <pcl/common/generate.h>
#include <pcl/common/transforms.h>

#include <benchmark/benchmark.h>

using namespace pcl::common;


class ICPTestHelper : public pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> {
public:
  using IterativeClosestPoint::transformCloud;
};

static void
BM_ICPTransform(benchmark::State& state)
{
  // Perform setup here
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  const auto pointCount = state.range(0);

  CloudGenerator<pcl::PointXYZ, UniformGenerator<float>> generator;
  auto seed = static_cast<std::uint32_t>(time(nullptr));
  UniformGenerator<float>::Parameters x_params(-5.0, 5.0, seed++);
  generator.setParametersForX(x_params);
  UniformGenerator<float>::Parameters y_params(-5.0, 5.0, seed++);
  generator.setParametersForY(y_params);
  UniformGenerator<float>::Parameters z_params(5.0, 5.0, seed++);
  generator.setParametersForZ(z_params);

  generator.fill(pointCount, 1, *cloud);

  ICPTestHelper icpTestHelper;

  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  transform = transform.rotate(Eigen::AngleAxisf(0.1, Eigen::Vector3f::UnitZ()));
  transform.translation() << 0.1, 0.1, 0.1;
  

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed(
      new pcl::PointCloud<pcl::PointXYZ>);
  cloud_transformed->resize(cloud->size());
  for (auto _ : state) {
    // This code gets timed
    icpTestHelper.transformCloud(
        *cloud, *cloud_transformed, transform.matrix());
  }
}

static void
BM_CommonTransform(benchmark::State& state)
{
  // Perform setup here
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  const auto pointCount = state.range(0);

  CloudGenerator<pcl::PointXYZ, UniformGenerator<float>> generator;
  auto seed = static_cast<std::uint32_t>(time(nullptr));
  UniformGenerator<float>::Parameters x_params(-5.0, 5.0, seed++);
  generator.setParametersForX(x_params);
  UniformGenerator<float>::Parameters y_params(-5.0, 5.0, seed++);
  generator.setParametersForY(y_params);
  UniformGenerator<float>::Parameters z_params(5.0, 5.0, seed++);
  generator.setParametersForZ(z_params);

  generator.fill(pointCount, 1, *cloud);

  ICPTestHelper icpTestHelper;

  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  transform = transform.rotate(Eigen::AngleAxisf(0.1, Eigen::Vector3f::UnitZ()));
  transform.translation() << 0.1, 0.1, 0.1;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed(
      new pcl::PointCloud<pcl::PointXYZ>);
  cloud_transformed->resize(cloud->size());
  for (auto _ : state) {
    // This code gets timed
    pcl::transformPointCloud(*cloud, *cloud_transformed, transform.matrix());
  }
}

int
main(int argc, char** argv)
{

  benchmark::RegisterBenchmark("BM_icp_transform", &BM_ICPTransform)
      ->RangeMultiplier(10)
      ->Range(100, 100000000)
      ->Unit(benchmark::kMicrosecond);

  
  benchmark::RegisterBenchmark("BM_common_transform", &BM_CommonTransform)
      ->RangeMultiplier(10)
      ->Range(100, 100000000)
      ->Unit(benchmark::kMicrosecond);

  benchmark::Initialize(&argc, argv);
  benchmark::RunSpecifiedBenchmarks();
}
