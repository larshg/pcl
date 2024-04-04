#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/pcd_io.h> // for PCDReader

#include <benchmark/benchmark.h>

static void
BM_StatisticalOutlierRemoval(benchmark::State& state, const std::string& file)
{
  // Perform setup here
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCDReader reader;
  reader.read(file, *cloud);

  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud);
  sor.setMeanK(50);
  sor.setStddevMulThresh(1.0);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(
      new pcl::PointCloud<pcl::PointXYZ>);
  for (auto _ : state) {
    // This code gets timed
    sor.filter(*cloud_filtered);
  }
}

int
main(int argc, char** argv)
{
  if (argc < 3) {
    std::cerr
        << "No test files given. Please download `table_scene_mug_stereo_textured.pcd` "
           "and `milk_cartoon_all_small_clorox.pcd`, and pass their paths to the test."
        << std::endl;
    return (-1);
  }

  benchmark::RegisterBenchmark("BM_StatisticalOutlierRemoval_mug", &BM_StatisticalOutlierRemoval, argv[1])
      ->Unit(benchmark::kMillisecond);
  benchmark::RegisterBenchmark(
      "BM_StatisticalOutlierRemoval_milk", &BM_StatisticalOutlierRemoval, argv[2])
      ->Unit(benchmark::kMillisecond);

  benchmark::Initialize(&argc, argv);
  benchmark::RunSpecifiedBenchmarks();
}
