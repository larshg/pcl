#define BOOST_DISABLE_CURRENT_LOCATION

#include <pcl/console/parse.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/time.h>
#include <pcl/common/point_tests.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/experimental/functor_filter.h>

#include <pcl/cuda/point_cloud.h>

#include <thrust/host_vector.h>
#include <thrust/device_vector.h>

#include <chrono>

struct passthrough {
  
      const float maxZ_;

      passthrough(const float maxZ) : maxZ_{maxZ} {}

      __host__ __device__
      bool
      operator()(const pcl::PointXYZ pt)
      {
        return isfinite(pt.x) && isfinite(pt.y) &&
               isfinite(pt.z) && pt.z < maxZ_;
      }
};

__global__ void
kernel_cudaWarmUpGPU()
{
  int ind = blockIdx.x * blockDim.x + threadIdx.x;
  ind = ind + 1;
}

cudaError_t
cudaWarmUpGPU()
{
  kernel_cudaWarmUpGPU<<<1, 1>>>();
  cudaDeviceSynchronize();
  return cudaGetLastError();
}

void
warmUpGPU()
{
  cudaError_t err = ::cudaSuccess;
  err = cudaSetDevice(0);
  if (err != ::cudaSuccess)
    return;

  err = cudaWarmUpGPU();
  if (err != ::cudaSuccess)
    return;
}


thrust::host_vector<pcl::PointXYZ>
filterGPU(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float maxZ)
{
  
  thrust::device_vector<pcl::PointXYZ> d_vec;
  {
    pcl::ScopeTime copyToDevice("Copying to device");
    d_vec = cloud->points;
  }
  thrust::device_vector<pcl::PointXYZ> d_result(d_vec.size());
  

  size_t resultCount;
  {
    pcl::ScopeTime cudaCall("Copy if");
  
  resultCount =
      thrust::copy_if(d_vec.begin(), d_vec.end(), d_result.begin(), passthrough(maxZ)) -
      d_result.begin();

  }
  d_result.resize(resultCount);

  thrust::host_vector<pcl::PointXYZ> h_result;

  {
    pcl::ScopeTime copyFromDeivce("Copy to host");
    h_result = d_result;
  }

  return h_result;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr
filterCPUpassthrough(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float maxZ)
{
  auto result = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0, maxZ);
  pass.filter(*result);
  return result;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr
filterCPUfunctor(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float maxZ)
{
  auto result = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  pcl::experimental::FilterFunction<pcl::PointXYZ> filter;
  filter = [=](const pcl::PointCloud<pcl::PointXYZ>& cloud, pcl::index_t idx) {
    
    return (pcl::isXYZFinite(cloud[idx]) && cloud[idx].z < maxZ);
  };
  // build the filter
  pcl::experimental::FunctionFilter<pcl::PointXYZ> func_filter(filter);
  func_filter.setInputCloud(cloud);
  func_filter.filter(*result);

  return result;
}

int
main(int argc, char** argv)
{
  auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  std::string fileName;
  pcl::console::parse<std::string>(argc, argv, "-f", fileName);

  float maxZ{0.0};
  pcl::console::parse(argc, argv, "-maxZ", maxZ);

  pcl::io::loadPCDFile(fileName, *cloud);

  warmUpGPU();
  thrust::host_vector<pcl::PointXYZ> filteredCloud;
  {
    pcl::ScopeTime filterGPUtime("Filter GPU");
    filteredCloud = filterGPU(cloud, maxZ);
  }
  std::cout << "Original cloud: " << cloud->size() << "\n";
  std::cout << "GPU Filter" << filteredCloud.size() << "\n";

  {
    pcl::ScopeTime filterCPUTime("Filter CPU");
    auto cpufilteredCloud = filterCPUpassthrough(cloud, maxZ);
    std::cout << "Passthrough: " << cpufilteredCloud->size() << "\n";
  }

  {
    pcl::ScopeTime filterFunctor("Filter Functor CPU");
    auto cpuFilterFuncCloud = filterCPUfunctor(cloud, maxZ);
    std::cout << "Func CPU: " << cpuFilterFuncCloud->size() << "\n";
  }

  return 0;
}
