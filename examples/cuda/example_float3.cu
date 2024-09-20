#include <pcl/common/time.h>
#include <pcl/console/parse.h>
#include <pcl/cuda/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <thrust/device_vector.h>
#include <thrust/host_vector.h>

#include <chrono>

struct passthrough {

  float maxZ_;

  passthrough(float maxZ) : maxZ_{maxZ} {}

  __host__ __device__ bool
  operator()(const float3 pt)
  {
    return pt.z < maxZ_;
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

thrust::host_vector<float3>
filterGPU(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float maxZ)
{
  thrust::host_vector<float3> vec;
  vec.reserve(cloud->size());

  {
    pcl::ScopeTime hostvector("Fill Host vector");
    for each (const auto& pt in* cloud) {
      vec.push_back({pt.x, pt.y, pt.z});
    }
  }
  thrust::device_vector<float3> d_vec;
  {
    pcl::ScopeTime copyToDevice("Copying to device");
    d_vec = vec;
  }
  thrust::device_vector<float3> d_result(d_vec.size());

  size_t resultCount;
  {
    pcl::ScopeTime cudaCall("Copy if");

    resultCount = thrust::copy_if(
                      d_vec.begin(), d_vec.end(), d_result.begin(), passthrough(maxZ)) -
                  d_result.begin();
  }
  d_result.resize(resultCount);

  thrust::host_vector<float3> h_result;

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

int
main(int argc, char** argv)
{
  auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  std::string fileName;
  pcl::console::parse<std::string>(argc, argv, "-f", fileName);

  float maxZ;
  pcl::console::parse(argc, argv, "-maxZ", maxZ);

  pcl::io::loadPCDFile(fileName, *cloud);

  warmUpGPU();
  thrust::host_vector<float3> filteredCloud;
  {
    pcl::ScopeTime filterGPUtime("Filter GPU");
    filteredCloud = filterGPU(cloud, maxZ);
  }
  std::cout << filteredCloud.size() << "\n";

  //{
  //  pcl::ScopeTime filterCPUTime("Filter CPU");
  //  auto cpufilteredCloud = filterCPUpassthrough(cloud, maxZ);
  //  std::cout << cpufilteredCloud->size() << "\n";
  //}

  return 0;
}
