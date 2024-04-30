/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  Author: Anatoly Baskeheev, Itseez Ltd, (myname.mysurname@mycompany.com)
 */


#include <gtest/gtest.h>
//   
#include <thrust/host_vector.h>
#include <thrust/device_vector.h>
#include <thrust/copy.h>
#include <thrust/iterator/zip_iterator.h>
#include <thrust/tuple.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

struct filter {
  __host__ __device__ bool
  operator()(thrust::tuple<float, float, float> point)
  {
    float x = point.get<0>();
    if (minX < x && x < maxX)
      return true;
    else
      return false;
  }

  float maxX;
  float minX;
};

constexpr uint32_t COUNT = 1000000;
constexpr float max_val = 1000.0;

TEST(PCL_cuda, pointcloud)
{
  thrust::host_vector<float> h_x;
  thrust::host_vector<float> h_y;
  thrust::host_vector<float> h_z;

  for (int i = 0; i < COUNT; i++)
  {
    h_x.push_back(static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / max_val)));
    h_y.push_back(static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/max_val)));
    h_z.push_back(static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/max_val)));
  }

  thrust::device_vector<float> d_x;
  thrust::device_vector<float> d_y;
  thrust::device_vector<float> d_z;

  d_x = h_x;
  d_y = h_y;
  d_z = h_z;

  auto zipBeginIt = thrust::make_zip_iterator(thrust::make_tuple(d_x.begin(), d_y.begin(), d_z.begin()));
  auto zipEndIt = thrust::make_zip_iterator(thrust::make_tuple(d_x.end(), d_y.end(), d_z.end()));

  thrust::device_vector<float> d_r_x(COUNT);
  thrust::device_vector<float> d_r_y(COUNT);
  thrust::device_vector<float> d_r_z(COUNT);

  auto zipResult = thrust::make_zip_iterator(
      thrust::make_tuple(d_r_x.begin(), d_r_y.begin(), d_r_z.begin()));

  auto filterIns = filter();
  filterIns.minX = 100;
  filterIns.maxX = 900;

  auto result = thrust::copy_if(zipBeginIt, zipEndIt, zipResult, filterIns);
  //auto result = thrust::copy(zipBeginIt, zipEndIt, zipResult);

  std::cout << "there are: " << result - zipResult << " left\n";

  //for (int i = 0; i < 10; i++) {
  //  std::cout << "x: " << d_r_x[i] << std::endl;
  //  std::cout << "y: " << d_r_y[i] << std::endl;
  //  std::cout << "z: " << d_r_z[i] << std::endl;
  //}
}

TEST(PCL_test_cuda, test_pcl)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  for (int i = 0; i < COUNT; i++) {
    cloud->push_back(pcl::PointXYZ(
        static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / max_val)),
        static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / max_val)),
        static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / max_val))));
  }

  pcl::PointCloud<pcl::PointXYZ> result;

  pcl::PassThrough<pcl::PointXYZ> passthrough;
  passthrough.setFilterFieldName("x");
  passthrough.setFilterLimits(100, 900);
  passthrough.setInputCloud(cloud);
  passthrough.filter(result);

  std::cout << "filtered cloud has " << result.size() << " points\n";
}

int
main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */

