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

#include <iostream>
#include <fstream>
#include <numeric>


#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/gpu/containers/initialization.h>
#include <pcl/gpu/features/features.hpp>
#include <pcl/gpu/octree/octree.hpp>
#include <pcl/gpu/containers/device_array.h>
#include <pcl/search/kdtree.h>

using namespace pcl;
using namespace pcl::gpu;

using KdTreePtr = search::KdTree<PointXYZ>::Ptr;

PointCloud<PointXYZ> cloud;
Indices indices;
KdTreePtr tree;

TEST(PCL_NormalsGPU, RadiusSearch)
{
  pcl::gpu::DeviceArray<pcl::PointXYZ> gpu_cloud;
  gpu_cloud.upload(cloud.data(), cloud.size());

  pcl::gpu::Feature::Indices gpu_ind;
  gpu_ind.create(indices.size());
  gpu_ind.upload(indices);

  pcl::gpu::NormalEstimation gne;
  gne.setInputCloud(gpu_cloud);
  gne.setIndices(gpu_ind);
  gne.setRadiusSearch(100, indices.size()); // setRadiusSearch(float radius, int
                                       // max_results);  What is max_results??

  pcl::gpu::NormalEstimation::Normals dev_normals;
  gne.compute(dev_normals);

  std::vector<pcl::Normal> gpu_normals;
  pcl::PointCloud<pcl::Normal>::Ptr cloud_gpu_normals(new pcl::PointCloud<pcl::Normal>);
  dev_normals.download(cloud_gpu_normals->points);

  for (size_t i = 0; i < gpu_normals.size(); ++i) {
    cloud_gpu_normals->points[i].normal_x = gpu_normals[i].normal_x;
    cloud_gpu_normals->points[i].normal_y = gpu_normals[i].normal_y;
    cloud_gpu_normals->points[i].normal_z = gpu_normals[i].normal_z;
    cloud_gpu_normals->points[i].curvature = gpu_normals[i].curvature;
  }

  EXPECT_EQ(cloud_gpu_normals->size(), indices.size());

  for (const auto& point : cloud_gpu_normals->points) {
    EXPECT_NEAR(point.normal[0], -0.035592, 1e-4);
    EXPECT_NEAR(point.normal[1], -0.369596, 1e-4);
    EXPECT_NEAR(point.normal[2], -0.928511, 1e-4);
    EXPECT_NEAR(point.curvature, 0.0693136, 1e-4);
  }
}

/* ---[ */
int
main (int argc, char** argv)
{
  pcl::gpu::setDevice(0);
  pcl::gpu::printShortCudaDeviceInfo(0);
  if (argc < 2) {
    std::cerr << "No test file given. Please download `bun0.pcd` and pass its path to "
                 "the test."
              << std::endl;
    return (-1);
  }

  if (io::loadPCDFile<PointXYZ>(argv[1], cloud) < 0) {
    std::cerr << "Failed to read test file. Please download `bun0.pcd` and pass its "
                 "path to the test."
              << std::endl;
    return (-1);
  }

  indices.resize(cloud.size());
  for (int i = 0; i < static_cast<int>(indices.size()); ++i)
    indices[i] = i;

  tree.reset(new search::KdTree<PointXYZ>(false));
  tree->setInputCloud(cloud.makeShared());

  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */

