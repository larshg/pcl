/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception Inc.
 *
 *  All rights reserved
 */

#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/test/gtest.h>
#include <pcl/pcl_config.h>
#include <pcl/point_types.h>
#include <pcl/type_traits.h>

std::string stp_file;

TEST(PCL, STPLoadFileTest)
{
  pcl::PolygonMesh mesh;
  pcl::io::loadPolygonFileSTP(stp_file, mesh);

  EXPECT_EQ(mesh.cloud.width, 188);
  EXPECT_EQ(mesh.polygons.size(), 168);
}

/* ---[ */
int
main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  if (argc < 2) {
    std::cerr << "Please add the path to TestObject.stp to this test." << std::endl;
    return (-1);
  }
  stp_file = argv[1];

  return (RUN_ALL_TESTS());
}
