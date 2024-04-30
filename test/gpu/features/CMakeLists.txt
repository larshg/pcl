set(SUBSYS_NAME tests_gpu_features)
set(SUBSYS_DESC "Point cloud library gpu feature tests")
set(SUBSYS_DEPS common io kdtree features octree search gpu_containers gpu_octree gpu_utils gpu_features)

set(DEFAULT ON)
set(build TRUE)
set(REASON "")
PCL_SUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSYS_DESC}" ${DEFAULT} "${REASON}")
PCL_SUBSYS_DEPEND(build NAME ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS} OPT_DEPS ${OPT_DEPS})

if(NOT build)
  return()
endif()

PCL_ADD_TEST(gpu_features_normals test_gpu_features_normals FILES gpu_features_normals.cpp LINK_WITH pcl_gtest pcl_common pcl_io pcl_kdtree pcl_search pcl_features pcl_gpu_containers pcl_gpu_octree pcl_gpu_features)

