set(SUBSYS_NAME tests_cuda_common)
set(SUBSYS_DESC "Point cloud library cuda common tests")
set(SUBSYS_DEPS common filters cuda_common)

PCL_SUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSYS_DESC}" ON)
PCL_SUBSYS_DEPEND(build NAME ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS} OPT_DEPS ${OPT_DEPS})

if(NOT build)
  return()
endif()

PCL_ADD_TEST(cuda_common test_cuda_common FILES cuda_common.cu LINK_WITH pcl_gtest pcl_common pcl_filters)

#Ensures that CUDA is added and the project can compile as it uses cuda calls.
#set_source_files_properties(src/cuda_common.cpp PROPERTIES LANGUAGE CUDA)
target_include_directories(test_cuda_common PRIVATE ${CUDA_TOOLKIT_INCLUDE})
target_link_libraries(test_cuda_common CUDA::cudart)
target_compile_options(test_cuda_common PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:--extended-lambda>)
