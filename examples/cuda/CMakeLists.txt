
find_package(Thrust REQUIRED CONFIG)
thrust_create_target(Thrust)

if(Thrust_FOUND)
  message("Thrust found")
endif()

if(TARGET Thrust)
  message("Thrust target exist")
endif()

PCL_ADD_EXAMPLE(pcl_example_cuda_common FILES example_cuda_common.cu LINK_WITH pcl_common pcl_io pcl_filters pcl_cuda_common Thrust CUDA::cudart CUDA)
target_compile_options(pcl_example_cuda_common PRIVATE $<$<COMPILE_LANGUAGE:CUDA>: ${GEN_CODE} --expt-relaxed-constexpr>)
