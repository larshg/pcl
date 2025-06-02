#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/shared_ptr.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>

namespace nb = nanobind;

NB_MODULE(pcl_filters_ext, m)
{
#if 0 // abstract class
  nb::class_<pcl::Filter<pcl::PointXYZ>(m, "FilterXYZ")
      .def(nb::init<>())
      .def("filter", &pcl::Filter<pcl::PointXYZ>::filter)
      ;
#endif
#if 0 // abstract class
  nb::class_<pcl::FilterIndices<pcl::PointXYZ>(m, "FilterIndicesXYZ")
      .def(nb::init<>())
      .def("setNegative", &pcl::FilterIndices<pcl::PointXYZ>::setNegative)
      ;
#endif
  nb::class_<pcl::PassThrough<pcl::PointXYZ>>(m, "PassThroughXYZ")
      .def(nb::init<>())
      .def("setInputCloud", &pcl::PassThrough<pcl::PointXYZ>::setInputCloud)
      .def("setFilterLimits", &pcl::PassThrough<pcl::PointXYZ>::setFilterLimits)
      .def("setFilterFieldName", &pcl::PassThrough<pcl::PointXYZ>::setFilterFieldName)
      .def("setNegative", &pcl::FilterIndices<pcl::PointXYZ>::setNegative)
      .def("filter",
           nb::overload_cast<pcl::PointCloud<pcl::PointXYZ>&>(
               &pcl::Filter<pcl::PointXYZ>::filter))
      ;

  nb::class_<pcl::PassThrough<pcl::PointXYZRGBA>>(m, "PassThroughXYZRGBA")
      .def(nb::init<>())
      .def("setInputCloud", &pcl::PassThrough<pcl::PointXYZRGBA>::setInputCloud)
      .def("setFilterLimits", &pcl::PassThrough<pcl::PointXYZRGBA>::setFilterLimits)
      .def("setFilterFieldName",
           &pcl::PassThrough<pcl::PointXYZRGBA>::setFilterFieldName)
      .def("setNegative",
           &pcl::FilterIndices<pcl::PointXYZRGBA>::setNegative)
      .def("filter",
           nb::overload_cast<pcl::PointCloud<pcl::PointXYZRGBA>&>(
               &pcl::Filter<pcl::PointXYZRGBA>::filter))
      ;
}
