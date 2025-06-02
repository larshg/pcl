#include <nanobind/nanobind.h>
#include <nanobind/make_iterator.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace nb = nanobind;

NB_MODULE(pcl_common_ext, m)
{
  nb::class_<pcl::PointXYZ>(m, "PointXYZ")
      .def(nb::init())
      .def(nb::init<float, float, float>())
      .def_rw("x", &pcl::PointXYZ::x)
      .def_rw("y", &pcl::PointXYZ::y)
      .def_rw("z", &pcl::PointXYZ::z)
      .def("__repr__", [](const pcl::PointXYZ& p) {
        return nb::str("PointXYZ(x={}, y={}, z={})").format(p.x, p.y, p.z);
      });

  nb::class_<pcl::PointXYZRGBA>(m, "PointXYZRGBA")
      .def(nb::init())
      .def(nb::init<float, float, float>())
      .def_rw("x", &pcl::PointXYZRGBA::x)
      .def_rw("y", &pcl::PointXYZRGBA::y)
      .def_rw("z", &pcl::PointXYZRGBA::z)
      .def("__repr__", [](const pcl::PointXYZRGBA& p) {
        return nb::str("PointXYZ(x={}, y={}, z={}, r={}, g={}, b={}, a={})").format(p.x, p.y, p.z, p.r, p.g, p.b, p.a);
      });


  // TODO must avoid code duplication, maybe write macro?
  nb::class_<pcl::PointCloud<pcl::PointXYZ>>(m, "PointcloudXYZ")
      .def(nb::init<>())
      .def("append", &pcl::PointCloud<pcl::PointXYZ>::push_back)
      .def("at", nb::overload_cast<std::size_t>(&pcl::PointCloud<pcl::PointXYZ>::at), "Get a point from the cloud")
      .def("__getitem__",
             [](pcl::PointCloud<pcl::PointXYZ> &cloud, std::size_t i) -> pcl::PointXYZ& {
                 return cloud[i];
             }
             //, TODO to discuss: do we want nb::rv_policy::reference_internal here? See https://nanobind.readthedocs.io/en/latest/api_extra.html#vector-bindings
             )
      .def("__len__", [](const pcl::PointCloud<pcl::PointXYZ> &v) { return v.size(); })
      .def("resize", nb::overload_cast<std::size_t>(&pcl::PointCloud<pcl::PointXYZ>::resize))
      .def("clear", [](pcl::PointCloud<pcl::PointXYZ> &v) { v.clear(); })
      .def("__iter__",
        [](const pcl::PointCloud<pcl::PointXYZ> &v) {
            return nb::make_iterator(nb::type<pcl::PointCloud<pcl::PointXYZ>>(), "iterator",
                                     v.begin(), v.end());
        }, nb::keep_alive<0, 1>());

  nb::class_<pcl::PointCloud<pcl::PointXYZRGBA>>(m, "PointcloudXYZRGBA")
      .def(nb::init<>())
      .def("append", &pcl::PointCloud<pcl::PointXYZRGBA>::push_back)
      .def("at",
           nb::overload_cast<std::size_t>(&pcl::PointCloud<pcl::PointXYZRGBA>::at),
           "Get a point from the cloud")
      .def("__getitem__",
           [](pcl::PointCloud<pcl::PointXYZRGBA>& cloud,
              std::size_t i) -> pcl::PointXYZRGBA& {
             return cloud[i];
           }
           //, TODO to discuss: do we want nb::rv_policy::reference_internal here? See https://nanobind.readthedocs.io/en/latest/api_extra.html#vector-bindings
           )
      .def("__len__", [](const pcl::PointCloud<pcl::PointXYZRGBA> &v) { return v.size(); })
      .def("resize",
           nb::overload_cast<std::size_t>(&pcl::PointCloud<pcl::PointXYZRGBA>::resize))
      .def("clear", [](pcl::PointCloud<pcl::PointXYZRGBA> &v) { v.clear(); })
      .def(
          "__iter__",
          [](const pcl::PointCloud<pcl::PointXYZRGBA>& v) {
            return nb::make_iterator(nb::type<pcl::PointCloud<pcl::PointXYZRGBA>>(),
                                     "iterator",
                                     v.begin(),
                                     v.end());
          },
          nb::keep_alive<0, 1>());
}
