/*
 * render_views_tesselated_sphere.h
 *
 *  Created on: Dec 23, 2011
 *      Author: aitor
 */

#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <vtkPolyData.h>
#include <vtkSmartPointer.h>

#include <functional>

namespace pcl {
namespace visualization {

/**
 * \brief @b Class to render synthetic views of a 3D mesh using a tessellated sphere
 * \author Aitor Aldoma
 * \ingroup visualization
 */
class PCL_EXPORTS RenderViewsTesselatedSphere {
private:
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> poses_;
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> generated_views_;
  std::vector<float> entropies_;
  int resolution_;
  int tesselation_level_;
  bool use_vertices_;
  float view_angle_;
  float radius_sphere_;
  bool compute_entropy_;
  vtkSmartPointer<vtkPolyData> polydata_;
  bool gen_organized_;
  std::function<bool(const Eigen::Vector3f&)> campos_constraints_func_;

  struct camPosConstraintsAllTrue {
    bool
    operator()(const Eigen::Vector3f& /*pos*/) const
    {
      return true;
    };
  };

public:
  RenderViewsTesselatedSphere() : campos_constraints_func_(camPosConstraintsAllTrue())
  {
    resolution_ = 150;
    tesselation_level_ = 1;
    use_vertices_ = false;
    view_angle_ = 57;
    radius_sphere_ = 1.f;
    compute_entropy_ = false;
    gen_organized_ = false;
  }

  void setCamPosConstraints(std::function<bool(const Eigen::Vector3f&)>& bb)
  {
    campos_constraints_func_ = bb;
  }
  void setGenOrganized(bool b) { gen_organized_ = b; }
  void setResolution(int res) { resolution_ = res; }
  void setUseVertices(bool use) { use_vertices_ = use; }
  void setRadiusSphere(float radius) { radius_sphere_ = radius; }
  void setComputeEntropies(bool compute) { compute_entropy_ = compute; }
  void setTesselationLevel(int level) { tesselation_level_ = level; }
  void setViewAngle(float angle) { view_angle_ = angle; }

  void addModelFromPolyData(vtkSmartPointer<vtkPolyData>& polydata)
  {
    polydata_ = polydata;
  }

  void generateViews();

  void getPoses(
      std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>& poses)
  {
    poses = poses_;
  }

  void getViews(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& views)
  {
    views = generated_views_;
  }

  void getEntropies(std::vector<float>& entropies)
  {
    entropies = entropies_;
  }
};

} // namespace visualization
} // namespace pcl