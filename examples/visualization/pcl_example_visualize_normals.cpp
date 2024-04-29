#include <pcl/features/normal_3d_omp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <iostream>

int main(int argc, char** argv) {

  if (argc < 2)
  {
    pcl::console::print_info ("Visualize normals example application.\n");
    pcl::console::print_info ("Syntax is: %s <source-pcd-file>\n", argv[0]);
    return (1);
  }

  pcl::console::print_info ("Reading %s\n", argv[1]);


  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) == -1) {
    pcl::console::print_error ("Couldn't read file %s!\n", argv[1]);
    return -1;
  }

  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud(cloud);
  ne.setNumberOfThreads(8);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZ>());
  ne.setSearchMethod(tree);
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  ne.setKSearch(100); 
  ne.setViewPoint(100, 100, 1000);
  ne.compute(*normals);


  pcl::visualization::PCLVisualizer viewer("Point Cloud Viewer");
  viewer.setBackgroundColor(0.0, 0.0, 0.0);
  viewer.addPointCloud<pcl::PointXYZ>(cloud, "cloud");
  viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(
      cloud, normals, 200, 20, "normals");
  viewer.setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
  viewer.spin();

  return 0;
}