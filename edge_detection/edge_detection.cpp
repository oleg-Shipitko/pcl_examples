  // Point Cloud Library
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <boost/thread/thread.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/registration/icp.h>
#include <pcl/common/time.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/console/print.h>
#include <pcl/features/vfh.h>
#include <pcl/features/organized_edge_detection.h>
// #include <pcl/features/rsd.h>
#include <Eigen/Core>
// Standatrd libraries
#include <iostream>
#include <vector>

int main (int argc, char** argv)
{
  std::string filename = argv[1];
  std::cout << "Reading " << filename << std::endl;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
  if(pcl::io::loadPCDFile<pcl::PointXYZRGBA> (filename, *cloud) == -1) // load the file
  {
    PCL_ERROR ("Couldn't read file");
    return -1;
  }
  std::cout << "points: " << cloud->points.size () <<std::endl;


  pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> ne;
  ne.setInputCloud (cloud);
  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA> ());
  ne.setSearchMethod (tree);
  // Output datasets
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  // Use all neighbors in a sphere of radius 3cm
  ne.setRadiusSearch (0.01);
  // Compute the features
  ne.compute (*cloud_normals);

  // cloud_normals->points.size () should have the same size as the input cloud->points.size ()*
  pcl::PCDWriter writer;
  writer.write<pcl::Normal> ("normals_scene.pcd", *cloud_normals, false); 

  pcl::OrganizedEdgeFromRGBNormals<pcl::PointXYZRGBA, pcl::Normal, pcl::Label> oed;
  oed.setInputNormals (cloud_normals);
  oed.setInputCloud (cloud);
  oed.setDepthDisconThreshold (0.03); // 2cm
  oed.setMaxSearchNeighbors (100);
  pcl::PointCloud<pcl::Label> labels;
  std::vector<pcl::PointIndices> label_indices;
  oed.compute (labels, label_indices);

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr occluding_edges (new pcl::PointCloud<pcl::PointXYZRGBA>),
        occluded_edges (new pcl::PointCloud<pcl::PointXYZRGBA>),
        boundary_edges (new pcl::PointCloud<pcl::PointXYZRGBA>),
        high_curvature_edges (new pcl::PointCloud<pcl::PointXYZRGBA>),
        rgb_edges (new pcl::PointCloud<pcl::PointXYZRGBA>);

  pcl::copyPointCloud (*cloud, label_indices[0].indices, *boundary_edges);
  pcl::copyPointCloud (*cloud, label_indices[1].indices, *occluding_edges);
  pcl::copyPointCloud (*cloud, label_indices[2].indices, *occluded_edges);
  pcl::copyPointCloud (*cloud, label_indices[3].indices, *high_curvature_edges);
  pcl::copyPointCloud (*cloud, label_indices[4].indices, *rgb_edges);
  
  writer.write<pcl::PointXYZRGBA> ("occluding_edges_scene.pcd", *occluding_edges, false); 
  writer.write<pcl::PointXYZRGBA> ("boundary_edges_scene.pcd", *boundary_edges, false); 
  writer.write<pcl::PointXYZRGBA> ("occluded_edges_scene.pcd", *occluded_edges, false); 
  writer.write<pcl::PointXYZRGBA> ("high_curvature_edges_scene.pcd", *high_curvature_edges, false);
  writer.write<pcl::PointXYZRGBA> ("rgb_edges_scene.pcd", *rgb_edges, false);
}
  