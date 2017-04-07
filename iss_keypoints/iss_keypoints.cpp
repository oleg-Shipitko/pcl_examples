
// STL
#include <iostream>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/impl/sift_keypoint.hpp>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/keypoints/iss_3d.h>
	
int
main(int, char** argv)
{
  std::string filename = argv[1];
  std::cout << "Reading " << filename << std::endl;
  pcl::PointCloud<pcl::PointXYZ>::Ptr object (new pcl::PointCloud<pcl::PointXYZ>);
  
  if(pcl::io::loadPCDFile<pcl::PointXYZ> (filename, *object) == -1) // load the file
  {
  PCL_ERROR ("Couldn't read file");
  return -1;
  }
  std::cout << "points: " << object->points.size () <<std::endl;



  double cloud_resolution (0.0058329);

  const float leaf = 0.005f;
  pcl::VoxelGrid<pcl::PointXYZ> grid;
  grid.setLeafSize (leaf, leaf, leaf);
  grid.setInputCloud (object);
  grid.filter (*object);

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

  pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ> iss_detector;
  iss_detector.setSearchMethod (tree);
  iss_detector.setSalientRadius (6 * cloud_resolution);
  iss_detector.setNonMaxRadius (4 * cloud_resolution);
  iss_detector.setThreshold21 (0.975);
  iss_detector.setThreshold32 (0.975);
  iss_detector.setMinNeighbors (5);
  iss_detector.setNumberOfThreads (1);
  pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints (new pcl::PointCloud<pcl::PointXYZ>);
  iss_detector.setInputCloud (object);
  iss_detector.compute (*keypoints);

  // Saving the resultant cloud 
  std::cout << "Resulting iss points are of box: " << keypoints->points.size() <<std::endl;
  pcl::io::savePCDFileASCII("iss_keypoints.pcd", *keypoints);

  // Visualization of keypoints along with the original cloud
  pcl::visualization::PCLVisualizer viewer("PCL Viewer");
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_color_handler (keypoints, 0, 255, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_handler (object, 255, 0, 0);
  viewer.setBackgroundColor( 0.0, 0.0, 0.0 );
  viewer.addPointCloud(object, cloud_color_handler, "cloud");
  viewer.addPointCloud(keypoints, keypoints_color_handler, "keypoints");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints");
  
  while(!viewer.wasStopped())
  {
    viewer.spinOnce ();
  }
  return 0;
  
}