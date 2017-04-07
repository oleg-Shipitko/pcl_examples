
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
#include <pcl/keypoints/harris_3d.h>
	
int
main(int, char** argv)
{
  std::string filename = argv[1];
  std::cout << "Reading " << filename << std::endl;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  if(pcl::io::loadPCDFile<pcl::PointXYZ> (filename, *cloud) == -1) // load the file
  {
  PCL_ERROR ("Couldn't read file");
  return -1;
  }
  std::cout << "points: " << cloud->points.size () <<std::endl;

  const float leaf = 0.005f;
  pcl::VoxelGrid<pcl::PointXYZ> grid;
  grid.setLeafSize (leaf, leaf, leaf);
  grid.setInputCloud (cloud);
  grid.filter (*cloud);

  // Estimate the keypoints
  pcl::HarrisKeypoint3D<pcl::PointXYZ,pcl::PointXYZI> detector;
  detector.setNonMaxSupression (true);
  detector.setRadius (0.04);
  detector.setRefine(false);
  detector.setInputCloud(cloud);
  pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints (new pcl::PointCloud<pcl::PointXYZI>);
  detector.compute(*keypoints);

  pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints3D(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointXYZ tmp;

  for(pcl::PointCloud<pcl::PointXYZI>::iterator i = keypoints->begin(); i!= keypoints->end(); i++){
    tmp = pcl::PointXYZ((*i).x,(*i).y,(*i).z);
    keypoints3D->push_back(tmp);
  }

  // Saving the resultant cloud 
  std::cout << "Resulting harris points are of size: " << keypoints->points.size() <<std::endl;
  pcl::io::savePCDFileASCII("harris_keypoints.pcd", *keypoints3D);
  
  // Visualization of keypoints along with the original cloud
  pcl::visualization::PCLVisualizer viewer("PCL Viewer");
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_color_handler (keypoints3D, 0, 255, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_handler (cloud, 255, 0, 0);
  viewer.setBackgroundColor( 0.0, 0.0, 0.0 );
  viewer.addPointCloud(cloud, cloud_color_handler, "cloud");
  viewer.addPointCloud(keypoints3D, keypoints_color_handler, "keypoints");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints");
  
  while(!viewer.wasStopped())
  {
  viewer.spinOnce ();
  }
  
  return 0;
  
}