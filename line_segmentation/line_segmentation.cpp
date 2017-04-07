#include <iostream>
#include <pcl/io/io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_parallel_line.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <boost/thread/thread.hpp>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>


int main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>),
  								                       cloud_p (new pcl::PointCloud<pcl::PointXYZRGB>), 
  										                   cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);

  // Fill in the cloud data
  pcl::PCDReader reader;
  reader.read ("rgb_edges_scene.pcd", *cloud);
  
  pcl::PCDWriter writer;

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PARALLEL_LINE);
  seg.setMethodType (pcl::SAC_RANSAC);
  Eigen::Vector3f parralel_to_x;
  parralel_to_x(0) = 1;
  parralel_to_x(1) = 0;
  parralel_to_x(2) = 0;
  seg.setAxis(parralel_to_x);
  seg.setEpsAngle(0.5);
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (0.01);

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;

  int i = 0, nr_points = (int) cloud->points.size ();
  // While 60% of the original cloud is still there
  while (cloud->points.size () > 0.6 * nr_points)
  {
    // Segment the largest line from the remaining cloud
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
    
    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a line model for the given dataset." << std::endl;
      break;
    }

    // Extract the inliers
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_p);
    std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

    std::stringstream ss;
    ss << "test_pcd_line_" << i << ".pcd";
    writer.write<pcl::PointXYZRGB> (ss.str (), *cloud_p, false);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> singleColor (cloud_p, 255 - (100*i)%256, (100*i)%256, (100*i)%256);

    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*cloud_f);
    cloud.swap (cloud_f);
    i++;
  }

  return (0);
}