#include <iostream>
#include <pcl/io/io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <boost/thread/thread.hpp>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>


int main (int argc, char** argv)
{
  pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2), cloud_filtered_blob (new pcl::PCLPointCloud2);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>), 
  										 cloud_filtered_pass (new pcl::PointCloud<pcl::PointXYZRGB>),
  										 cloud_p (new pcl::PointCloud<pcl::PointXYZRGB>), 
  										 cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);

  // Fill in the cloud data
  pcl::PCDReader reader;
  reader.read ("../data/test_pcd_28.pcd", *cloud_blob);

  int v1(0), v2(0), v3(0);
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer("3D Viewer"));
  viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);// viewer->createViewPort(0.0, 0.0, 0.4, 1.0, v1);
  viewer->setBackgroundColor(0.0, 0.0, 0.0, v1);
  viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);// viewer->createViewPort(0.4, 0.0, 0.7, 1.0, v2);
  viewer->setBackgroundColor(0.1, 0.1, 0.1, v2);
  // viewer->createViewPort(0.7, 0.0, 1.0, 1.0, v3);
  // viewer->setBackgroundColor(0.2, 0.2, 0.2, v3);
  viewer->addCoordinateSystem (0.5);
  viewer->initCameraParameters ();
  
  std::cerr << "PointCloud before filtering: " << cloud_blob->width * cloud_blob->height << " data points." << std::endl;

  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud_blob);
  sor.setLeafSize (0.01f, 0.01f, 0.01f);
  sor.filter (*cloud_filtered_blob);

  // Convert to the templated PointCloud
  pcl::fromPCLPointCloud2 (*cloud_filtered_blob, *cloud_filtered);

  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;

  // Write the downsampled version to disk
  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZRGB> ("test_pcd_downsampled.pcd", *cloud_filtered, false);

  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_filtered);
  viewer->addPointCloud<pcl::PointXYZRGB>(cloud_filtered, rgb, "Filtered cloud", v1);
  
  //  // Create the filtering object
  // pcl::PassThrough<pcl::PointXYZRGB> pass;
  // pass.setInputCloud (cloud_filtered);
  // pass.setFilterFieldName ("x");
  // pass.setFilterLimits (-100.0, 0.1);
  // //pass.setFilterLimitsNegative (true);
  // pass.filter (*cloud_filtered_pass);

  // pcl::PassThrough<pcl::PointXYZRGB> pass;
  // pass.setInputCloud (cloud_filtered);
  // pass.setFilterFieldName ("y");
  // pass.setFilterLimits (-100.0, 0.1);
  // //pass.setFilterLimitsNegative (true);
  // pass.filter (*cloud_filtered_pass);

  //   pcl::PassThrough<pcl::PointXYZRGB> pass;
  // pass.setInputCloud (cloud_filtered);
  // pass.setFilterFieldName ("z");
  // pass.setFilterLimits (-100.0, 0.1);
  // //pass.setFilterLimitsNegative (true);
  // pass.filter (*cloud_filtered_pass);

  // pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb_1(cloud_filtered_pass);
  // viewer->addPointCloud<pcl::PointXYZRGB>(cloud_filtered_pass, rgb_1, "PassThrough Filtered cloud", v3);

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (0.01);

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;

  int i = 0, nr_points = (int) cloud_filtered->points.size ();
  // While 30% of the original cloud is still there
  while (cloud_filtered->points.size () > 0.6 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the inliers
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_p);
    std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

    std::stringstream ss;
    ss << "test_pcd_plane_" << i << ".pcd";
    writer.write<pcl::PointXYZRGB> (ss.str (), *cloud_p, false);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> singleColor (cloud_p, 255 - (100*i)%256, (100*i)%256, (100*i)%256);
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud_p, singleColor, ss.str(), v2);

    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*cloud_f);
    cloud_filtered.swap (cloud_f);
    i++;
  }
  
  while(!viewer->wasStopped())
  {
    viewer->spinOnce();
    boost::this_thread::sleep (boost::posix_time::microseconds(100000));
  }
  viewer->close();

  return (0);
}