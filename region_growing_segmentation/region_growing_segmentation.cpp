#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>

int main (int argc, char** argv)
{
  pcl::search::Search <pcl::PointXYZRGB>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB> > (new pcl::search::KdTree<pcl::PointXYZRGB>);

  pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud <pcl::PointXYZRGB>),
                                          cloud_p (new pcl::PointCloud<pcl::PointXYZRGB>),
                                          cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB>);
  if ( pcl::io::loadPCDFile <pcl::PointXYZRGB> (argv[1], *cloud) == -1 )
  {
    std::cout << "Cloud reading failed." << std::endl;
    return (-1);
  }

  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<pcl::PointXYZRGB> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (0.01f, 0.01f, 0.01f);
  sor.filter (*cloud);

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  Eigen::Vector3f perpendicular_to(1, 1, 0); 
  seg.setAxis(perpendicular_to);
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (0.02);
  seg.setInputCloud (cloud);
  seg.segment (*inliers, *coefficients);
  if (inliers->indices.size () == 0)
  {
    std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
  }

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  // Extract the inliers
  extract.setInputCloud (cloud);
  extract.setIndices (inliers);
  extract.setNegative (false);
  extract.filter (*cloud_plane);
  std::cerr << "PointCloud representing the planar component: " << cloud_plane->width * cloud_plane->height << " data points." << std::endl;
  
  pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
  reg.setInputCloud (cloud_plane);
  reg.setSearchMethod (tree);
  reg.setDistanceThreshold (0.1); // Distance between two points to consider them to be neighbours
  reg.setPointColorThreshold (3); // Threshold to deside that two points belongs to one cluster
  reg.setRegionColorThreshold (5); // Threshold to deside that two clusters are different
  reg.setMinClusterSize (600);

  std::vector <pcl::PointIndices> clusters;
  reg.extract (clusters);

  std::cout << "Number of clusters: " << clusters.size() << std::endl;

  pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();
  int v1(0), v2(0);
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer("3D Viewer"));
  viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
  viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
  viewer->setBackgroundColor(0.0, 0.0, 0.0, v1);
  viewer->addCoordinateSystem (0.5);
  viewer->initCameraParameters ();
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(colored_cloud);
  viewer->addPointCloud<pcl::PointXYZRGB>(colored_cloud, rgb, "Colored cloud", v2);
  pcl::io::savePCDFileASCII("clusters.pcd", *colored_cloud);

  for (int i = 0; i < clusters.size(); ++i)
  {
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices (clusters[i]));
    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud (cloud_plane);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_p);

    sor.setInputCloud (cloud_p);
    sor.setLeafSize (0.03f, 0.03f, 0.03f);
    sor.filter (*cloud_p);

    // // MovingLeastSquares upsampling VOXEL_GRID_DILATION
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr upsampled_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    // pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    // pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGB> mls;
    // mls.setComputeNormals(false);
    // mls.setInputCloud(cloud_p);
    // mls.setPolynomialFit(false);
    // mls.setSearchMethod(tree);
    // mls.setSearchRadius(0.03);
    // mls.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZRGB,pcl::PointXYZRGB>::VOXEL_GRID_DILATION);
    // mls.setDilationVoxelSize(0.08);
    // mls.process(*upsampled_cloud); 

    // Outlier Removal
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    // pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;
    // // build the filter
    // outrem.setInputCloud(upsampled_cloud);
    // outrem.setRadiusSearch(0.3);
    // outrem.setMinNeighborsInRadius (4);
    // // apply filter
    // outrem.filter (*filtered_cloud);
    // Create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud (cloud_p);
    sor.setMeanK (30);
    sor.setStddevMulThresh (1.0);
    sor.filter (*filtered_cloud);

    std::stringstream ss;
    ss << "cluster_" << i <<".pcd";
    pcl::io::savePCDFileASCII(ss.str(), *filtered_cloud);
    ss << "cluster_" << i;
    viewer->addPointCloud<pcl::PointXYZRGB>(filtered_cloud, rgb, ss.str(), v1);

    // // Create a Convex Hull representation of the projected inliers
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZRGB>);
    // pcl::ConcaveHull<pcl::PointXYZRGB> chull;
    // chull.setInputCloud (filtered_cloud);
    // chull.setComputeAreaVolume(true);
    // chull.setDimension(2); 

    // Create a Convex Hull representation of the projected inliers
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::ConcaveHull<pcl::PointXYZRGB> chull;
    chull.setInputCloud (filtered_cloud);
    chull.setAlpha(0.1);
    chull.setDimension(2); 
    chull.reconstruct (*cloud_hull);

    std::cerr << "Convex hull has: " << cloud_hull->points.size () << " data points." << std::endl;
    // std::cerr << "Convex hull area is: " << chull.getTotalArea () << std::endl;  
    // pcl::PCDWriter writer;
    // writer.write ("convex_hull.pcd", *cloud_hull, false);
    // std::stringstream ss;
    pcl::PointXYZ convex_center(0, 0, 0);
    for (int j = 0; j < cloud_hull->points.size()-1; ++j)
    { 
      // convex_center.x += cloud_hull->points[j].x;
      // convex_center.y += cloud_hull->points[j].y;
      // convex_center.z += cloud_hull->points[j].z;
      ss << i << "_line_" << j << ".pcd";
      viewer->addLine(cloud_hull->points[j], cloud_hull->points[j+1], ss.str(), v1);
    }
    
    pcl::PointXYZRGB min_pt, max_pt;
    pcl::getMinMax3D(*cloud_hull, min_pt, max_pt);
    std::cout << "Min: " << min_pt.x << " " << min_pt.y << " " << min_pt.z << " " << std::endl;
    std::cout << "Max: " << max_pt.x << " " << max_pt.y << " " << max_pt.z << " " << std::endl;
    //float av_z = (min_pt.z + max_pt.z)/2;
    
    pcl::PointXYZRGB x1, x2, x3, x4;
    x1.x = min_pt.x; x1.y = max_pt.y; x1.z = min_pt.z;
    x2.x = min_pt.x; x2.y = min_pt.y; x2.z = max_pt.z;
    x3.x = max_pt.x; x3.y = min_pt.y; x3.z = max_pt.z;
    x4.x = max_pt.x; x4.y = max_pt.y; x4.z = min_pt.z;

    std::cout << "x1: " << x1.x << " " << x1.y << " " << x1.z << " " << std::endl;
    std::cout << "x2: " << x2.x << " " << x2.y << " " << x2.z << " " << std::endl;
    std::cout << "x3: " << x3.x << " " << x3.y << " " << x3.z << " " << std::endl;
    std::cout << "x4: " << x4.x << " " << x4.y << " " << x4.z << " " << std::endl;

    ss << i << "_rectangle_1_front" << ".pcd";
    viewer->addLine(x1, x2, 255, 0, 0, ss.str(), v1);
    ss << i << "_rectangle_2_front" << ".pcd";
    viewer->addLine(x2, x3, 255, 0, 0, ss.str(), v1);
    ss << i << "_rectangle_3_front" << ".pcd";
    viewer->addLine(x3, x4, 255, 0, 0, ss.str(), v1);
    ss << i << "_rectangle_4_front" << ".pcd";
    viewer->addLine(x4, x1, 255, 0, 0, ss.str(), v1); 

    // x1.z = min_pt.z+0.6;
    // x2.z = max_pt.z+0.6;
    // x3.z = max_pt.z+0.6;
    // x4.z = min_pt.z+0.6;
    // ss << i << "_rectangle_1_back" << ".pcd";
    // viewer->addLine(x1, x2, 255, 0, 0, ss.str(), v1);
    // ss << i << "_rectangle_2_back" << ".pcd";
    // viewer->addLine(x2, x3, 255, 0, 0, ss.str(), v1);
    // ss << i << "_rectangle_3_back" << ".pcd";
    // viewer->addLine(x3, x4, 255, 0, 0, ss.str(), v1);
    // ss << i << "_rectangle_4_back" << ".pcd";
    // viewer->addLine(x4, x1, 255, 0, 0, ss.str(), v1); 

    // convex_center.x /= cloud_hull->points.size();
    // convex_center.y /= cloud_hull->points.size();
    // convex_center.z /= cloud_hull->points.size();

    // std::cout << "Center coordinates: " << convex_center.x << " " << convex_center.y << " " << convex_center.z << std::endl;  
    std::cout << std::endl;

    // ss << i << "_center_line" << ".pcd";
    // viewer->addLine(convex_center,  cloud_hull->points[0], ss.str(), v1);

    // ss << "last_line_" << i << ".pcd";
    // viewer->addLine(cloud_hull->points[cloud_hull->points.size()-1], cloud_hull->points[0], ss.str(), v1);
  }
  

  while(!viewer->wasStopped())
  {
    viewer->spinOnce();
    boost::this_thread::sleep (boost::posix_time::microseconds(100000));
  }
  return (0);
}