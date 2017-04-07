#include <iostream>
#include <vector>

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
#include <pcl/common/intersections.h>
#include <Eigen/Core>
#include <pcl/sample_consensus/sac_model_parallel_line.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/principal_curvatures.h>


int
main (int argc, char** argv)
{
  // Point clouds
  pcl::PointCloud<pcl::PointXYZ>::Ptr object (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr object_aligned (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr scene (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr object_curvatures (new pcl::PointCloud<pcl::PrincipalCurvatures>);
  pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr scene_curvatures (new pcl::PointCloud<pcl::PrincipalCurvatures>);
  pcl::PointCloud<pcl::Normal>::Ptr object_normals (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<pcl::Normal>::Ptr scene_normals (new pcl::PointCloud<pcl::Normal>);

  typedef pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> ColorHandlerT;

  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/oleg/Desktop/PickToGo/catkin_ws/src/box_detector_pkg/data/input_data/cube_origin.pcd", *object) == -1) //* load the file
  {
    ROS_INFO("Couldn't read file!");
    return (-1);
  }

  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/oleg/Desktop/PickToGo/catkin_ws/src/box_detector_pkg/data/output_data/dominant_cluster.pcd", *scene) == -1) //* load the file
  {
    ROS_INFO("Couldn't read file!");
    return (-1);
  }

  std::cout << "Loaded object " << object->points.size() << " points." << std::endl;
  std::cout << "Loaded scene " << scene->points.size() << " points." << std::endl;

  // Compute the normals
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
  normalEstimation.setInputCloud (object);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  normalEstimation.setSearchMethod (tree);
  normalEstimation.setRadiusSearch (0.03);
  normalEstimation.compute (*object_normals);
  normalEstimation.setInputCloud (scene);
  normalEstimation.compute (*scene_normals);


  std::cout << "Normals estimated " << std::endl;

  // Setup the principal curvatures computation
  pcl::PrincipalCurvaturesEstimation<pcl::PointXYZ, pcl::Normal, pcl::PrincipalCurvatures> principalCurvaturesEstimation;
  // Provide the original point cloud (without normals)
  principalCurvaturesEstimation.setInputCloud (object);
  // Provide the point cloud with normals
  principalCurvaturesEstimation.setInputNormals(object_normals);
  // Use the same KdTree from the normal estimation
  principalCurvaturesEstimation.setSearchMethod (tree);
  principalCurvaturesEstimation.setRadiusSearch(1.0);
  // Actually compute the principal curvatures
  principalCurvaturesEstimation.compute (*object_curvatures);

  // Provide the original point cloud (without normals)
  principalCurvaturesEstimation.setInputCloud (scene);
  // Provide the point cloud with normals
  principalCurvaturesEstimation.setInputNormals(scene_normals);
  // Actually compute the principal curvatures
  principalCurvaturesEstimation.compute (*scene_curvatures);

  const float leaf = 0.005f;

  // Perform alignment
  std::cout << "Starting alignment..." << std::endl;
  pcl::SampleConsensusPrerejective<pcl::PointNormal, pcl::PointNormal, pcl::PrincipalCurvatures> align;
  align.setInputSource (object);
  align.setSourceFeatures (object_curvatures);
  align.setInputTarget (scene);
  align.setTargetFeatures (scene_curvatures);
  align.setMaximumIterations (1000000); // Number of RANSAC iterations
  align.setNumberOfSamples (3); // Number of points to sample for generating/prerejecting a pose
  align.setCorrespondenceRandomness (5);// Number of nearest features to use
  align.setSimilarityThreshold (0.9f); // Polygonal edge length similarity threshold
  align.setMaxCorrespondenceDistance (2.5f * leaf); // Inlier threshold
  align.setInlierFraction (0.3f); // Required inlier fraction for accepting a pose hypothesis
  {
    pcl::ScopeTime t("Alignment");
    align.align (*object_aligned);
  }
  
  if (align.hasConverged ())
  {
    // Print results
    printf ("\n");
    Eigen::Matrix4f transformation = align.getFinalTransformation ();
    pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation(0,0), transformation(0,1), transformation(0,2));
    pcl::console::print_info ("R = | %6.3f %6.3f %6.3f | \n", transformation(1,0), transformation(1,1), transformation(1,2));
    pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation(2,0), transformation(2,1), transformation(2,2));
    pcl::console::print_info ("\n");
    pcl::console::print_info ("t = < %0.3f, %0.3f, %0.3f >\n", transformation(0,3), transformation(1,3), transformation(2,3));
    pcl::console::print_info ("\n");
    pcl::console::print_info ("Inliers: %i/%i\n", align.getInliers().size(), object->size());
    
    // Show alignment
    pcl::visualization::PCLVisualizer visu("Alignment");
    visu.addPointCloud (scene, ColorHandlerT (scene, 0.0, 255.0, 0.0), "scene");
    visu.addPointCloud (object_aligned, ColorHandlerT (object_aligned, 0.0, 0.0, 255.0), "object_aligned");
    visu.spin ();
  }
  else
  {
    pcl::console::print_error ("Alignment failed!\n");
  }  

  return 0;
}