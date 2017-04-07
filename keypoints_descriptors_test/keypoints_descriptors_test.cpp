
// STL
#include <iostream>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/filters/filter.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/impl/sift_keypoint.hpp>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/keypoints/iss_3d.h>
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
#include <pcl/features/fpfh.h>
#include <pcl/console/print.h>
#include <pcl/features/vfh.h>
#include <pcl/common/intersections.h>
#include <Eigen/Core>
#include <pcl/sample_consensus/sac_model_parallel_line.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/features/usc.h>
// Standatrd libraries
#include <iostream>
#include <vector>
#include <string>
#include <stack>
	
double computeCloudResolution(const pcl::PointCloud<pcl::PointNormal>::ConstPtr& cloud)
{
  double resolution = 0.0;
  int numberOfPoints = 0;
  int nres;
  std::vector<int> indices(2);
  std::vector<float> squaredDistances(2);
  pcl::search::KdTree<pcl::PointNormal> tree;
  tree.setInputCloud(cloud);

  for (size_t i = 0; i < cloud->size(); ++i)
  {
    if (! pcl_isfinite((*cloud)[i].x))
      continue;

    // Considering the second neighbor since the first is the point itself.
    nres = tree.nearestKSearch(i, 2, indices, squaredDistances);
    if (nres == 2)
    {
      resolution += sqrt(squaredDistances[1]);
      ++numberOfPoints;
    }
  }
  if (numberOfPoints != 0)
    resolution /= numberOfPoints;

  return resolution;
}

int main(int, char** argv)
{
  //**********************LOAD FILES FROM DISK**********************************************
  std::string box_filename = argv[1];
  std::cout << "Reading " << box_filename << std::endl;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr box (new pcl::PointCloud<pcl::PointXYZRGB>);
  
  if(pcl::io::loadPCDFile<pcl::PointXYZRGB> (box_filename, *box) == -1) // load the file
  {
  PCL_ERROR ("Couldn't read file");
  return -1;
  }
  std::cout << "points: " << box->points.size () <<std::endl;

  std::string scene_filename = argv[2];
  std::cout << "Reading " << scene_filename << std::endl;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene (new pcl::PointCloud<pcl::PointXYZRGB>);
  
  if(pcl::io::loadPCDFile<pcl::PointXYZRGB> (scene_filename, *scene) == -1) // load the file
  {
  PCL_ERROR ("Couldn't read file");
  return -1;
  }
  std::cout << "points: " << scene->points.size () <<std::endl;

  //**********************VOXEL GRID**********************************************
  float leaf = 0.005f;
  pcl::VoxelGrid<pcl::PointXYZRGB> grid;
  grid.setLeafSize (leaf, leaf, leaf);
  grid.setInputCloud (box);
  grid.filter (*box);
  grid.setInputCloud (scene);
  grid.filter (*scene);

  //**********************COMPUTE NORMALS**********************************************
  pcl::PointCloud<pcl::PointNormal>::Ptr box_normals (new pcl::PointCloud<pcl::PointNormal>);
  pcl::PointCloud<pcl::PointNormal>::Ptr scene_normals (new pcl::PointCloud<pcl::PointNormal>);
  pcl::copyPointCloud(*box, *box_normals);
  pcl::copyPointCloud(*scene, *scene_normals);
  // Estimate normals for scene and object
  pcl::console::print_highlight ("Estimating scene and object normals...\n");
  pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::PointNormal> nest;
  nest.setViewPoint(3, -3, 0);
  nest.setRadiusSearch (0.025f); 
  nest.setInputCloud (box);
  nest.compute (*box_normals);
  nest.setInputCloud (scene);
  nest.compute (*scene_normals);

  // Saving the resultant cloud 
  pcl::io::savePCDFileASCII("normals_box.pcd", *box_normals);
  pcl::io::savePCDFileASCII("normals_scene.pcd", *scene_normals);  

  //**********************COMPUTE ISS KEYPOINTS**********************************************
  pcl::PointCloud<pcl::PointNormal>::Ptr box_keypoints (new pcl::PointCloud<pcl::PointNormal>);
  pcl::PointCloud<pcl::PointNormal>::Ptr scene_keypoints (new pcl::PointCloud<pcl::PointNormal>);
  

  pcl::search::KdTree<pcl::PointNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointNormal>);
  pcl::ISSKeypoint3D<pcl::PointNormal, pcl::PointNormal> iss_detector;
  double resolution = computeCloudResolution(box_normals);
  iss_detector.setSearchMethod (tree);
  iss_detector.setSalientRadius (6 * resolution);
  iss_detector.setNonMaxRadius (4 * resolution);
  iss_detector.setThreshold21 (0.975);
  iss_detector.setThreshold32 (0.975);
  iss_detector.setMinNeighbors (5);
  iss_detector.setNumberOfThreads (4);
  iss_detector.setInputCloud (box_normals);
  iss_detector.compute (*box_keypoints);
  resolution = computeCloudResolution(scene_normals);
  iss_detector.setSalientRadius (6 * resolution);
  iss_detector.setNonMaxRadius (4 * resolution);
  iss_detector.setInputCloud (scene_normals);
  iss_detector.compute (*scene_keypoints);

  // for(size_t i = 0; i < box_keypoints->width; i++)
  // {
  //   for (size_t j = 0; j < box_normals->width; j++)
  //   {
  //     if(box_normals->points[j].x == box_keypoints->points[i].x && 
  //        box_normals->points[j].y == box_keypoints->points[i].y && 
  //        box_normals->points[j].z == box_keypoints->points[i].z)
  //     {
  //       box_keypoints->points[i].normal_x = box_normals->points[j].normal_x;
  //       box_keypoints->points[i].normal_y = box_normals->points[j].normal_y;
  //       box_keypoints->points[i].normal_z = box_normals->points[j].normal_z;
  //       box_keypoints->points[i].curvature = box_normals->points[j].curvature;
  //     }
  //   }
  // }

  // for(size_t i = 0; i < scene_keypoints->width; i++)
  // {
  //   for (size_t j = 0; j < scene_normals->width; j++)
  //   {
  //     if(scene_normals->points[j].x == scene_keypoints->points[i].x && 
  //        scene_normals->points[j].y == scene_keypoints->points[i].y && 
  //        scene_normals->points[j].z == scene_keypoints->points[i].z)
  //     {
  //       scene_keypoints->points[i].normal_x = scene_normals->points[j].normal_x;
  //       scene_keypoints->points[i].normal_y = scene_normals->points[j].normal_y;
  //       scene_keypoints->points[i].normal_z = scene_normals->points[j].normal_z;
  //       scene_keypoints->points[i].curvature = scene_normals->points[j].curvature;
  //     }
  //   }
  // }

  // Saving the resultant cloud 
  std::cout << "Resulting iss points of box: " << box_keypoints->points.size() <<std::endl;
  pcl::io::savePCDFileASCII("iss_keypoints_box.pcd", *box_keypoints);
  std::cout << "Resulting iss points of scene: " << scene_keypoints->points.size() <<std::endl;
  pcl::io::savePCDFileASCII("iss_keypoints_scene.pcd", *scene_keypoints);  

  std::vector<int> v;
  pcl::removeNaNFromPointCloud(*scene_keypoints, *scene_keypoints, v);
  pcl::removeNaNFromPointCloud(*box_keypoints, *box_keypoints, v);

//**********************COMPUTE FPFH IN KEYPOINTS**********************************************
  // Estimate features
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr box_features (new pcl::PointCloud<pcl::FPFHSignature33>);
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr scene_features (new pcl::PointCloud<pcl::FPFHSignature33>);
  pcl::console::print_highlight ("Estimating features...\n");
  pcl::FPFHEstimation<pcl::PointNormal, pcl::PointNormal, pcl::FPFHSignature33> fest;
  fest.setRadiusSearch (0.055f); // 0.025 
  fest.setInputCloud (box_normals);
  fest.setInputNormals (box_normals);
  // for (int i = 0; i < box_normals->points.size(); i++) //!!! Check how to acess normals !!!
  // {
  //   if (!pcl::isFinite<pcl::PointNormal>(box_normals->points[i]))
  //   {
  //    std::cout << "normals[%d] are not finite\n";
  //   }
  // }
  fest.compute (*box_features);

  fest.setInputCloud (scene_normals);
  fest.setInputNormals (scene_normals);
  // for (int i = 0; i < scene_normals->points.size(); i++) //!!! Check how to acess normals !!!
  // {
  //   if (!pcl::isFinite<pcl::PointNormal>(scene_normals->points[i]))
  //   {
  //     std::cout << "normals[%d] are not finite\n";
  //   }
  // }
  fest.compute (*scene_features);
  pcl::io::savePCDFileASCII("features_box.pcd", *box_features);
  pcl::io::savePCDFileASCII("features_scene.pcd", *scene_features);

  // USC estimation object.
  // pcl::PointCloud<pcl::UniqueShapeContext1960>::Ptr box_features (new pcl::PointCloud<pcl::UniqueShapeContext1960>);
  // pcl::PointCloud<pcl::UniqueShapeContext1960>::Ptr scene_features (new pcl::PointCloud<pcl::UniqueShapeContext1960>);
  // pcl::UniqueShapeContext<pcl::PointNormal, pcl::UniqueShapeContext1960, pcl::ReferenceFrame> usc;
  // usc.setInputCloud(scene_keypoints);
  // // Search radius, to look for neighbors. It will also be the radius of the support sphere.
  // usc.setRadiusSearch(0.05);
  // // The minimal radius value for the search sphere, to avoid being too sensitive
  // // in bins close to the center of the sphere.
  // usc.setMinimalRadius(0.05 / 10.0);
  // // Radius used to compute the local point density for the neighbors
  // // (the density is the number of points within that radius).
  // usc.setPointDensityRadius(0.05 / 5.0);
  // // Set the radius to compute the Local Reference Frame.
  // usc.setLocalRadius(0.05);
  // usc.compute(*scene_features);
  // usc.setInputCloud(box_keypoints);
  // usc.compute(*box_features);

  //**********************FIND CORRESPONDENCES BETWEEN SCENE AND BOX**********************************************
  // A kd-tree object that uses the FLANN library for fast search of nearest neighbors.
  // pcl::KdTreeFLANN<pcl::FPFHSignature33> matching;
  pcl::KdTreeFLANN<pcl::FPFHSignature33> matching;
  matching.setInputCloud(box_features);
  // A Correspondence object stores the indices of the query and the match,
  // and the distance/weight.
  pcl::Correspondences correspondences;

  // Check every descriptor computed for the scene.
  for (size_t i = 0; i < scene_features->size(); ++i)
  {
    std::vector<int> neighbors(1);
    std::vector<float> squaredDistances(1);
    // Ignore NaNs.
    // if (pcl_isfinite(scene_features->at(i).descriptor[0]))
    // {
      // Find the nearest neighbor (in descriptor space)...
      int neighborCount = matching.nearestKSearch(scene_features->at(i), 1, neighbors, squaredDistances);
      // ...and add a new correspondence if the distance is less than a threshold
      // (SHOT distances are between 0 and 1, other descriptors use different metrics).
        // std::cout << sqrt(squaredDistances[0] / squaredDistances[1]) << std::endl;
      if (sqrt(squaredDistances[0] / squaredDistances[1]) > 0.6 && sqrt(squaredDistances[0] / squaredDistances[1]) != std::numeric_limits<float>::infinity())
      {
        pcl::Correspondence correspondence(neighbors[0], static_cast<int>(i), squaredDistances[0]);
        correspondences.push_back(correspondence);
      }
    // }
  }
  std::cout << "Found " << correspondences.size() << " correspondences." << std::endl;

  //**********************PERFORM BOX TO SCENE ALIGNMENT**********************************************
  pcl::PointCloud<pcl::PointNormal>::Ptr box_aligned (new pcl::PointCloud<pcl::PointNormal>);
  // Perform alignment
  leaf = 0.005f;
  pcl::console::print_highlight ("Starting alignment...\n");
  pcl::SampleConsensusPrerejective<pcl::PointNormal,pcl::PointNormal,pcl::FPFHSignature33> align;
  align.setInputSource (box_normals);
  align.setSourceFeatures (box_features);
  align.setInputTarget (scene_normals);
  align.setTargetFeatures (scene_features);
  align.setMaximumIterations (50000); // Number of RANSAC iterations
  align.setNumberOfSamples (5); // Number of points to sample for generating/prerejecting a pose
  align.setCorrespondenceRandomness (3); // Number of nearest features to use
  align.setSimilarityThreshold (0.6f); // Polygonal edge length similarity threshold
  align.setMaxCorrespondenceDistance (5.5f * leaf); // Inlier threshold
  align.setInlierFraction (0.1f); // Required inlier fraction for accepting a pose hypothesis
  {
    pcl::ScopeTime t("Alignment");
    align.align (*box_aligned);
  }
  if (align.hasConverged ())
  {
    // Print results
    printf ("\n");
    Eigen::Matrix4f transformation = align.getFinalTransformation ();
    pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (0,0), transformation (0,1), transformation (0,2));
    pcl::console::print_info ("R = | %6.3f %6.3f %6.3f | \n", transformation (1,0), transformation (1,1), transformation (1,2));
    pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (2,0), transformation (2,1), transformation (2,2));
    pcl::console::print_info ("\n");
    pcl::console::print_info ("t = < %0.3f, %0.3f, %0.3f >\n", transformation (0,3), transformation (1,3), transformation (2,3));
    pcl::console::print_info ("\n");
    pcl::console::print_info ("Inliers: %i/%i\n", align.getInliers ().size (), box_aligned->size ());
    
    pcl::io::savePCDFileASCII("box_aligned.pcd", *box_aligned);
  }
  else
  {
    pcl::console::print_error ("Alignment failed!\n");
  }
  

  //**********************VISUALIZATION**********************************************
  // Visualization of keypoints along with the original cloud
  pcl::visualization::PCLVisualizer viewer("PCL Viewer");
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> box_keypoints_color_handler (box_keypoints, 0, 255, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> scene_keypoints_color_handler (scene_keypoints, 0, 255, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> box_color_handler (box, 255, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> scene_color_handler (scene, 255, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> box_aligned_color_handler (box_aligned, 0, 0, 255);  
  viewer.setBackgroundColor( 0.0, 0.0, 0.0 );
  viewer.addPointCloud(box, box_color_handler, "box");
  viewer.addPointCloud(box_keypoints, box_keypoints_color_handler, "box_keypoints");
  viewer.addPointCloud(scene, scene_color_handler, "scene");
  viewer.addPointCloud(box_aligned, box_aligned_color_handler, "aligned");
  viewer.addPointCloud(scene_keypoints, scene_keypoints_color_handler, "scene_keypoints");
  viewer.addCorrespondences<pcl::PointXYZRGB>(scene, box, correspondences);
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "box_keypoints");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "scene_keypoints");
  
  while(!viewer.wasStopped())
  {
    viewer.spinOnce ();
  }
  return 0;
}