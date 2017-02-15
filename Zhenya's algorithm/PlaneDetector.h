#ifndef PLANE_DETECTOR_FILE_COMPILED
#define PLANE_DETECTOR_FILE_COMPILED  

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <math.h>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/io/png_io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/transforms.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_perpendicular_plane.h>

#include <pcl/filters/project_inliers.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/features/organized_edge_detection.h>

#include <pcl/range_image/range_image.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "TempMatching.h"

class PlaneDetector
{
public:
    PlaneDetector();
    PlaneDetector(pcl::PointCloud<pcl::PointXYZRGB> cloud);
    // ~PlaneDetector();
    void applyThroughFilter();
    void applyStatisticalOutlierRemoval();
    void applyVoxelFilter();
    void detectPlanes();
    pcl::PointCloud<pcl::PointXYZRGB> routine();    
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>> getPlanes();
    std::vector<pcl::ModelCoefficients> getCoefficients();
    void edgeDetection(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
    void projectTo2D (pcl::ModelCoefficients coefficients, 
                      pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud, 
                      pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud);

private:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>> planes_;
    // coefficients_ store ModelCoefficients for all planes_
    std::vector<pcl::ModelCoefficients> coefficients_;   
    // Red axis on viewew
    double passFilterXMin_ = -0.52;
    double passFilterXMax_ = 0.2;
    // Green axis on viewew
    double passFilterYMin_ = -0.6;
    double passFilterYMax_ = 0.6;
    // Blue axis on viewew
    double passFilterZMin_ = 0.6;
    double passFilterZMax_ = 2.6;
    int outlierMeanK_ = 0;
    double outlierStddevThresh_ = 1;
    double leafSize_ = 0.01;
    int planeMaxIterations_ = 2000;
    double planeDistThresh_ = 0.015;
    int planeMinPoints_ = 100;
    int planeNo_ = 0;
    // <discrete_> meters is exactly 1 px ot resulted 2d image
    double discrete_ = 0.002; //0.2mm
};

#endif