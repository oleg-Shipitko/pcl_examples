
#include "PlaneDetector.h"

void
PlaneDetector::projectTo2D (pcl::ModelCoefficients coefficients, pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr planeCloud)
{
    planeNo_++;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr  rotatedPlaneCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::ModelCoefficients::Ptr             modelCoefficientsPtr (new pcl::ModelCoefficients (coefficients));
    pcl::ProjectInliers<pcl::PointXYZRGB>   proj;

    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setInputCloud (inputCloud);
    proj.setModelCoefficients (modelCoefficientsPtr);
    proj.filter (*planeCloud);

    Eigen::Matrix<float, 1, 3> floorPlaneNormalVec, zAxis, rotationVec;
    zAxis << 0.0,0.0,1.0;

    for (int i = 0; i < 3; i++) {
        floorPlaneNormalVec[i] = coefficients.values[i];    // Already normalized
    }

    // transform (rotate) plane to view, where Z == const.
    Eigen::Affine3f affineTransform = Eigen::Affine3f::Identity();
    affineTransform.translation() << 0,0,0;

    rotationVec = zAxis.cross (floorPlaneNormalVec);
    float theta = -atan2 (rotationVec.norm(), zAxis.dot (floorPlaneNormalVec));
    affineTransform.rotate (Eigen::AngleAxisf (theta, rotationVec));

    pcl::transformPointCloud (*planeCloud, *rotatedPlaneCloud,  affineTransform);
    // planeCloud all points projected on the corresponded ideal planes
    // rotatedPlaneCloud - rotated, to make  Z == const

    // looking for bounding rectangle to crop only informative part of plane
    pcl::PointXYZRGB minPoint;
    pcl::PointXYZRGB maxPoint;
    
    pcl::getMinMax3D (*rotatedPlaneCloud, minPoint, maxPoint);
    // fill 2D Mat with values of corresponding cloud points (ignore Z)
    int w = ceil ((maxPoint.x - minPoint.x)/discrete_);
    int h = ceil ((maxPoint.y - minPoint.y)/discrete_);
    cv::Mat discretedPlane = cv::Mat::zeros (w+1, h+1, CV_64FC3);
    cv::Mat channels[3];
    for (int i = 0; i < 3; i++) {
        channels[i] = cv::Mat::zeros (w+1, h+1, CV_64FC1);
    }

    for (int i = 0; i < rotatedPlaneCloud->size(); i++) {
        channels[0].at<double>(ceil ((rotatedPlaneCloud->points[i].x - minPoint.x)/discrete_),
                               ceil ((rotatedPlaneCloud->points[i].y - minPoint.y)/discrete_)) = rotatedPlaneCloud->points[i].getRGBVector3i()[0];
        channels[1].at<double>(ceil ((rotatedPlaneCloud->points[i].x - minPoint.x)/discrete_), 
                               ceil ((rotatedPlaneCloud->points[i].y - minPoint.y)/discrete_)) = rotatedPlaneCloud->points[i].getRGBVector3i()[1];
        channels[2].at<double>(ceil ((rotatedPlaneCloud->points[i].x - minPoint.x)/discrete_), 
                               ceil ((rotatedPlaneCloud->points[i].y - minPoint.y)/discrete_)) = rotatedPlaneCloud->points[i].getRGBVector3i()[2];
    }
    merge (channels, 3, discretedPlane);

    const std::string filename (std::to_string (planeNo_) + "i.png");
    cv::imwrite (filename, discretedPlane);
}

PlaneDetector::PlaneDetector(pcl::PointCloud<pcl::PointXYZRGB> cloud)
{
    cloud_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new  pcl::PointCloud<pcl::PointXYZRGB>(cloud));
}

// Throughpass filter
void 
PlaneDetector::applyThroughFilter()
{
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (cloud_);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (passFilterXMin_, passFilterXMax_);

    pass.filter (*cloud_);
    pass.setInputCloud (cloud_);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (passFilterYMin_, passFilterYMax_);

    pass.filter (*cloud_);
    pass.setInputCloud (cloud_);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (passFilterZMin_, passFilterZMax_);
    pass.filter (*cloud_);
}

// Outlier filter
// Delete areas where less than <outlierMeanK_> points in radius of <outlierStddevThresh_> meters
void 
PlaneDetector::applyStatisticalOutlierRemoval()
{
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud (cloud_);
    sor.setMeanK (outlierMeanK_);
    sor.setStddevMulThresh (outlierStddevThresh_);
    sor.filter (*cloud_);
}

// Voxel Grid Filter
// merge all points in <leafSize_> x <leafSize_> x <leafSize_> meters area
void 
PlaneDetector::applyVoxelFilter()
{
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud (cloud_);
    sor.setLeafSize (leafSize_, leafSize_, leafSize_);
    sor.filter (*cloud_);
}

// RANSAC plane detector
void 
PlaneDetector::detectPlanes()
{
    // planes_ store all planes
    // coefficients_ store ModelCoefficients for all planes_
    planes_.clear();
    coefficients_.clear();

    pcl::PointIndices::Ptr inliers (new pcl::PointIndices());
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setOptimizeCoefficients (true);

    //Uses for looking planes, perpendicular to origin (NOT USED NOW!!!)
    seg.setAxis (Eigen::Vector3f(0, -0.55, 1));
    seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (planeMaxIterations_);
    seg.setDistanceThreshold (planeDistThresh_);
    seg.setEpsAngle (360.0f * (M_PI/180.0f));

    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    int count = 0;
    int nPoints = (int) cloud_->points.size();

    // While 10% of original cloud points are still here
    while(cloud_->points.size() > 0.1*nPoints) {
        seg.setInputCloud (cloud_);
        pcl::ModelCoefficients coefficients;
        // find the most probable plane
        seg.segment (*inliers, coefficients);

        if (inliers->indices.size() < planeMinPoints_) {
            std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        // Extract the inliers
        extract.setInputCloud (cloud_);
        extract.setIndices (inliers);
        // Добавим инлайеры в новое облако
        extract.setNegative(false);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr planeCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
        extract.filter(*inputCloud);
        projectTo2D (coefficients, inputCloud, planeCloud);
        
        // don't need separate points
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
        sor.setInputCloud (planeCloud);
        sor.setMeanK (outlierMeanK_);
        sor.setStddevMulThresh (outlierStddevThresh_);
        sor.filter (*planeCloud);

        planes_.push_back (*planeCloud);
        coefficients_.push_back (coefficients);
        // Delete finded planeCloud from original point cloud_
        extract.setNegative (true);
        extract.filter (*cloud_);
        count++;
    }
}

pcl::PointCloud<pcl::PointXYZRGB>
PlaneDetector::routine()
{
    this->applyThroughFilter();
    this->applyStatisticalOutlierRemoval();
    this->applyVoxelFilter();
    // this->edgeDetection(this->cloud_);
    this->detectPlanes();
    cv::Mat img_object = cv::imread ("source_front.png", CV_LOAD_IMAGE_COLOR);
    cv::Mat img_scene = cv::imread ("scene.png", CV_LOAD_IMAGE_COLOR);

    TempMatching (img_scene, img_object);
    return pcl::PointCloud<pcl::PointXYZRGB> (*cloud_);
}

void 
PlaneDetector::edgeDetection(pcl::PointCloud<pcl::PointXYZRGB>::Ptr rotatedPlaneCloud)
{
    rotatedPlaneCloud->width = 640; // Image-like organized structure, with 640 rows and 480 columns,
    rotatedPlaneCloud->height = 480;

    double depthDisconThreshold = 0.05; //1cm
    int maxSearchNeighbors = 50;
    pcl::OrganizedEdgeFromRGB<pcl::PointXYZRGB, pcl::Label> oed;
    oed.setInputCloud (rotatedPlaneCloud);
    oed.setDepthDisconThreshold (depthDisconThreshold);
    oed.setMaxSearchNeighbors (maxSearchNeighbors);
    oed.setEdgeType (oed.EDGELABEL_NAN_BOUNDARY | oed.EDGELABEL_OCCLUDING | oed.EDGELABEL_OCCLUDED | oed.EDGELABEL_HIGH_CURVATURE | oed.EDGELABEL_RGB_CANNY );
    pcl::PointCloud<pcl::Label> labels;
    std::vector<pcl::PointIndices> indices;
    oed.compute (labels, indices);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr occluding_edges (new pcl::PointCloud<pcl::PointXYZRGB>),
                                           occluded_edges (new pcl::PointCloud<pcl::PointXYZRGB>),
                                           boundary_edges (new pcl::PointCloud<pcl::PointXYZRGB>),
                                           high_curvature_edges (new pcl::PointCloud<pcl::PointXYZRGB>),
                                           rgb_edges (new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::copyPointCloud (*rotatedPlaneCloud, indices[0].indices, *boundary_edges);
    pcl::copyPointCloud (*rotatedPlaneCloud, indices[1].indices, *occluding_edges);
    pcl::copyPointCloud (*rotatedPlaneCloud, indices[2].indices, *occluded_edges);
    pcl::copyPointCloud (*rotatedPlaneCloud, indices[3].indices, *high_curvature_edges);
    pcl::copyPointCloud (*rotatedPlaneCloud, indices[4].indices, *rgb_edges);
}

std::vector<pcl::PointCloud<pcl::PointXYZRGB>> 
PlaneDetector::getPlanes()
{
    return planes_;
}

std::vector<pcl::ModelCoefficients> 
PlaneDetector::getCoefficients()
{
    return coefficients_;
}