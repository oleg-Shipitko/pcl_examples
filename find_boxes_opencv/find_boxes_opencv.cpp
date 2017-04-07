#include <iostream>
#include <math.h>
#include <string.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/png_io.h>
#include <pcl/io/io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/transforms.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

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

#include <boost/thread/thread.hpp>

int thresh = 50, N = 11;
const char* wndname = "Square Detection Demo";

static double angle( cv::Point pt1, cv::Point pt2, cv::Point pt0 )
{
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;
    return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

static void findSquares( const cv::Mat& image, std::vector<std::vector<cv::Point> >& squares )
{
    squares.clear();

    cv::Mat pyr, timg, gray0(image.size(), CV_8U), gray;

    // down-scale and upscale the image to filter out the noise
    cv::pyrDown(image, pyr, cv::Size(image.cols/2, image.rows/2));
    cv::pyrUp(pyr, timg, image.size());
    std::vector<std::vector<cv::Point> > contours;

    // find squares in every color plane of the image
    for( int c = 0; c < 3; c++ )
    {
        int ch[] = {c, 0};
        cv::mixChannels(&timg, 1, &gray0, 1, ch, 1);

        // try several threshold levels
        for( int l = 0; l < N; l++ )
        {
            // hack: use Canny instead of zero threshold level.
            // Canny helps to catch squares with gradient shading
            if( l == 0 )
            {
                // apply Canny. Take the upper threshold from slider
                // and set the lower to 0 (which forces edges merging)
                cv::Canny(gray0, gray, 0, thresh, 5);
                // dilate canny output to remove potential
                // holes between edge segments
                cv::dilate(gray, gray, cv::Mat(), cv::Point(-1,-1));
            }
            else
            {
                // apply threshold if l!=0:
                //     tgray(x,y) = gray(x,y) < (l+1)*255/N ? 255 : 0
                gray = gray0 >= (l+1)*255/N;
            }

            // find contours and store them all as a list
            cv::findContours(gray, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

            std::vector<cv::Point> approx;

            // test each contour
            for( size_t i = 0; i < contours.size(); i++ )
            {
                // approximate contour with accuracy proportional
                // to the contour perimeter
                cv::approxPolyDP(cv::Mat(contours[i]), approx, cv::arcLength(cv::Mat(contours[i]), true)*0.02, true);

                // square contours should have 4 vertices after approximation
                // relatively large area (to filter out noisy contours)
                // and be convex.
                // Note: absolute value of an area is used because
                // area may be positive or negative - in accordance with the
                // contour orientation
                if( approx.size() == 4 &&
                    fabs(cv::contourArea(cv::Mat(approx))) > 1000 &&
                    cv::isContourConvex(cv::Mat(approx)) )
                {
                    double maxCosine = 0;

                    for( int j = 2; j < 5; j++ )
                    {
                        // find the maximum cosine of the angle between joint edges
                        double cosine = fabs(angle(approx[j%4], approx[j-2], approx[j-1]));
                        maxCosine = MAX(maxCosine, cosine);
                    }

                    // if cosines of all angles are small
                    // (all angles are ~90 degree) then write quandrange
                    // vertices to resultant sequence
                    if( maxCosine < 0.3 )
                        squares.push_back(approx);
                }
            }
        }
    }
}

// the function draws all the squares in the image
static void drawSquares( cv::Mat& image, const std::vector< std::vector<cv::Point> >& squares )
{
    for(size_t i = 0; i < squares.size(); i++ )
    {
        const cv::Point* p = &squares[i][0];
        int n = (int)squares[i].size();
        cv::polylines(image, &p, &n, 1, true, cv::Scalar(0,255,0), 3, cv::LINE_AA);
    }

    imshow(wndname, image);
}

void projectTo2D (pcl::ModelCoefficients coefficients, pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr planeCloud)
{
    double discrete_ = 0.002;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr  rotatedPlaneCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::ModelCoefficients::Ptr             modelCoefficientsPtr (new pcl::ModelCoefficients (coefficients));
    pcl::ProjectInliers<pcl::PointXYZRGB>   proj;

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
    cv::merge (channels, 3, discretedPlane);

    cv::imwrite ("image_from_point_cloud.png", discretedPlane);
    std::vector<std::vector<cv::Point> > squares;
    findSquares(discretedPlane, squares);
}





int main(int argc, char** argv)
{
    std::string box_filename = argv[1];
    std::cout << "Reading " << box_filename << std::endl;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

    if(pcl::io::loadPCDFile<pcl::PointXYZRGB> (box_filename, *cloud) == -1) // load the file
    {
    PCL_ERROR ("Couldn't read file");
    return -1;
    }
    std::cout << "points: " << cloud->points.size () <<std::endl;

    std::cerr << "PointCloud after filtering: " << cloud->width * cloud->height << " data points." << std::endl;

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
    seg.setDistanceThreshold (0.01);

    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;

    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZRGB>);
    // Extract the inliers
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_p);
    std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;
    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZRGB> ("extracted_plane.pcd", *cloud_p, false);

    projectTo2D(*coefficients, cloud, cloud_p);

    return 0;
}