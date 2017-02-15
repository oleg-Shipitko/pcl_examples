#include <iostream>
#include <pcl/io/io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <boost/thread/thread.hpp>
#include <pcl/features/normal_3d.h>

typedef pcl::PointCloud<pcl::PointXYZ> Cloud;
typedef Cloud::Ptr CloudPtr;
CloudPtr cube (new Cloud ());


int main (int argc, char** argv)
{
	int v1(0);
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->createViewPort(0.0, 0.0, 1.0, 1.0, v1);
	viewer->setBackgroundColor(0.0, 0.0, 0.0, v1);
	viewer->addCoordinateSystem (0.5);
	viewer->initCameraParameters ();

	//generate cloud (Cube)
	int height = 35;
	int width = 25;
	int length = 40;
	cube->points.resize (height*width*length);
	cube->width = cube->points.size ();
	cube->height = 1;

	int p = 0;
	for (size_t i = 0; i < length; i++)
		for (size_t j = 0; j < width; j++)
			for (size_t k = 0; k < height; k++)
			{	
				if ((i == length-1) || (j == width-1) || (k == height-1) ||
					(i == 0) 	  || (j == 0)     || (k == 0)) 
				{
					cube->points[p++].getVector3fMap() = Eigen::Vector3f(i*0.01, j*0.01, k*0.01);
				}
			}

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> singleColor (cube, 255, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(cube, singleColor, "Cube", v1);
	
	// Write the downsampled version to disk
  	pcl::PCDWriter writer;
    writer.write<pcl::PointXYZ> ("cube.pcd", *cube, false);

 //    // Create the normal estimation class, and pass the input dataset to it
 //  	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
 //  	ne.setInputCloud (cube);

 //  	// Create an empty kdtree representation, and pass it to the normal estimation object.
 //  	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
 //  	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
 //  	ne.setSearchMethod (tree);

 //  	// Output datasets
 //  	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

 //  	// Use all neighbors in a sphere of radius 3cm	
 //  	ne.setRadiusSearch (3.0);

 //  	// Compute the features
 //  	ne.compute (*cloud_normals);

	// std::cerr << "Cube cloud size:" << cube->points.size () << std::endl;
 //  	std::cerr << "Normals cloud size:" << cloud_normals->points.size ();
 //  	// cloud_normals->points.size () should have the same size as the input cloud->points.size ()*
 //    // pcl::visualization::PointCloudColorHandlerCustom<pcl::Normal> singleColor_normals (cloud_normals, 0, 255, 0);
 //    // viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (cube, cloud_normals, 1, 1, "Normals", v1);

	while(!viewer->wasStopped())
	{
    	viewer->spinOnce();
    	boost::this_thread::sleep (boost::posix_time::microseconds(100000));
  	}
  	viewer->close();

  	return (0);
}