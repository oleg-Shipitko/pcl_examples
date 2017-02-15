#include "PlaneDetector.h"
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include "Skeleton.h"

int user_data;
	
void 
viewerOneOff (pcl::visualization::PCLVisualizer& viewer)
{
	viewer.setBackgroundColor (1.0, 0.5, 1.0);
	pcl::PointXYZ o;
	o.x = 1.0;
	o.y = 0;
	o.z = 0;
	viewer.setCameraPosition(0, 0, -0.6, 0, 0, 1, 0, -1, -1);		

	std::cout << "i only run once" << std::endl;
}
	
void 
viewerPsycho (pcl::visualization::PCLVisualizer& viewer)
{
	static unsigned count = 0;
	std::stringstream ss;
	ss << "Once per viewer loop: " << count++;
	viewer.removeShape ("text", 0);
	viewer.addText (ss.str(), 200, 300, "text", 0);

	//FIXME: possible race condition here:
	user_data++;
}


int 
main ()
{
	pcl::PointCloud<pcl::PointXYZRGB> cloud;
	pcl::io::loadPCDFile ("../data/test_pcd_2.pcd", cloud);

	// pcl::PointCloud<pcl::PointXYZRGB>::Ptr show_cloud (new pcl::PointCloud<pcl::PointXYZRGB>(cloud));
	PlaneDetector detector = PlaneDetector (cloud);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZRGB>(detector.routine()));

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(1.0, 0.5, 1.0);
	//viewer->addPointCloud<pcl::PointXYZRGB>(show_cloud, "std::to_string(i)"); 

	std::vector<pcl::PointCloud<pcl::PointXYZRGB>> tempPlanes = detector.getPlanes();
	
	for(int i = 0; i < tempPlanes.size(); i++)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptr(new pcl::PointCloud<pcl::PointXYZRGB>(tempPlanes.at(i)));
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> singleColor (ptr, 255 - (100*i)%256, (100*i)%256, 20 + i);
		// viewer->addPlane(detector.planeCoefs.at(i), std::to_string(-i-1));
		//viewer->addPointCloud<pcl::PointXYZRGB>(ptr, singleColor, std::to_string(i));
	}
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, "dsdsd");
    Box box(0.2, 0.3, 0.4, cv::Point3f(0.0, 0.0, 0.5), SK_TOP, SK_LEFT, true);
    box.drawBox((void*)viewer.get ());
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	viewer->setCameraPosition (-0.3, 0, -0.6, 0.0, 0, 1, 0, -1, -1);	
	while(!viewer->wasStopped())
	{
		viewer->spinOnce();
		boost::this_thread::sleep (boost::posix_time::microseconds(100000));
	}
	viewer->close();
	return(0);
}


