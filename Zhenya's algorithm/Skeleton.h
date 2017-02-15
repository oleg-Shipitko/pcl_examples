#ifndef SKELETON_FILE_COMPILED
#define SKELETON_FILE_COMPILED  

#define SK_FRONT 0
#define SK_BACK 1
#define SK_LEFT 2
#define SK_RIGHT 3
#define SK_TOP 4
#define SK_BOTTOM 5

#include <pcl/visualization/pcl_visualizer.h>

#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>

class Box
{
public:
	Box(double heigh, double width, double depth, cv::Point3f center, int topView, int frontView, bool isExist);
	void drawBox (void* viewer_void);
private : 
	cv::Mat view[6];
	//front, back, left, right, top, bottom
	double heigh_, width_, depth_;
	cv::Point3f center_;
	int topView_;
	int frontView_;
	bool isExist_;
};

// class Skeleton
// {
// 	std::vector<Box> boxes;
// }

#endif