#include "Skeleton.h"



Box::Box(double heigh, double width, double depth, cv::Point3f center, int topView, int frontView, bool isExist)
{

	    heigh_ = heigh;
	    width_ = width;
	    depth_ = depth;
	   center_ = center;
	frontView_ = frontView;
	  topView_ = topView;
	  isExist_ = isExist;
}


void
Box::drawBox (void* viewer_void)
{
    pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);

	double h, w, d;

	switch (frontView_) {
		case SK_FRONT:
		case SK_BACK:
			if (topView_ == SK_TOP or topView_ == SK_BOTTOM) {
				h = heigh_;
				w = width_;
				d = depth_;
			}
			else {
				w = heigh_;
				h = width_;
				d = depth_;
			}
			break;

		case SK_LEFT:
		case SK_RIGHT:
			if (topView_ == SK_TOP or topView_ == SK_BOTTOM) {
				h = heigh_;
				w = depth_;
				d = width_;
			}
			else {
				w = heigh_;
				h = depth_;
				d = width_;
			}
			break;

		case SK_TOP:
		case SK_BOTTOM:
					if (topView_ == SK_FRONT or topView_ == SK_BACK) {
				h = depth_;
				w = width_;
				d = heigh_;
			}
			else {
				w = depth_;
				h = width_;
				d = heigh_;
			}
			break;
		default:
		std::cout << "INCORRECT NUM IN FRONT_VIEW : " << frontView_ << std::endl;
	}
	double r = 1.0, b = 0.0, g = 0.0;
	if (isExist_ == true) {
		r = 0.0;
		g = 0.1;
	}
	// viewer->addCube (center_.x - h/2.0, center_.x + h/2.0, center_.y - w/2.0, center_.y + w/2.0, center_.z - d/2.0, center_.z + d/2.0, r, g, b, std::to_string(rand()));
	viewer->addLine (center_ + cv::Point3f(-h/2.0, w/2.0, d/2.0), center_ + cv::Point3f(h/2.0, w/2.0, d/2.0), r, g, b, std::to_string(rand()));
	viewer->addLine (center_ + cv::Point3f(-h/2.0, -w/2.0, d/2.0), center_ + cv::Point3f(h/2.0, -w/2.0, d/2.0), r, g, b, std::to_string(rand()));
	viewer->addLine (center_ + cv::Point3f(-h/2.0, w/2.0, -d/2.0), center_ + cv::Point3f(h/2.0, w/2.0, -d/2.0), r, g, b, std::to_string(rand()));
	viewer->addLine (center_ + cv::Point3f(-h/2.0, -w/2.0, -d/2.0), center_ + cv::Point3f(h/2.0, -w/2.0, -d/2.0), r, g, b, std::to_string(rand()));


	viewer->addLine (center_ + cv::Point3f(h/2.0, -w/2.0, d/2.0), center_ + cv::Point3f(h/2.0, -w/2.0, d/2.0), r, g, b, std::to_string(rand()));
	viewer->addLine (center_ + cv::Point3f(h/2.0, w/2.0, d/2.0), center_ + cv::Point3f(h/2.0, w/2.0, d/2.0), r, g, b, std::to_string(rand()));
	viewer->addLine (center_ + cv::Point3f(-h/2.0, -w/2.0, -d/2.0), center_ + cv::Point3f(-h/2.0, -w/2.0, -d/2.0), r, g, b, std::to_string(rand()));
	viewer->addLine (center_ + cv::Point3f(-h/2.0, w/2.0, -d/2.0), center_ + cv::Point3f(-h/2.0, w/2.0, -d/2.0), r, g, b, std::to_string(rand()));

	viewer->addLine (center_ + cv::Point3f(h/2.0, w/2.0, -d/2.0), center_ + cv::Point3f(h/2.0, w/2.0, d/2.0), r, g, b, std::to_string(rand()));
	viewer->addLine (center_ + cv::Point3f(h/2.0, -w/2.0, -d/2.0), center_ + cv::Point3f(h/2.0, -w/2.0, d/2.0), r, g, b, std::to_string(rand()));
	viewer->addLine (center_ + cv::Point3f(-h/2.0, w/2.0, -d/2.0), center_ + cv::Point3f(-h/2.0, w/2.0, d/2.0), r, g, b, std::to_string(rand()));
	viewer->addLine (center_ + cv::Point3f(-h/2.0, -w/2.0, -d/2.0), center_ + cv::Point3f(-h/2.0, -w/2.0, d/2.0), r, g, b, std::to_string(rand()));
}
