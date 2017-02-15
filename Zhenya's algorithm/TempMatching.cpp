#include "TempMatching.h"


// TO_DO rotation-INVARIANT
void
TempMatching(cv::Mat img, cv::Mat templ)
{
	/// Global Variables
	bool use_mask = 0;
	cv::Mat mask; cv::Mat result;
	const char* image_window = "Source Image";
	const char* result_window = "Result window";

	int match_method = 0;
	int max_Trackbar = 5;
	cv::namedWindow (image_window, cv::WINDOW_AUTOSIZE);
	cv::namedWindow (result_window, cv::WINDOW_AUTOSIZE);

	double globMinVal = 1.0e8, globMaxVal = -1.0e8; 
	double trueAngle = 0;
	cv::Point trueLoc;
	/// Source image to display
	for (double angle = 0; angle < 360; angle += 90) {

		cv::Mat rotTempl;
		cv::Point2f center(templ.cols/2.0, templ.rows/2.0);
		cv::Mat rotationMat = getRotationMatrix2D(center, angle, 1.0);
		warpAffine(templ, rotTempl, rotationMat, templ.size());
		/// Create the result matrix
		int result_cols =  rotTempl.cols - templ.cols + 1;
		int result_rows = rotTempl.rows - templ.rows + 1;

		result.create( result_rows, result_cols, CV_32FC1 );

		/// Do the Matching and Normalize
		bool method_accepts_mask = (CV_TM_SQDIFF == match_method || match_method == CV_TM_CCORR_NORMED);
		if (use_mask && method_accepts_mask) {
			cv::matchTemplate (img, rotTempl, result, match_method, mask); 
		}
		else { 
			cv::matchTemplate (img, rotTempl, result, match_method); 
		}

		cv::normalize (result, result, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());

		/// Localizing the best match with minMaxLoc
		double minVal, maxVal;
		cv::Point minLoc, maxLoc;

		cv::minMaxLoc (result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat());


		/// For SQDIFF and SQDIFF_NORMED, the best matches are lower values. For all the other methods, the higher the better
		if (match_method  == cv::TM_SQDIFF || match_method == cv::TM_SQDIFF_NORMED) {
			if (minVal < globMinVal) {
				trueLoc = minLoc;
				trueAngle = angle;
			}
		}
		else {
			if (maxVal > globMaxVal) {
				trueLoc = maxLoc;
				trueAngle = angle;
			}
		}
	}
	/// Show me what you got
	cv::Mat rotTempl;
	cv::Point2f center(templ.cols/2.0, templ.rows/2.0);
	cv::Mat rotationMat = getRotationMatrix2D(center, trueAngle, 1.0);
	warpAffine(templ, rotTempl, rotationMat, templ.size());

	std::cout << "angle =" << trueAngle << std::endl;

	cv::rectangle (img, trueLoc, cv::Point (trueLoc.x + rotTempl.cols, trueLoc.y + rotTempl.rows), cv::Scalar(255, 0, 0), 2, 8, 0);
	cv::rectangle (result, trueLoc, cv::Point (trueLoc.x + rotTempl.cols, trueLoc.y + rotTempl.rows), cv::Scalar(255, 0, 0), 2, 8, 0);

	cv::imshow (image_window, img);
	cv::imshow (result_window, result);
	cv::imwrite ("example.png", img);
	cv::waitKey(0);
	return;
}

