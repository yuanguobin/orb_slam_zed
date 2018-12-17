#include "ros/ros.h"
#include "opencv2/opencv.hpp"
#include "memory"
#include "opencv2/features2d/features2d.hpp"
class Image
{
private:
	cv::Ptr<cv::ORB> Ptr_ORB = cv::ORB::create();
public:
		typedef std::shared_ptr<Image> Ptr;
		std::vector<cv::KeyPoint> Vec_KeyPoints;
		cv::Mat descriptors;
		cv::Mat Img_m; 
		//std::vector<cv::DMatch> DMatch_m;
		cv::Mat rotation_Matrix;
		cv::Mat transfrom_Matrix;

		Image() {};
		Image(cv::Mat Img):Img_m(Img)
		{ 
			Ptr_ORB->detectAndCompute(Img_m, cv::noArray(), Vec_KeyPoints, descriptors);
			rotation_Matrix=cv::Mat::eye(3,3,CV_64F);
			transfrom_Matrix=cv::Mat::zeros(3,1,CV_64F);
		}//find keypoints in an image and compute descriptor 
		void show(void)
		{
			cv::Mat outImage,rgb;
		    
			cv::cvtColor(Img_m, rgb, CV_RGBA2RGB);//将RGBA TO RGB
			cv::drawKeypoints(rgb,Vec_KeyPoints,outImage);//要求image.type=cv_8uc1,cv_8uc3
			cv::imshow("Img-KEYPOINT",outImage);
			cv::waitKey(0);
		}
};
		