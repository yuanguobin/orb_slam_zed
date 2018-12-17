
#include "ros/ros.h"
#include "orb_slam_yuan/camera.h"
#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
cv::Point2f Camera::Img2Cam (const cv::Point2f& P_2f) const

{
	cv::Point2f temp(0,0);
	temp.x=(P_2f.x-u_0)/f_x;
	temp.y=(P_2f.y-v_0)/f_y;
     return temp;
}

cv::Mat Camera::Word2Img(const cv::Matx<float,1,3>& P_3f,const cv::Mat& Rotation_Matrix,const cv::Vec3f& Translation_Vec)
{
	cv::Mat Temp_Vec;
	cv::Vec3f Rotation_Vec;
	cv::Rodrigues(Rotation_Matrix,Rotation_Vec);
	cv::projectPoints(P_3f,Rotation_Vec,Translation_Vec,Camera_Matrix,cv::noArray(),Temp_Vec);
	return Temp_Vec;
}
cv::Mat Camera::Word2Cam(const cv::Mat& Point_Src,const cv::Mat& Point_Dst,const cv::Mat& Rotation_Matrix)
{  
	cv::Mat Temp_Vec;
	cv::perspectiveTransform(Point_Src,Point_Dst,Rotation_Matrix);
	return Temp_Vec;
}
cv::Mat Camera::undistortion_I(cv::Mat& src)
{
	
	cv::Mat dst;
	cv::Mat camera_matrix=(cv::Mat_<double>(3,3)<<f_x,0.0,u_0,0.0,f_y,v_0,0.0,0.0,1.0);
	cv::Mat distortion_coefficients(4,1,CV_32FC1,cv::Scalar(k_1,k_2,k_3,k_4));
	//cv::undistort(src,dst,camera_matrix,distortion_coefficients);
	//cv::imshow("wind1",src);
	//cv::imshow("wind2",dst);
	//cv::waitKey(0);
	return src
;
}