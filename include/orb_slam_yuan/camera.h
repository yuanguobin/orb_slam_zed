#include "ros/ros.h"
#include "opencv-3.3.1-dev/opencv2/core/core.hpp"
#include "memory"
//using namespace cv;
class Camera
{
public:
	double f_x,f_y,u_0,v_0,k_1,k_2,k_3,k_4,k_5; 
	typedef std::shared_ptr<Camera> Ptr;

public:
	cv::Mat Camera_Matrix;
	Camera();
	Camera(float fx ,float fy,float u0,float v0,float k1=0.0,float k2=0.0,float k3=0.0,float k4=0.0,float k5=0.0):
	  f_x(fx),f_y(fy),u_0(u0),v_0(v0),k_1(k1),k_2(k2),k_3(k3),k_4(k4),k_5(k5)
	 {
       Camera_Matrix=(cv::Mat_<double>(3,3)<<f_x,0,u_0,0,f_y,v_0,0,0,1);
	 }
	  cv::Point2f Img2Cam (const cv::Point2f& P_2f) const;
	  cv::Mat Word2Img(const cv::Matx<float,1,3>& P_3f,const cv::Mat& Rotation_Matrix,const cv::Vec3f& Translation_Vec);
	  cv::Mat Word2Cam(const cv::Mat&Point_Src,const cv::Mat& Point_Dst,const cv::Mat& Rotation_Matrix);
	  cv::Mat undistortion_I(cv::Mat& src);
};
