#include "ros/ros.h"
#include "opencv-3.3.1-dev/opencv2/opencv.hpp"
#include "opencv-3.3.1-dev/opencv2/core/core.hpp"

namespace  yuan_quat
{
    /* code */
template <typename T>
void QuaternionFromMatrix(const cv::Mat& R, T q[])
{
 
	q[0] = sqrt(1 + R.at<T>(0, 0) + R.at<T>(1, 1) + R.at<T>(2, 2))/ 2;
 
	q[1] = (R.at<T>(2, 1) - R.at<T>(1, 2) ) / (4*q[0]);
	q[2] = (R.at<T>(0, 2) + R.at<T>(2, 0) ) / (4*q[0]);
	q[3] = (R.at<T>(1, 0) - R.at<T>(0, 1) ) / (4*q[0]);
    
    T tem=sqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);

    q[0] /= tem;
	q[1] /= tem;
	q[2] /= tem;
	q[3] /= tem;


//    q(2) = (T(3,2)-T(2,3))/(4*q(1));
//    q(3) = (T(1,3)-T(3,1))/(4*q(1));
//    q(4) = (T(2,1)-T(1,2))/(4*q(1));
//    q=q/sqrt(q(1)^2+q(2)^2+q(3)^2+q(4)^2);%%归1化

}

}

