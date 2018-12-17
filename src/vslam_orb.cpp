// VSLAM_Odometry.cpp 
#include "ros/ros.h"
#include "opencv-3.3.1-dev/opencv2/opencv.hpp"
#include "opencv-3.3.1-dev/opencv2/core/core.hpp"
#include "opencv-3.3.1-dev/opencv2/imgproc/imgproc.hpp"
#include "opencv-3.3.1-dev/opencv2/features2d/features2d.hpp"
#include "opencv-3.3.1-dev/opencv2/calib3d/calib3d.hpp"
#include "opencv-3.3.1-dev/opencv2/core/types.hpp"
#include "orb_slam_yuan/camera.h"
#include "orb_slam_yuan/Image11.h"
#include "pcl-1.7/pcl/visualization/cloud_viewer.h"
#include "pcl-1.7/pcl/visualization/pcl_visualizer.h"
#include "pcl-1.7/pcl/point_cloud.h"
#include "pcl-1.7/pcl/point_types.h"
#include "sensor_msgs/Image.h"
#include "boost/bind.hpp"
#include "cv_bridge/cv_bridge.h"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/TransformStamped.h"
#include "sensor_msgs/Imu.h"
#include "orb_slam_yuan/matrix2quat.h"
#include "suitesparse/cholmod.h"
#include "suitesparse/cholmod_core.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#include "g2o/types/sba/types_six_dof_expmap.h"
#include "g2o/types/sba/types_sba.h"
#include "g2o/core/base_unary_edge.h"
#include "opencv-3.3.1-dev/opencv2/core/eigen.hpp" //eigen matrix to opencv mat

using namespace std;

//全局变量

namespace IMU
{
	double phi=23.0/180*3.14159;
	double wie[3]={0.000067,0.0,-0.000028};//{wie*cos(phi),0,wie*sin(phi)}//北东地
	double freq=0.02;//单位s
	cv::Mat T=cv::Mat::eye(3,3,CV_64F);//捷联矩阵，初始为单位矩阵
	double yaw,pitch,roll;//弧度
	bool flag_AccGetAttit=false;
	double g=9.8;
	double v_x=0.0,v_y=0.0,s_x=0.0,s_y=0.0;
}

//图优化函数声明
g2o::SE3Quat optimize_pose_frompnp(cv::Mat rotation_mat,cv::Mat transform_mat,std::vector<cv::Point3f> Vec_Point3f_Last,std::vector<cv::Point2f> Vec_Point2f_Cur,Camera Cam);
//误差模型 模板参数：误差值维度，类型，连接顶点类型
void optimize_pose(g2o::SparseOptimizer& optimizer);
void add_Vertex_and_edge(g2o::SparseOptimizer& optimizer,cv::Mat& rotation_mat_cur2last,cv::Mat& transform_mat_cur2last,std::vector<cv::Point3f>& Vec_Point3f_Cur,
                          std::vector<cv::Point2f>& Vec_Point2f_Last,int count_vertex_id,int count_edge_id);

class EdgepProjectXYZ2UV:public g2o::BaseUnaryEdge<2,Eigen::Vector2d,g2o::VertexSE3Expmap>
{
  public:
     
     EdgepProjectXYZ2UV(Camera Cam,Eigen::Vector3d point_3d):_Cam(Cam),_point_3d(point_3d),BaseUnaryEdge(){}
    //计算误差式
     void computeError()
      {
          const g2o::VertexSE3Expmap* v=static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);//一元边
          const g2o::SE3Quat pose =v->estimate();
		  Eigen::Vector3d temp_3 =pose.map(_point_3d);
		  Eigen::Vector2d temp_2 ;
		  temp_2[0]=temp_3[0]*_Cam.f_x/temp_3[2]+_Cam.u_0;
		  temp_2[1]=temp_3[1]*_Cam.f_y/temp_3[2]+_Cam.v_0; //投影
          _error=_measurement-temp_2;
		  //typedef std::vector<Vertex*>                      VertexContainer;
          //typedef Eigen::Matrix<double, D(维数), 1, Eigen::ColMajor> ErrorVector;
		  // typedef E Measurement;
      } 
     void linearizeOplus()
	 {
          const g2o::VertexSE3Expmap* v=static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);//一元边
          const g2o::SE3Quat pose =v->estimate();
		  Eigen::Vector3d temp_3 =pose.map(_point_3d);
		  //投影（u v）对se3李代数的 2x6的雅可比式
		  double x=temp_3[0];double y=temp_3[0];double z=temp_3[0];double z_2=z*z;
          _jacobianOplusXi(0,0)=x*y/z_2*_Cam.f_x;
		  _jacobianOplusXi(0,1)=-(1+(x*x/z_2))*_Cam.f_x;
		  _jacobianOplusXi(0,2)=y/z*_Cam.f_x;
		  _jacobianOplusXi(0,3)=-1.0/z*_Cam.f_x;
		  _jacobianOplusXi(0,4)=0;
		  _jacobianOplusXi(0,5)=x/z_2*_Cam.f_x;
	
		  _jacobianOplusXi(1,0)=(1+y*y/z_2)*_Cam.f_y;
		  _jacobianOplusXi(1,1)=-(x*y/z_2)*_Cam.f_y;
		  _jacobianOplusXi(1,2)=-x/z*_Cam.f_y;
		  _jacobianOplusXi(1,3)=0;
		  _jacobianOplusXi(1,4)=-1.0/z*_Cam.f_y;
          _jacobianOplusXi(1,5)=y/z_2*_Cam.f_y;
	 }
     virtual bool read(std::istream& in ){}
     virtual bool write(std::ostream& out)const{}
  public:
     Eigen::Vector3d  _point_3d;
	 Camera _Cam;
};
//回调函数声明
void orb_slam_deal_callback(const sensor_msgs::ImageConstPtr& msg_ptr,const sensor_msgs::ImageConstPtr& msg_depth_ptr,const Camera&Cam);
void imu_callback(const sensor_msgs::ImuConstPtr& imu_ptr);

//主函数
int main(int argc, char* argv[])
{
    ros::init(argc, argv, "orb_slam_yuan");
	ros::NodeHandle nh;

    //参数
	nh.param("flag_AccGetAttit_bool",IMU::flag_AccGetAttit,false);

	//相机的外参数及畸变系数（属性）
	Camera Cam(668.9363,668.9363,625.3835,357.4095,0.0,0.0,0.0,0.0);
    
    //订阅主题设置及回调函数
    ros::Subscriber sub = nh.subscribe("/zed/imu/data", 10,imu_callback); //订阅imu信息

	message_filters::Subscriber<sensor_msgs::Image> sub_Rgb_image(nh,"/zed/rgb/image_rect_color",10);
	message_filters::Subscriber<sensor_msgs::Image> sub_Depth_image(nh,"/zed/depth/depth_registered",10);
	//message_filters::TimeSynchronizer<sensor_msgs::Image,sensor_msgs::Image> sync(sub_Rgb_image,sub_Depth_image,10);//消息同步
    //sync.registerCallback(boost::bind(orb_slam_deal_callback,_1,_2,Cam));
    
   
    ros::spin();
	return 0;
}

//orb_slam算法处理（双目）
void orb_slam_deal_callback(const sensor_msgs::ImageConstPtr& msg_ptr,const sensor_msgs::ImageConstPtr& msg_depth_ptr,const Camera&Cam)
{
	
    //存储所有接收到的图像
    //问题：：保存图片太多，内存容易溢满，报错！
	//static std::vector<Image> Vec_Image_Comput_All;
    //static std::vector<cv::Mat> Vec_Mat_Comput_All_Depth;

	//上一张图像，用于当前图像特征点匹配
	static Image Img_Last;
    static cv::Mat Img_Last_Depth;

	
	static int image_count = 0;
	image_count++;

    //接收sensor::image消息to cvimage(获取图像)rgb
    cv_bridge::CvImagePtr cvimageptr= cv_bridge::toCvCopy(msg_ptr);
    cv::Mat image= cvimageptr->image;
    Image Img_Cur(image);	//图像特征点的提取在类构造函数里实现 CV_8UC3
	if (Img_Cur.Img_m.data == NULL)
	 {
	  exit(0);
	 }
	else
	 {
		//显示rgb图像
      //Img_Cur.show();
	 }

	 //接收sensor::image消息to cvimage(获取图像)depth
    cv_bridge::CvImagePtr cvimageptr_depth= cv_bridge::toCvCopy(msg_depth_ptr);
    cv::Mat Img_Cur_Depth= cvimageptr_depth->image;  //cv_32F
    //cv::imshow("img_depth",Img_Cur_Depth);//显示深度图
	//cv::waitKey(0);
	 
	 //显示单个图像的点云
#define  TEST_IMAGE_CLOUD  
#ifndef  TEST_IMAGE_CLOUD
	 #define TEST_IMAGE_CLOUD 
	 
	 pcl::PointCloud<pcl::PointXYZRGB>::Ptr Cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

	 //遍历图像
    for(size_t y=0; y<Img_Cur.Img_m.rows;y++)
	{

      cv::Vec4b *rgb=Img_Cur.Img_m.ptr<cv::Vec4b>(y);//the ptr to row
	 // cv::Vec4f v= Img_Cur.Img_m.at<cv::Vec4f>(0,0);
      
	  for (size_t x = 0; x <Img_Cur.Img_m.cols ; x++)
	  {
		  /* code for loop body */

		  //读入对应的深度值
		  float depth=Img_Cur_Depth.at<float>(y,x);
          if (std::isnan(depth) || std::isinf(depth))  continue;        //distance more than 20m continue 
		  
		  //归一化坐标系上
		  cv::Point2f Tem_Point2f(x,y);
		  Tem_Point2f = Cam.Img2Cam(Tem_Point2f);
          
		  //保存到点云数据上
		  pcl::PointXYZRGB Point;
		  Point.x=Tem_Point2f.x *depth;
		  Point.y=Tem_Point2f.y *depth;
		  Point.z=depth;

		  Point.r=rgb[x][0];
		  Point.g=rgb[x][1];
		  Point.b=rgb[x][2];
		  Cloud_ptr->push_back(Point);
	  }
	}

   //view cloud
    pcl::visualization::CloudViewer viewer("Image_Cloud_View");
    viewer.showCloud(Cloud_ptr);
    while(!viewer.wasStopped()){}
#endif

    //如果为第一帧图像则返回
	if(image_count == 1) 
	{
		Img_Last=Img_Cur;
	    Img_Last_Depth=Img_Cur_Depth;
	    //Vec_Image_Comput_All.push_back(Img_Last);
		//Vec_Mat_Comput_All_Depth.push_back(Img_Last_Depth);
	    return ;
	}

    //特征点匹配-暴力匹配
	ROS_INFO("Enter the BFMatcher.....");
	cv::BFMatcher BFMatcher_t(cv::NORM_HAMMING);
	std::vector<cv::DMatch> Vec_Match_Full;
	BFMatcher_t.match(Img_Cur.descriptors, Img_Last.descriptors, Vec_Match_Full);//存储两帧图像间的匹配点对

  //匹配点对优化
  
	std::vector<cv::DMatch>  Vec_Match_Optim;
	std::vector<cv::Point3f> Vec_Point3f_Last,Vec_Point3f_Cur;
	std::vector<cv::Point2f> Vec_Point2f_Cur,Vec_Point2f_Last;
	for (size_t i = 0; i < Vec_Match_Full.size(); i++)
	  {
		
		 if (Vec_Match_Full[i].distance<30.0)
			{
				Vec_Match_Optim.push_back(Vec_Match_Full[i]);	

				//get the 3d point of the current image 
				 cv::Point2f Tem_Point2f=Img_Cur.Vec_KeyPoints[Vec_Match_Full[i].queryIdx].pt;
				 float depth=Img_Cur_Depth.at<float>(Tem_Point2f);
				 if (std::isinf(depth) || std::isnan(depth) || depth>20 ||depth<0.1)    continue;
				 Tem_Point2f=Cam.Img2Cam(Tem_Point2f);
				 cv::Point3f Tem_Point3f_1(Tem_Point2f.x*depth,Tem_Point2f.y*depth,depth);
				 Vec_Point3f_Cur.push_back(Tem_Point3f_1);

                //get the 2d point of the last image
			    cv::Point2f Tem_Point2f_1=Img_Last.Vec_KeyPoints[Vec_Match_Full[i].trainIdx].pt;
                Vec_Point2f_Last.push_back(Tem_Point2f_1);
			}
	  }
	  //std::cout<<Vec_Match_Optim.size()<<std::endl; 
  
	//显示优化后匹配点对
	cv::Mat Img_Match_Optim,Rgb1,Rgb2;
	cv::cvtColor(Img_Last.Img_m, Rgb1, CV_RGBA2RGB);//RGBA TO RGB
	cv::cvtColor(Img_Cur.Img_m, Rgb2, CV_RGBA2RGB);//RGBA TO RGB
	cv::drawMatches(Rgb1,Img_Last.Vec_KeyPoints,Rgb2,Img_Cur.Vec_KeyPoints,Vec_Match_Optim,Img_Match_Optim);
    //cv::namedWindow("Matcher", CV_WINDOW_NORMAL);
    //cv::imshow("Matcher",Img_Match_Optim);
    //cv::waitKey(5000);

    /////***********************************************************************************************************////
	////*******************************Enter get the attitude of camera********************************************////
    
	//PnP法求前后一帧相机姿态
	cv::Mat rotation_vec_cur2last, rotation_mat_cur2last,transform_mat_cur2last;

	cv::solvePnP(Vec_Point3f_Cur,Vec_Point2f_Last,Cam.Camera_Matrix,cv::noArray(),rotation_vec_cur2last,transform_mat_cur2last);//cv_64F
    cv::Rodrigues(rotation_vec_cur2last, rotation_mat_cur2last); //Converts a rotation matrix to a rotation vector 
    std::cout<<transform_mat_cur2last<<std::endl;
    
	////*******************************图优化********************************************////
	//增加两帧间的非线性优化（练习G2o的使用）
	/*g2o::SE3Quat se3 = optimize_pose_frompnp(rotation_mat,transform_mat,Vec_Point3f_Last,Vec_Point2f_Cur,Cam);
    Eigen::Quaterniond q = se3.rotation();
	Eigen::Vector3d    v = se3.translation();
    Eigen::Matrix3d    r = q.matrix();
	cv::eigen2cv(v,transform_mat);
	cv::eigen2cv(r,rotation_mat);
    std::cout<<transform_mat<<std::endl;*/
	g2o::SparseOptimizer optimizer;
    optimize_pose(optimizer);
	static int count_vertex_id=0;
	static int count_edge_id =0;
	
	//向图中添加顶点和边(位姿)
    add_Vertex_and_edge(optimizer,rotation_mat_cur2last,transform_mat_cur2last,Vec_Point3f_Cur,Vec_Point2f_Last,count_vertex_id,count_edge_id);
    /////*************************************************************************//////
	
	//get the transform from current image to firs image
	cv::Mat rotation_mat_cur2first,transfrom_mat_cur2first;
    rotation_mat_cur2first = rotation_mat_cur2last*(Img_Last.rotation_Matrix);
	transfrom_mat_cur2first = rotation_mat_cur2last*(Img_Last.transfrom_Matrix)+transform_mat_cur2last;
    Img_Cur.rotation_Matrix=rotation_mat_cur2first;  //save
	Img_Cur.transfrom_Matrix=transfrom_mat_cur2first;
    //std::cout<<rotation_mat_cur2first<<std::endl;
	//std::cout<<Img_Last.rotation_Matrix<<std::endl;


	//get the current keypoint in the last camera coordiate
	 //遍历图像
	 std::vector<cv::Point3f> Vec_Point3f_cur_camera_coord,Vec_Point3f_cur_last_coord;
	 std::vector<cv::Vec3b> Vec_intens_cur;
    for(size_t y=0; y<Img_Cur.Img_m.rows;y++)
	{

      cv::Vec4b *rgb=Img_Cur.Img_m.ptr<cv::Vec4b>(y);//the ptr to row
	 
	  for (size_t x = 0; x <Img_Cur.Img_m.cols ; x++)
	  {
		  //读入对应的深度值
		  float depth=Img_Cur_Depth.at<float>(y,x);
          if (std::isnan(depth) || std::isinf(depth) ||depth>20 ||depth<0.1)  continue;        //distance more than 20m continue 
		  
		  //归一化坐标系上
		  cv::Point2f Tem_Point2f(x,y);
		  Tem_Point2f = Cam.Img2Cam(Tem_Point2f);
          
		  //保存数据
		  cv::Point3f Point;
		  Point.x=Tem_Point2f.x *depth;
		  Point.y=Tem_Point2f.y *depth;
		  Point.z=depth;
          
		  cv::Vec3b rgb_point(rgb[x][0],rgb[x][1],rgb[x][2]); //3-channel rgb

      
		  Vec_Point3f_cur_camera_coord.push_back(Point);
		  Vec_intens_cur.push_back(rgb_point);
	  }
	}

	
   //get 4x4 transform matrix
  
	cv::Mat m = (cv::Mat_<double>(4, 4) <<  rotation_mat_cur2first.at<double>(0, 0), rotation_mat_cur2first.at<double>(0, 1), rotation_mat_cur2first.at<double>(0, 2), transfrom_mat_cur2first.at<double>(0, 0),
				                            rotation_mat_cur2first.at<double>(1, 0), rotation_mat_cur2first.at<double>(1, 1), rotation_mat_cur2first.at<double>(1, 2), transfrom_mat_cur2first.at<double>(1, 0),
				                            rotation_mat_cur2first.at<double>(2, 0), rotation_mat_cur2first.at<double>(2, 1), rotation_mat_cur2first.at<double>(2, 2), transfrom_mat_cur2first.at<double>(2, 0),
				                            0, 0, 0, 1); 
	//Img_Cur.Pose_Matrix = m;//save the pose_matrix
	cv::perspectiveTransform(Vec_Point3f_cur_camera_coord, Vec_Point3f_cur_last_coord, m);//坐标变换
   
   // ROS_INFO("%f",Vec_Point3f_cur_camera_coord[0].x);
  

   //保存点云数据
   	
     static pcl::PointCloud<pcl::PointXYZRGB> Cloud_Slam; 
	 for (size_t i = 0; i < Vec_Point3f_cur_last_coord.size(); i++)
		{
			pcl::PointXYZRGB P;
			P.x = Vec_Point3f_cur_last_coord[i].x;                    //出现问题，Vec_Point3f_cur_last_coord[i].x=nan,调试发现变换矩阵出错
			P.y = Vec_Point3f_cur_last_coord[i].y;
			P.z = Vec_Point3f_cur_last_coord[i].z;
			P.r = Vec_intens_cur[i][0];
			P.g = Vec_intens_cur[i][1];
			P.b = Vec_intens_cur[i][2];
			Cloud_Slam.push_back(P);    
		}
   

   //显示地图
   {
    //pcl::visualization::CloudViewer viewer_slam("Cloud_Viewer");
    //viewer_slam.showCloud(Cloud_Slam.makeShared());
    //while(!viewer_slam.wasStopped()){}
    static pcl::visualization::PCLVisualizer viewer_slam("Cloud_Viewer");
    viewer_slam.addPointCloud(Cloud_Slam.makeShared(),"Cloud");
	if(!viewer_slam.wasStopped()){viewer_slam.spinOnce(1000);}
    viewer_slam.removePointCloud("Cloud");
   }
   //广播里程计信息
   {
      static tf2_ros::TransformBroadcaster tf2_broad;
      
      //声明一个geometry_msgs::transformstamped
      //geometry_msgs::TransformStamped transform_stampe;
      geometry_msgs::TransformStamped transform_cur2first;
      transform_cur2first.header.stamp=ros::Time::now();
      transform_cur2first.header.frame_id="first_camera_frame";
      transform_cur2first.child_frame_id="cur_camera_frame";
   
      transform_cur2first.transform.translation.x=transfrom_mat_cur2first.at<double>(0,0);  //位移信息
	  transform_cur2first.transform.translation.y=transfrom_mat_cur2first.at<double>(1,0);
      transform_cur2first.transform.translation.z=transfrom_mat_cur2first.at<double>(2,0);

      //quaternion
      double q[4];
	  yuan_quat::QuaternionFromMatrix<double>(rotation_mat_cur2first,q); //Mat to quaternion
	

      transform_cur2first.transform.rotation.w=q[0];
      transform_cur2first.transform.rotation.x=q[1];
	  transform_cur2first.transform.rotation.y=q[2];
	  transform_cur2first.transform.rotation.z=q[3];
     
	  //纠错，出现误匹配
      // std::cout<<q[0]<<" "<<q[1]<<" "<<q[2]<<" "<<q[3]<<std::endl;
	  //std::cout<<transfrom_mat_cur2first<<std::endl;
	 //if (q[0]<0.8)
     //{
		/* code for True */
	 //  cv::namedWindow("Matcher", CV_WINDOW_NORMAL);
     //   cv::imshow("Matcher",Img_Match_Optim);
     //    cv::waitKey(0);
     //	}
	
      tf2_broad.sendTransform(transform_cur2first); //broadcast tf
   }

   //图像保存
   //Vec_Image_Comput_All.push_back(Img_Cur);
   //Vec_Mat_Comput_All_Depth.push_back(Img_Cur_Depth);
   Img_Last = Img_Cur;
   Img_Last_Depth = Img_Cur_Depth;
}

//惯导解算
void imu_callback(const sensor_msgs::ImuConstPtr& imu_ptr)
{  
   using namespace IMU;

    ROS_INFO("enter the imucallback...");
    /****更新捷联矩阵**********/
	//保存旋转矢量(默认旋转方向为ZYX（yaw pitch roll）(北东地))
	cv::Mat rotat_vec=(cv::Mat_<double>(3,1)<<imu_ptr->angular_velocity.x*freq,imu_ptr->angular_velocity.y*freq,imu_ptr->angular_velocity.z*freq);

	cv::Mat rotat_mat;
	cv::Rodrigues(rotat_vec, rotat_mat);
    T=rotat_mat*T;
	
	cv::Rodrigues(T,rotat_vec);
    roll=rotat_vec.at<double>(0,0);
	pitch=rotat_vec.at<double>(1,0);
	yaw=rotat_vec.at<double>(2,0);

	//use the linear_acceleration get the Horizontal attitude
	if (true==flag_AccGetAttit)
	{
		/* code for True */
	   roll=std::acos(imu_ptr->linear_acceleration.y/g);
	   pitch=std::acos(imu_ptr->linear_acceleration.x/g);
	}
	
    /*******更新速度信息***********///(北东地))
    cv::Mat accel=(cv::Mat_<double>(3,1)<<imu_ptr->linear_acceleration.x,imu_ptr->linear_acceleration.y,imu_ptr->linear_acceleration.z);
	cv::Mat fp=T*accel;
	double fv_x = fp.at<double>(0,0)+2*wie[2]*v_y;
    double fv_y = fp.at<double>(1,0)-2*wie[2]*v_x;  //平面上速度
   
    v_x = v_x + fv_x*freq;
    v_y = v_y + fv_y*freq;


  // std::cout<<yaw*180.0/3.14159<<"  "<<pitch*180.0/3.14159<<"  "<<roll*180.0/3.14159<<"  "<<v_x<<"  "<<v_y<<"  "<<std::endl;
	//更新水平位移
    s_x = s_x + v_x*freq;
    s_y = s_y + v_y*freq;
   
    /*******发布坐标变换***********/
   static tf2_ros::TransformBroadcaster   tf2_broadcast;
   geometry_msgs::TransformStamped tf2_msg;
   tf2_msg.header.stamp=imu_ptr->header.stamp;    //时间戳
   tf2_msg.header.frame_id="first_pose_IMU";

   tf2_msg.child_frame_id=imu_ptr->header.frame_id;
   
   tf2::Quaternion q;
   q.setRPY (roll,pitch,yaw);
 
   tf2_msg.transform.rotation.x=q.x();
   tf2_msg.transform.rotation.y=q.y();
   tf2_msg.transform.rotation.z=q.z();
   tf2_msg.transform.rotation.w=q.w();
   tf2_msg.transform.translation.x=s_x;
   tf2_msg.transform.translation.y=s_y;

   tf2_broadcast.sendTransform(tf2_msg);

   
}

//增加两帧间的非线性优化（练习G2o的使用）
g2o::SE3Quat optimize_pose_frompnp(cv::Mat rotation_mat,cv::Mat transform_mat,std::vector<cv::Point3f> Vec_Point3f_Last,std::vector<cv::Point2f> Vec_Point2f_Cur,Camera Cam)
{
       //构建图优化，先设定g2o
       //设定矩阵块：每个误差项优化变量维度为6,误差量维度为2（R，T对应的李代数,误差量维度（u,v））
       typedef g2o::BlockSolver< g2o::BlockSolverTraits<6,2> >   Block;

       //设定线性方程求解器
       Block::LinearSolverType* linearSolver=new g2o::LinearSolverDense<Block::PoseMatrixType>();
       Block* solver_ptr = new Block(linearSolver);//矩阵块求解器

       //设置非线性优化方法-列文伯格
       g2o::OptimizationAlgorithmLevenberg* solver=new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    
       //优化函数
       g2o::SparseOptimizer optimizer;
       optimizer.setAlgorithm(solver);
	  
	   //向图中添加顶点
       g2o::VertexSE3Expmap* v=new g2o::VertexSE3Expmap();
	   Eigen::Matrix3d m_rot;                  //cv::mat 2 eigen::matrix
	   cv::cv2eigen(rotation_mat,m_rot);
	   Eigen::Vector3d m_tras;
	   cv::cv2eigen(transform_mat,m_tras);
       g2o::SE3Quat se3(m_rot,m_tras);

       v->setEstimate(se3);//设定初始值
       v->setId(0);
       optimizer.addVertex(v);

	  //向图中添加边
	  for (size_t i = 0; i < Vec_Point3f_Last.size(); i++)
	  {
		  /* code for loop body */
		  Eigen::Vector3d Point_3d;
		  Point_3d[0]=Vec_Point3f_Last[i].x;
		  Point_3d[1]=Vec_Point3f_Last[i].y;
		  Point_3d[2]=Vec_Point3f_Last[i].z;

		  EdgepProjectXYZ2UV*edge=new EdgepProjectXYZ2UV(Cam,Point_3d);
		  edge->setId(i);
		  edge->setVertex(0,v);
         // set the ith vertex on the hyper-edge to the pointer supplied
		  edge->setMeasurement(Eigen::Vector2d(Vec_Point2f_Cur[i].x,Vec_Point2f_Cur[i].y));
		  edge->setInformation(Eigen::Matrix2d::Identity());

		  optimizer.addEdge(edge);
	  }
	  //执行优化
	  optimizer.initializeOptimization();
	  optimizer.optimize(10);

	  //返回
	  return v->estimate();
	  
}

//后端BA非线性优化
void optimize_pose(g2o::SparseOptimizer& optimizer)
{
	//构建图优化，先设定g2o
       //设定矩阵块：每个误差项优化变量维度为6,误差量维度为2（R，T对应的李代数,误差量维度（u,v））
       typedef g2o::BlockSolver< g2o::BlockSolverTraits<9,2> >   Block;

       //设定线性方程求解器(稀疏矩阵)
       Block::LinearSolverType* linearSolver=new g2o::LinearSolverCholmod<Block::PoseMatrixType>();
       Block* solver_ptr = new Block(linearSolver);//矩阵块求解器

       //设置非线性优化方法-列文伯格
       g2o::OptimizationAlgorithmLevenberg* solver=new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    
       //优化函数
       //g2o::SparseOptimizer optimizer;
       optimizer.setAlgorithm(solver);
	
}

//添加point顶点和边（pointxyz）
void add_Vertex_and_edge(g2o::SparseOptimizer& optimizer,cv::Mat& rotation_mat_cur2last,cv::Mat& transform_mat_cur2last,std::vector<cv::Point3f>& Vec_Point3f_Cur,
                          std::vector<cv::Point2f>& Vec_Point2f_Last,int count_vertex_id,int count_edge_id)
{
       //添加位姿顶点
	   g2o::VertexSE3Expmap* v_se3=new g2o::VertexSE3Expmap();
	   Eigen::Matrix3d m_rot;                  //cv::mat 2 eigen::matrix
	   cv::cv2eigen(rotation_mat_cur2last,m_rot);
	   Eigen::Vector3d m_trans;
	   cv::cv2eigen(transform_mat_cur2last,m_trans);
       g2o::SE3Quat se3(m_rot,m_trans);

       v_se3->setEstimate(se3);//设定初始值
       v_se3->setId(count_vertex_id);
	   count_vertex_id++;
       optimizer.addVertex(v_se3);
	   
      //添加point顶点和边
	  for (size_t i = 0; i < Vec_Point3f_Cur.size(); i++)
	  {
		/* code for loop body */  
	     g2o::VertexSBAPointXYZ* v_XYZ=new g2o::VertexSBAPointXYZ();
	     Eigen::Vector3d pointXYZ;                  //cv::mat 2 eigen::matrix
         pointXYZ[0]=Vec_Point3f_Cur[i].x;
         pointXYZ[1]=Vec_Point3f_Cur[i].y;
	     pointXYZ[2]=Vec_Point3f_Cur[i].z;
         v_XYZ->setEstimate(pointXYZ);//设定初始值
         v_XYZ->setId(count_vertex_id);
		 count_vertex_id++;
         optimizer.addVertex(v_XYZ);
	   
	   //添加边
	   	  
		  g2o::EdgeProjectXYZ2UV*edge=new g2o::EdgeProjectXYZ2UV();
		  edge->setId(count_edge_id);
		  count_edge_id++;
		  edge->setVertex(0,v_XYZ);//point vertex
		  edge->setVertex(1,v_se3);//pose vertex
         // set the ith vertex on the hyper-edge to the pointer supplied
		  edge->setMeasurement(Eigen::Vector2d(Vec_Point2f_Last[i].x,Vec_Point2f_Last[i].y));
		  edge->setInformation(Eigen::Matrix2d::Identity());
          g2o::CameraParameters*cam=new g2o::CameraParameters(668.9363,Eigen::Vector2d(625.3835,357.4095),63.0);
		  edge->_cam=cam;
		  optimizer.addEdge(edge);
       
	  }

}