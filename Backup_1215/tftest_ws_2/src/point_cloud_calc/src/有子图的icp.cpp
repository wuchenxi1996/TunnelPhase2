#include "calc_header.h"


Convert::Convert()
{
	std::cout << "Please input  maximum time for Sampling: " << std::endl;
	std::cin >> rotation_time;
	if(std::cin.fail()){
		rotation_time = 10;
		ROS_INFO("Input is not a valid number, Program will run under default time: 10s!");
	}

	size_limit = 30000 * rotation_time; //1875*16=30000
	pub = node.advertise<sensor_msgs::PointCloud>("/Cloud", 100);
	point_num = 30000;
	pointcloud_base.points.resize(30000*rotation_time);
	pointcloud_last.points.resize(30000*rotation_time);
	pc_sub.points.resize(150000);
	pointcloud_base_ptr = pointcloud_base.makeShared();
	pointcloud_last_ptr = pointcloud_last.makeShared();
	pc_sub_ptr =  pc_sub.makeShared();
	pose_last.x = 0.0;
	pose_last.y = 0.0;
	pose_last.z = 0.0;
	pose_last.roll = 0.0;
	pose_last.pitch = 0.0;
	pose_last.yaw = 0.0;
	//pointcloud.header.frame_id = "vlp16_link";
	//pointcloud_base.header.frame_id = "base_link";
	//pointcloud_last.header.frame_id = "base_link";
	current_size = 0;
	iteration = 0;
	std::cout << "Initialization Complete." << std::endl;
}

Convert::~Convert()
{

}

void callback(const sensor_msgs::PointCloud2ConstPtr &pc_in, Convert *convertob_)
{
	convertob_->CombinedCallback(pc_in);
}

void Convert::TransformToMatrix(const tf::StampedTransform& transform, Eigen::Matrix4f& Ematrix){
	Eigen::Translation3f tl_btol(
		transform.getOrigin().getX(),
		transform.getOrigin().getY(),
		transform.getOrigin().getZ());
	double roll, pitch, yaw;
	tf::Matrix3x3(transform.getRotation()).getEulerYPR(roll, pitch, yaw);
	Eigen::AngleAxisf rot_x_btol(roll, Eigen::Vector3f::UnitX());
	Eigen::AngleAxisf rot_y_btol(pitch, Eigen::Vector3f::UnitY());
	Eigen::AngleAxisf rot_z_btol(yaw, Eigen::Vector3f::UnitZ());
	Ematrix = (tl_btol * rot_z_btol * rot_y_btol *rot_x_btol).matrix();
}

void Convert::CombinedCallback(const sensor_msgs::PointCloud2ConstPtr& pc_in)
{
	tf::StampedTransform  transform = get_transform();
	Eigen::Matrix4f eigen_tf;
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_in (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_trans_ptr (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromROSMsg(*pc_in, *pcl_in);
	TransformToMatrix(transform, eigen_tf);
/*----------- Motion detection and filter to get rid of the useless points ----------
if(MotionFilter(pose_last) == 0){
	return;
}*/
/*----------registration of pointclouds----------*/

if(current_size < size_limit){
	if(iteration == 0){
		pcl::transformPointCloud(*pcl_in, *pcl_trans_ptr,eigen_tf);
		pointcloud_base = *pcl_trans_ptr;
		current_size += 30000;
		pointcloud_last_ptr = pointcloud_base.makeShared();
		pointcloud_last = *pointcloud_last_ptr;
		pc_arr.push_back(*pointcloud_last_ptr);
		pc_sub = pointcloud_last;
		pc_sub_ptr = pc_sub.makeShared();
		++iteration;
		ROS_INFO("Iteration begins.");
	}
	else if(iteration < 5){
		pcl_trans_ptr = registration_icp(pcl_in, pc_sub_ptr, eigen_tf).makeShared();
		pc_arr.push_back(*pcl_trans_ptr);
		pointcloud_last = pointcloud_last + (*pcl_trans_ptr);
  		pointcloud_last_ptr = pointcloud_last.makeShared();
		for(int i = 0; i < iteration; ++i){
			pc_sub = pc_sub + (*pcl_trans_ptr);
			current_size += 30000;
		}
		++iteration;
		ROS_INFO("Iteration +, %d.", iteration);
	}
	else if(iteration >= 5){
		(*pcl_trans_ptr) = registration_icp(pcl_in, pc_sub_ptr, eigen_tf);
		pc_arr.pop_front();
		pc_arr.push_back(*pcl_trans_ptr);
		pointcloud_last = pointcloud_last + (*pcl_trans_ptr);
  		pointcloud_last_ptr = pointcloud_last.makeShared();
		pc_sub_ptr = pc_arr.front().makeShared();
		pc_sub= *pc_sub_ptr;
		for(int i = 1; i < 5; ++i){
			pc_sub = pc_sub + pc_arr[i];
		}
		current_size += 30000;
		++iteration;
		ROS_INFO("Iteration ++, %d.", iteration);
	}
}
else{
	std::cout<< current_size <<std::endl;
	pcl::io::savePCDFileASCII ("/home/pibot/Documents/phase2.pcd", *pointcloud_last_ptr);
	pcl::KdTreeFLANN<pcl::PointXYZ> cloud_kdTree;
//	cloud_kdTree.setInputCloud(pointcloud_base);
	ros::shutdown();
	ROS_INFO("Process finished.");
}
}
bool Convert::MotionFilter(Pose pose_last)
{
	Pose pose;
	get_position(get_transform(), pose);
	if(abs(pose.x - pose_last.x) >= 0.01 | abs(pose.y - pose_last.y) >= 0.01 | abs(pose.z - pose_last.z) >= 0.01 |
		abs(pose.yaw - pose_last.yaw) >=0.01 | abs(pose.pitch - pose_last.pitch) >=0.01 | abs(pose.roll - pose_last.roll) >=0.01){
		pose_last.x = pose.x;
		pose_last.y = pose.y;
		pose_last.z = pose.z;
		pose_last.roll = pose.roll;
		pose_last.pitch = pose.pitch;
		pose_last.yaw = pose.yaw;
		return true;

	}
	else{
		return false;
	}
}

void signalHandler(int signum)
{
	std::cout << "Process stopped manually." << std::endl;
	ros::shutdown();
}
Pose Convert::get_position(tf::StampedTransform transform, Pose pose) // get lidar pose
{
	geometry_msgs::TransformStamped transform_pose;
	tf::transformStampedTFToMsg(transform, transform_pose);
	pose.x = transform.getOrigin().x();
	pose.y = transform.getOrigin().y();
	pose.z = transform.getOrigin().z();
	tf::Matrix3x3(transform.getRotation()).getEulerYPR(pose.roll, pose.pitch, pose.yaw);
	return pose;
}

tf::StampedTransform Convert::get_transform() //get transform object
{
	try
	{
	listener.waitForTransform("/base_link", "/vlp16_link", ros::Time(0), ros::Duration(1.0));//ros::Time(0)表示当前时刻最近的一帧变换，不能写成ros::Time::now();
	listener.lookupTransform("/base_link", "/vlp16_link", ros::Time(0), transform);
	}
	catch(tf::TransformException &ex)
	{
	ROS_ERROR("%s", ex.what());
	ros::Duration(1.0).sleep();
	}

	return transform;
}
 pcl::PointCloud<pcl::PointXYZ> Convert::registration_icp(	pcl::PointCloud<pcl::PointXYZ>::Ptr source_pc, 	pcl::PointCloud<pcl::PointXYZ>::Ptr aim_pc, Eigen::Matrix4f& Ematrix){
	 pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	 icp.setMaxCorrespondenceDistance(0.10);
	 icp.setTransformationEpsilon(1e-6);
	 icp.setEuclideanFitnessEpsilon(0.1);
	 icp.setMaximumIterations (10);
	 pcl::PointCloud<pcl::PointXYZ> result_pc;
	 icp.setInputSource(source_pc);
	 icp.setInputTarget(aim_pc);
	 icp.align(result_pc, Ematrix);
	 Eigen::Matrix4f final_trans = icp.getFinalTransformation();
	 std::cout << "result size is: " << result_pc.points.size()<<std::endl;
	 return result_pc;
 }
