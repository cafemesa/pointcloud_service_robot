#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <math.h>
#include <iostream>
#include <fstream>
#include <string>
#include <std_msgs/String.h>
#include "nav_msgs/Odometry.h"
#define PI 3.14159265


class pc2cs
{
public:
	pc2cs();

	void poseCallback(const sensor_msgs::PointCloud2ConstPtr &input);	
	
	std::string pose_topic_sub, fileName;	

	std::ofstream fs;

private:
	ros::NodeHandle node_, ns_;
	ros::Subscriber PoseSub;
};

pc2cs::pc2cs() : node_("~"),
					 pose_topic_sub("/People_PC"),
					 fileName("pc2csv")
{
	/** Get parameters from the launch file */
	node_.param("pose_topic_sub", pose_topic_sub, pose_topic_sub);
	node_.param("fileName", fileName, fileName);

	/** Define Subscriber */
	PoseSub = ns_.subscribe(pose_topic_sub, 50, &pc2cs::poseCallback, this);

	fs.open(fileName.c_str());
    fs << "Position X" << "," << "Position Y" << std::endl;  
}

void pc2cs::poseCallback(const sensor_msgs::PointCloud2ConstPtr &input)
{
	pcl::PointCloud<pcl::PointXYZ> laserCloudIn;
	pcl::fromROSMsg(*input, laserCloudIn);

	for (int i = 0; i < laserCloudIn.size(); i++)
	{
		fs << laserCloudIn[i].x << "," << laserCloudIn[i].y << std::endl;  
	}	
}


/** Main function*/
int main(int argc, char **argv)
{
	ros::init(argc, argv, "velodyne_people");
	pc2cs filter;
	
    
	ros::spin();
	return 0;
}
