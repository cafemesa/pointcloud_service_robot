#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/LaserScan.h>
#define PI 3.14159265

class astraFilter {

	public:
		astraFilter();
		void pcCallback(const sensor_msgs::PointCloud2ConstPtr &input);
		float robotHeight, sensorHeight, resolution;
		int visionAngle, position;
		std::string topicSubscriber, topicPublisher, frameID;

		struct Point3D
		{
			float x;
			float y;
			float z;
			Point3D(float x_, float y_, float z_)
				: x(x_), y(y_), z(z_)
			{
			}
		};

	private:
		ros::NodeHandle node_, ns_;
		ros::Publisher scanPub;
		ros::Subscriber pcSub;
};

astraFilter::astraFilter() : node_("~"),
					 topicSubscriber("/camera/depth_registered/points"),
					 topicPublisher("/scan"),
					 frameID("base_link"),
					 robotHeight(0.85),
					 sensorHeight(0.74),
					 resolution(0.2),
					 visionAngle(360)
{
	/** Get parameters from the launch file */
	node_.param("topicSubscriber", topicSubscriber, topicSubscriber);
	node_.param("topicPublisher", topicPublisher, topicPublisher);
	node_.param("frameID", frameID, frameID);
	node_.param("robotHeight", robotHeight, robotHeight);
	node_.param("sensorHeight", sensorHeight, sensorHeight);
	node_.param("resolution", resolution, resolution);
	node_.param("visionAngle", visionAngle, visionAngle);

	/** Define Subscriber */
	pcSub = ns_.subscribe(topicSubscriber, 50, &astraFilter::pcCallback, this);

	/** Define Publisher */	
	scanPub = ns_.advertise<sensor_msgs::LaserScan>(topicPublisher, 1, false);
}

void astraFilter::pcCallback (const sensor_msgs::PointCloud2ConstPtr& input)
{ 
	
	int numPts = input->height * input->width;
	char* raw3DPtsData = (char*)(input->data.data());
	std::vector<Point3D> pc(numPts, Point3D(0.,0.,0.));

	pcl::PointCloud<pcl::PointXYZ> laserCloudIn;
	pcl::fromROSMsg(*input, laserCloudIn);

	/** Set scan parameters */
	ros::Time scan_time = ros::Time::now();
	sensor_msgs::LaserScan scan;
	scan.header.stamp = scan_time;
	scan.header.frame_id = frameID;
	scan.angle_min = PI;
	scan.angle_max = -PI;
	scan.angle_increment = -2 * PI / (360 / resolution);
	scan.time_increment = 1 / (600 * (360 / resolution));
	scan.range_min = 0.2;
	scan.range_max = 4.0;
	scan.ranges.resize(360 / resolution);
	scan.intensities.resize(360 / resolution);

	
	for (int i = 0; i < laserCloudIn.size(); i++)
	{
		float* base = (float*)(raw3DPtsData + i * input->point_step);
		Point3D point3d(0.,0.,0.);       

		laserCloudIn[i].x=base[2];
		laserCloudIn[i].y=base[0];
		laserCloudIn[i].z=-base[1];

		float hip = sqrt(pow(laserCloudIn[i].x, 2) + pow(laserCloudIn[i].y, 2));
		float hangle = (asin((laserCloudIn[i].x) / hip)) * 180 / PI;

		/** Discard points */
		if (laserCloudIn[i].z > robotHeight - sensorHeight ||
			(-sensorHeight - 0.1 <= laserCloudIn[i].z && laserCloudIn[i].z < -sensorHeight + 0.05) ||
			(visionAngle <= 180 && laserCloudIn[i].x <= 0) ||
			(visionAngle < 180 && laserCloudIn[i].x > 0 && hangle < 90 - (visionAngle / 2)) ||
			(visionAngle > 180 && visionAngle < 360 && laserCloudIn[i].x < 0 && abs(hangle) > (visionAngle - 180) / 2) ||
			(isnan(laserCloudIn[i].x)==1 || isnan(laserCloudIn[i].y)==1) || hip>4.0)
		{
			continue;
		}

		/** Stairs/holes detection */
		if (laserCloudIn[i].z < -sensorHeight - 0.1)
		{
			float hip2 = sqrt(pow(hip, 2) + pow(laserCloudIn[i].z, 2));
			float hangle2 = asin((laserCloudIn[i].z / hip2) * sin(1.5708)) * 180 / PI;
			if (hangle2 < -14)
			{
				hip = 0.22;
				laserCloudIn[i].x = hip * sin(hangle * PI / 180);
				if (laserCloudIn[i].y > 0)
				{
					laserCloudIn[i].y = sqrt(pow(hip, 2) - pow(laserCloudIn[i].x, 2));
				}
				else
				{
					laserCloudIn[i].y = -sqrt(pow(hip, 2) - pow(laserCloudIn[i].x, 2));
				}
			}
			if (hangle2 > -14)
			{
				hip2 = sensorHeight * sin(PI / 2) / sin((2 - hangle2) * PI / 180);
				hip = sqrt(pow(hip2, 2) - pow(sensorHeight, 2));
				laserCloudIn[i].x = hip * sin(hangle * PI / 180);
				if (laserCloudIn[i].y > 0)
				{
					laserCloudIn[i].y = sqrt(pow(hip, 2) - pow(laserCloudIn[i].x, 2));
				}
				else
				{
					laserCloudIn[i].y = -sqrt(pow(hip, 2) - pow(laserCloudIn[i].x, 2));
				}
			}
		}

		/** Get the position in the array */
		if (laserCloudIn[i].y > 0)
		{
			position = (180.0 + hangle) / resolution;
		}
		else if (laserCloudIn[i].x > 0 && laserCloudIn[i].y <= 0)
		{
			position = (360.0 - hangle) / resolution;
		}
		else
		{
			position = -hangle / resolution;
		}

		/** Add near point to the scan array */
		if (scan.ranges[position] == 0 || hip < scan.ranges[position])
		{
			scan.ranges[position] = hip;
			scan.intensities[position] = 100;
		}
	}

	// Publishing of the LaserScan topic
    scanPub.publish(scan);
}

/** Main function*/
int main(int argc, char **argv)
{
	ros::init(argc, argv, "astraFilter");
	astraFilter astrafilter;
	ros::spin();
	return 0;
}
