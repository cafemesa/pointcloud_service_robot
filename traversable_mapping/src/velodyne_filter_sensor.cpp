#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/LaserScan.h>
#define PI 3.14159265

class velodyneFilter {

	public:
		velodyneFilter();
		void pcCallback(const sensor_msgs::PointCloud2ConstPtr &input);
		float robotHeight, sensorHeight, resolution;
		float initial_angle, initial_robotpose_x, initial_robotpose_y;
		int visionAngle;
		std::string topicSubscriber, topicPublisher, topicPublisherPC, frameID;

	private:
		ros::NodeHandle node_, ns_;
		ros::Publisher scanPub, scanPcPub;
		ros::Subscriber pcSub;
};

velodyneFilter::velodyneFilter() : node_("~"),
					 topicSubscriber("/velodyne_points"),
					 topicPublisher("/scan"),
					 topicPublisherPC("/scanpc"),
					 frameID("base_link"),
					 initial_angle(0.85),
					 initial_robotpose_x(0.85),
					 initial_robotpose_y(0.85),
					 robotHeight(0.85),
					 sensorHeight(0.74),
					 resolution(0.2),
					 visionAngle(360)
{
	/** Get parameters from the launch file */
	node_.param("topicSubscriber", topicSubscriber, topicSubscriber);
	node_.param("topicPublisher", topicPublisher, topicPublisher);
	node_.param("topicPublisherPC", topicPublisherPC, topicPublisherPC);
	node_.param("frameID", frameID, frameID);
	node_.param("robotHeight", robotHeight, robotHeight);
	node_.param("sensorHeight", sensorHeight, sensorHeight);
	node_.param("resolution", resolution, resolution);
	node_.param("visionAngle", visionAngle, visionAngle);


	node_.param("initial_angle", initial_angle, initial_angle);
	node_.param("initial_robotpose_x", initial_robotpose_x, initial_robotpose_x);
	node_.param("initial_robotpose_y", initial_robotpose_y, initial_robotpose_y);

	/** Define Subscriber */
	pcSub = ns_.subscribe(topicSubscriber, 50, &velodyneFilter::pcCallback, this);

	/** Define Publisher */	
	scanPub = ns_.advertise<sensor_msgs::LaserScan>(topicPublisher, 1, false);
	scanPcPub = ns_.advertise<sensor_msgs::PointCloud2>(topicPublisherPC, 1, false);
}

void velodyneFilter::pcCallback (const sensor_msgs::PointCloud2ConstPtr& input)
{ 

	std::cout << initial_robotpose_x << "\n";

	pcl::PointCloud<pcl::PointXYZ> laserCloudIn, scanout;
	pcl::fromROSMsg(*input, laserCloudIn);

	scanout.resize(360/resolution);
	int scanoutpos = 0;

	//Laser Transform variables
	unsigned int num_readings = 360/resolution;
  	double laser_frequency = 600;
  	double ranges[num_readings]={0};
  	double intensities[num_readings]={0};
	int addpos=0;
	int position=0;	

	// Configuration of parameters for conversion to LaserScan
	ros::Time scan_time = ros::Time::now();
    sensor_msgs::LaserScan scan;
    scan.header.stamp = scan_time;
    scan.header.frame_id = frameID;
    scan.angle_min = 3*PI/2;
    scan.angle_max = -PI/2;
    scan.angle_increment = -2*PI / num_readings;
    scan.time_increment = (1 / laser_frequency) / (num_readings);
    scan.range_min = 0.1;
    scan.range_max = 50.0;
    scan.ranges.resize(num_readings);
    scan.intensities.resize(num_readings);

	for (int i = 0; i < laserCloudIn.size(); i++) {
		
		float hip = sqrt((laserCloudIn[i].x)*(laserCloudIn[i].x)+((laserCloudIn[i].y)*(laserCloudIn[i].y)));
		float hangle = (asin ((laserCloudIn[i].x)/hip))*180/PI;

		// Vertical filter : Points with a height greater than the height of the robot are discarded
		if (laserCloudIn[i].z > robotHeight-sensorHeight || (-sensorHeight-0.1<=laserCloudIn[i].z && laserCloudIn[i].z < -sensorHeight + 0.05)) {continue;}

		// Dicard points outside the area of interest (Horizontal Filter)
		if (visionAngle==180 && laserCloudIn[i].x<=0){continue;}
		if ((visionAngle<180 && laserCloudIn[i].x>0 && hangle<90-(visionAngle/2)) || (visionAngle<180 && laserCloudIn[i].x<=0)){continue;}
		if (visionAngle>180 && visionAngle<360 && laserCloudIn[i].x<0 && abs(hangle)>(visionAngle-180)/2){continue;}

		//Stairs detection
		if (laserCloudIn[i].z < -sensorHeight-0.1){
			float hip2 = sqrt(pow(hip,2)+pow(laserCloudIn[i].z,2));
			float hangle2= asin((laserCloudIn[i].z/hip2)*sin(1.5708))*180/PI;
			if (hangle2<-14)
			{
				hip=0.22;
				laserCloudIn[i].x = hip * sin(hangle*PI/180);
				if (laserCloudIn[i].y>0)
				{
					laserCloudIn[i].y = sqrt(pow(hip,2)-pow(laserCloudIn[i].x,2));
				}
				else
				{
					laserCloudIn[i].y = -sqrt(pow(hip,2)-pow(laserCloudIn[i].x,2));
				}
			}
			if (hangle2>-14)
			{
				hip2=sensorHeight*sin(PI/2)/sin((2-hangle2)*PI/180);
				hip = sqrt(pow(hip2,2)-pow(sensorHeight,2));
				laserCloudIn[i].x = hip * sin (hangle*PI/180);
				if (laserCloudIn[i].y>0)
				{
					laserCloudIn[i].y = sqrt(pow(hip,2)-pow(laserCloudIn[i].x,2));
				}
				else
				{
					laserCloudIn[i].y = -sqrt(pow(hip,2)-pow(laserCloudIn[i].x,2));
				}
			}
		}

		// Get the position in the arrays for get the nearest points
		if (laserCloudIn[i].y>0) {
			position=(180.0+hangle)/resolution;
		}
		else if (laserCloudIn[i].x>0 && laserCloudIn[i].y<=0) {
			position=(360.0-hangle)/resolution;
		}
		else {
			position=-hangle/resolution;			
		}	


		


		if (ranges[position]==0 || hip < ranges[position])
		{
			ranges[position]=hip;
			intensities[position]=0;

			float hippos = sqrt((laserCloudIn[i].x) * (laserCloudIn[i].x) + ((laserCloudIn[i].y) * (laserCloudIn[i].y)));
			float finalangle = ((double)position * scan.angle_increment) + scan.angle_min;
			float laserpointx = hippos * cos(-finalangle - (double)initial_angle - PI);
			float laserpointy = hippos * sin(-finalangle - (double)initial_angle - PI);
			scanout[scanoutpos].x = -laserpointx + initial_robotpose_x;
			scanout[scanoutpos].y = laserpointy + initial_robotpose_y;
			scanoutpos++;
		}					
	}
	
	

	// assign values to the vector of ranges and intensities to publish the LaserScan
    for(unsigned int i = 0; i < num_readings; ++i){
      scan.ranges[i] = ranges[i];
      scan.intensities[i] = intensities[i];
    }  

	// Publishing of the LaserScan topic
    scanPub.publish(scan);

	//Publish TrunkCloud
	sensor_msgs::PointCloud2 TrunkCloud_output;
	pcl::toROSMsg(scanout, TrunkCloud_output);
	TrunkCloud_output.header.frame_id = "map";
	scanPcPub.publish(TrunkCloud_output);
}

/** Main function*/
int main(int argc, char **argv)
{
	ros::init(argc, argv, "velodyneFilter");
	velodyneFilter velodynefilter;
	ros::spin();
	return 0;
}
