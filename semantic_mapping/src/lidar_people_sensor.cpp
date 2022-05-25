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


class semanticMappingPeople
{
public:
	semanticMappingPeople();

	void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg);
	void poseAMCLCallback(const nav_msgs::Odometry::ConstPtr& msg);	

	void filter(const sensor_msgs::PointCloud2ConstPtr &input);
	pcl::PointCloud<pcl::PointXYZ> cloudfilter(pcl::PointCloud<pcl::PointXYZ> LegsCloud, float separation, float minWidth, float maxWidth);
	pcl::PointCloud<pcl::PointXYZ> cloudmatcher(pcl::PointCloud<pcl::PointXYZ> PeopleCloud, pcl::PointCloud<pcl::PointXYZ> LegsCloud, pcl::PointCloud<pcl::PointXYZ> TrunkCloud, float separation);
	pcl::PointCloud<pcl::PointXYZ> mapfilter(pcl::PointCloud<pcl::PointXYZ> PeopleCloud);
	pcl::PointCloud<pcl::PointXYZ> rotatePC(pcl::PointCloud<pcl::PointXYZ> PeopleCloud);

	float robotheight, sensorheight, resolution, legs_begin, legs_end, trunk_begin, trunk_end, pose_x, pose_y, initial_angle2, initial_angle, initial_robotpose_x, initial_robotpose_y, initial_robotpose_a;
	int visionangle, position, Ncloud;
	std::string topic_pub1, topic_pub2, topic_pub3, topic_pub4, sensor_topic_sub, pose_topic_sub, map_topic_sub, frame_id;
	
	int** maparray;
	int  map_width = 0, map_height = 0;
	float originx, originy;

	std::ofstream fs;
	std::string filename = "people_positions.csv";

private:
	ros::NodeHandle node_, ns_;
	ros::Publisher people_pub, legs_pub, trunk_pub, laser_pub;
	ros::Subscriber SensorSub, PoseSub, MapSub;
};

semanticMappingPeople::semanticMappingPeople() : node_("~"),
					 robotheight(0.8),
					 sensorheight(0.74),
					 visionangle(360),
					 resolution(0.2),
					 legs_begin(0.25),
					 legs_end(0.60),
					 trunk_begin(0.80),
					 trunk_end(1.50),
					 initial_robotpose_x(0.0),
					 initial_robotpose_y(0.0),
					 initial_robotpose_a(0.0),
					 topic_pub1("/People_PC"),
					 topic_pub2("/Legs_PC"),
					 topic_pub3("/Trunk_PC"),
					 topic_pub4("/Laser_PC"),
					 sensor_topic_sub("/velodyne_points"),
					 pose_topic_sub("/odom"),
					 map_topic_sub("/map"),
					 frame_id("map")
{
	/** Get parameters from the launch file */
	node_.param("robot_height", robotheight, robotheight);
	node_.param("sensor_height", sensorheight, sensorheight);
	node_.param("horizontal_fov", visionangle, visionangle);
	node_.param("resolution", resolution, resolution);
	node_.param("legs_begin", legs_begin, legs_begin);
	node_.param("legs_end", legs_end, legs_end);
	node_.param("trunk_begin", trunk_begin, trunk_begin);
	node_.param("trunk_end", trunk_end, trunk_end);
	node_.param("initial_robotpose_x", initial_robotpose_x, initial_robotpose_x);
	node_.param("initial_robotpose_y", initial_robotpose_y, initial_robotpose_y);
	node_.param("initial_robotpose_a", initial_robotpose_a, initial_robotpose_a);

	node_.param("topic_pub_people", topic_pub1, topic_pub1);
	node_.param("topic_pub_legs", topic_pub2, topic_pub2);
	node_.param("topic_pub_trunk", topic_pub3, topic_pub3);
	node_.param("topic_pub_laser", topic_pub4, topic_pub4);
	node_.param("sensor_topic_sub", sensor_topic_sub, sensor_topic_sub);
	node_.param("pose_topic_sub", pose_topic_sub, pose_topic_sub);
	node_.param("map_topic_sub", map_topic_sub, map_topic_sub);
	
	node_.param("frame_id", frame_id, frame_id);

	pose_x = initial_robotpose_x;
	pose_y = initial_robotpose_y;
	initial_angle2 = initial_robotpose_a;

	/** Define Subscriber */
	SensorSub = ns_.subscribe(sensor_topic_sub, 50, &semanticMappingPeople::filter, this);
	MapSub = ns_.subscribe(map_topic_sub, 50, &semanticMappingPeople::mapCallback, this);

	/** Define Publisher */
	people_pub = ns_.advertise<sensor_msgs::PointCloud2>(topic_pub1, 1, false);
	legs_pub = ns_.advertise<sensor_msgs::PointCloud2>(topic_pub2, 1, false);
	trunk_pub = ns_.advertise<sensor_msgs::PointCloud2>(topic_pub3, 1, false);
	laser_pub = ns_.advertise<sensor_msgs::PointCloud2>(topic_pub4, 1, false);

	Ncloud = 0;


	fs.open(filename.c_str());
    fs << "Position X" << "," << "Position Y" << std::endl;  
}

void semanticMappingPeople::filter(const sensor_msgs::PointCloud2ConstPtr &input)
{


	if(map_width != 0 && map_height != 0){
		initial_angle=initial_angle2;
		Ncloud++;

		pcl::PointCloud<pcl::PointXYZ> laserCloudIn, LegsCloud, TrunkCloud, PeopleCloud, LaserCloud;
		pcl::PointXYZ pointlegs1, pointlegs2, pointtrunk1, pointtrunk2;
		pcl::fromROSMsg(*input, laserCloudIn);

		unsigned int num_readings = 360 / resolution;
		LegsCloud.points.resize(num_readings);
		TrunkCloud.points.resize(num_readings);
		PeopleCloud.points.resize(num_readings);

		LaserCloud.points.resize(num_readings);
		double ranges[num_readings]={0};
		double intensities[num_readings]={0};

		float addpos = 0;
		double LegsRanges[num_readings] = {0}, TrunkRanges[num_readings] = {0};
		int position = 0, postrunk10 = 0, postrunk30 = 0, poslegs10 = 0, poslegs30 = 0;

		for (int i = 0; i < laserCloudIn.size(); i++)
		{
			float hip = sqrt(pow(laserCloudIn[i].x, 2) + pow(laserCloudIn[i].y, 2));
			float hangle = (asin((laserCloudIn[i].x) / hip)) * 180 / PI;

			/** Discard points */
			if ((laserCloudIn[i].z < -sensorheight + 0.10) || (laserCloudIn[i].z > -sensorheight + 2.0) || isnan(laserCloudIn[i].x)==1 || isnan(laserCloudIn[i].y)==1 ||
				fabs(laserCloudIn[i].x)<0.01 || fabs(laserCloudIn[i].y)<0.01 ||
				(visionangle <= 180 && laserCloudIn[i].x <= 0) ||
				(visionangle < 180 && laserCloudIn[i].x > 0 && hangle < 90 - (visionangle / 2)) ||
				(visionangle > 180 && visionangle < 360 && laserCloudIn[i].x < 0 && abs(hangle) > (visionangle - 180) / 2))
			{
				continue;
			}

			// Get the position in the arrays for get the nearest points
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

			if ((ranges[position]==0 || hip < ranges[position]) )
			{
				ranges[position]=hip;
				intensities[position]=0;
				LaserCloud.points[position].x = laserCloudIn[i].x;
				LaserCloud.points[position].y = laserCloudIn[i].y;
			}	
			
			// Generate  Legs Point Cloud
			if (-sensorheight + legs_begin < laserCloudIn[i].z && laserCloudIn[i].z < -sensorheight + legs_end)
			{
				// Extraction of nearest points
				if (LegsRanges[position] == 0 || hip < LegsRanges[position])
				{
					
					LegsCloud.points[position].x = laserCloudIn[i].x;
					LegsCloud.points[position].y = laserCloudIn[i].y;
					LegsCloud.points[position].z = 0; // Projection onto 2D
					LegsRanges[position] = hip;
				}
			}

			// Generate  Trunk Point Cloud
			if (-sensorheight + trunk_begin < laserCloudIn[i].z && laserCloudIn[i].z < -sensorheight + trunk_end)
			{
				// Extraction of nearest points
				if (TrunkRanges[position] == 0 || hip < TrunkRanges[position])
				{
					TrunkCloud.points[position].x = laserCloudIn[i].x;
					TrunkCloud.points[position].y = laserCloudIn[i].y;
					TrunkCloud.points[position].z = 0; // Projection onto 2D
					TrunkRanges[position] = hip;
				}
			}
		}

		
		LegsCloud = cloudfilter(LegsCloud, 0.15, 0.03, 0.65);
		TrunkCloud = cloudfilter(TrunkCloud, 0.25, 0.25, 0.85);
		PeopleCloud = cloudmatcher(PeopleCloud, LegsCloud, TrunkCloud, 0.35);	
		PeopleCloud = mapfilter(PeopleCloud);
		
		LegsCloud = rotatePC(LegsCloud);
		TrunkCloud = rotatePC(TrunkCloud);
		LaserCloud = rotatePC(LaserCloud);

		//Publish LaserCloud
		sensor_msgs::PointCloud2 LaserCloud_output;
		pcl::toROSMsg(LaserCloud, LaserCloud_output);
		LaserCloud_output.header.frame_id = frame_id;
		laser_pub.publish(LaserCloud_output);	


		//Publish PeopleCloud
		sensor_msgs::PointCloud2 PeopleCloud_output;
		pcl::toROSMsg(PeopleCloud, PeopleCloud_output);
		PeopleCloud_output.header.frame_id = frame_id;
		people_pub.publish(PeopleCloud_output);	

		//Publish LegsCloud
		sensor_msgs::PointCloud2 LegsCloud_output;
		pcl::toROSMsg(TrunkCloud, LegsCloud_output);
		LegsCloud_output.header.frame_id = "map";
		legs_pub.publish(LegsCloud_output);

		//Publish TrunkCloud
		sensor_msgs::PointCloud2 TrunkCloud_output;
		pcl::toROSMsg(TrunkCloud, TrunkCloud_output);
		TrunkCloud_output.header.frame_id = "map";
		trunk_pub.publish(TrunkCloud_output);
	}

	

	
}

/** Legs and Trunk filters */
pcl::PointCloud<pcl::PointXYZ> semanticMappingPeople::cloudfilter(pcl::PointCloud<pcl::PointXYZ> Cloud, float separation, float minWidth, float maxWidth)
{
	pcl::PointXYZ point1, point2;
	int pos1 = 0, pos2 = 0;

	for (int i = 0; i < Cloud.size(); i++)
	{
		if (fabs(Cloud.points[i].x) < 0.1 && fabs(Cloud.points[i].y) < 0.1)
		{
			continue;
		}

		// Save the first point of the sample
		if (pos2 == 0)
		{
			point1.x = point2.x = Cloud.points[i].x;
			point1.y = point2.y = Cloud.points[i].y;
			pos2++;
		}
		else
		{
			float hiplegs1 = sqrt(pow(Cloud.points[i].x - point1.x, 2.0) + pow(Cloud.points[i].y - point1.y, 2.0));

			// Check if a new cluster appear in the field of view
			if (hiplegs1 > separation)
			{
				float hiplegs2 = sqrt(pow(point1.x - point2.x, 2.0) + pow(point1.y - point2.y, 2.0));
				// Chech if the detecte object have the posible dimension for a leg
				if (minWidth < hiplegs2 && hiplegs2 < maxWidth)
				{
					//Save de average   between the first and the last point in the group
					Cloud.points[pos1].x = (point1.x + point2.x) / 2;
					Cloud.points[pos1].y = (point1.y + point2.y) / 2;
					Cloud.points[pos1].z = 0;
					pos1++;
				}
				// Store the first point of the new detection
				point2.x = Cloud.points[i].x;
				point2.y = Cloud.points[i].y;
			}
			//Store the last point to compare in the nest round
			point1.x = Cloud.points[i].x;
			point1.y = Cloud.points[i].y;
		}
	}
	Cloud.points.resize(pos1);
	return Cloud;
}

/** Cloud matcher */
pcl::PointCloud<pcl::PointXYZ> semanticMappingPeople::cloudmatcher(pcl::PointCloud<pcl::PointXYZ> PeopleCloud, pcl::PointCloud<pcl::PointXYZ> LegsCloud, pcl::PointCloud<pcl::PointXYZ> TrunkCloud, float separation)
{
	int pospos = 0;
	for (int i = 0; i < TrunkCloud.size(); i++)
	{
		for (int j = 0; j < LegsCloud.size(); j++)
		{
			// Get the distance between the points in Trunk PointCloud and Legs PointCloud
			float hip1 = sqrt(pow(TrunkCloud.points[i].x - LegsCloud.points[j].x, 2.0) + pow(TrunkCloud.points[i].y - LegsCloud.points[j].y, 2.0));
			// If exist a match between the two clouds the point in Trunk PointCloud us stored in People PointCloud
			if (hip1 < separation)
			{
				
				// Store the first point
				if (pospos == 0)
				{
					//std::cout << pospos << "\n";
					PeopleCloud.points[pospos].x = TrunkCloud.points[i].x;
					PeopleCloud.points[pospos].y = TrunkCloud.points[i].y;
					PeopleCloud.points[pospos].z = 0;
					pospos++;
				}
				else
				{
					
					float hip10 = sqrt(pow(PeopleCloud.points[pospos - 1].x - TrunkCloud.points[i].x, 2.0) + pow(PeopleCloud.points[pospos - 1].y - TrunkCloud.points[i].y, 2.0));
					if (hip10 > 0.6)
					{
						PeopleCloud.points[pospos].x = TrunkCloud.points[i].x;
						PeopleCloud.points[pospos].y = TrunkCloud.points[i].y;
						PeopleCloud.points[pospos].z = 0;
						pospos++;
					}
				}
			}
		}
	}

	// People PointCloud resize
	PeopleCloud.points.resize(pospos);
	return PeopleCloud;
}

/** Map filter */
pcl::PointCloud<pcl::PointXYZ> semanticMappingPeople::mapfilter(pcl::PointCloud<pcl::PointXYZ> PeopleCloud)
{
	int lastpos = 0;
	for (int i = 0; i < PeopleCloud.size(); i++)
	{
		int position = 0;
		float angle_min = 3 * PI / 2;
		float angle_increment = -2 * PI / (360 / resolution);
		float hip = sqrt((PeopleCloud[i].x) * (PeopleCloud[i].x) + ((PeopleCloud[i].y) * (PeopleCloud[i].y)));
		float hangle = (asin((PeopleCloud[i].x) / hip)) * 180 / PI;

		if (PeopleCloud[i].y > 0)
		{
			position = (180.0 + hangle) / (double)resolution;
		}
		else if (PeopleCloud[i].x > 0 && PeopleCloud[i].y <= 0)
		{
			position = (360.0 - hangle) / (double)resolution;
		}
		else
		{
			position = -hangle / (double)resolution;
		}

		float finalangle = ((double)position * angle_increment) + angle_min;

		float laserpointx = hip * cos(-finalangle - (double)initial_angle - PI);
		float laserpointy = hip * sin(-finalangle - (double)initial_angle - PI);

		PeopleCloud[i].x = -laserpointx + pose_x;
		PeopleCloud[i].y = laserpointy + pose_y;

		int posxpx = ((/** - origin X in map.yaml */originx-PeopleCloud[i].x) / (-0.05));
		int posypx = ((/** - origin Y in map.yaml */originy-PeopleCloud[i].y) / (-0.05));
		int spacecount = 0;

		if (maparray[posxpx][posypx] >5 || 
			PeopleCloud[i].x<-3 || 
			PeopleCloud[i].x>25 || 
			(PeopleCloud[i].x>3.5 && PeopleCloud[i].x<5 && PeopleCloud[i].y>-1.2 && PeopleCloud[i].y<0.6) //robot position
			)
		{
			continue;
		}
		else {
			PeopleCloud[lastpos].x = PeopleCloud[i].x;
			PeopleCloud[lastpos].y = PeopleCloud[i].y;
			fs << PeopleCloud[lastpos].x << "," << PeopleCloud[lastpos].y << std::endl; 
			lastpos++;
		}
	}
	PeopleCloud.resize(lastpos);
	return PeopleCloud;
}

/** Rotate */
pcl::PointCloud<pcl::PointXYZ> semanticMappingPeople::rotatePC(pcl::PointCloud<pcl::PointXYZ> PeopleCloud)
{
	int lastpos = 0;
	for (int i = 0; i < PeopleCloud.size(); i++)
	{
		int position = 0;
		float angle_min = 3 * PI / 2;
		float angle_increment = -2 * PI / (360 / resolution);
		float hip = sqrt((PeopleCloud[i].x) * (PeopleCloud[i].x) + ((PeopleCloud[i].y) * (PeopleCloud[i].y)));
		float hangle = (asin((PeopleCloud[i].x) / hip)) * 180 / PI;

		if (PeopleCloud[i].y > 0)
		{
			position = (180.0 + hangle) / (double)resolution;
		}
		else if (PeopleCloud[i].x > 0 && PeopleCloud[i].y <= 0)
		{
			position = (360.0 - hangle) / (double)resolution;
		}
		else
		{
			position = -hangle / (double)resolution;
		}

		float finalangle = ((double)position * angle_increment) + angle_min;

		float laserpointx = hip * cos(-finalangle - (double)initial_angle - PI);
		float laserpointy = hip * sin(-finalangle - (double)initial_angle - PI);

		PeopleCloud[i].x = -laserpointx + pose_x;
		PeopleCloud[i].y = laserpointy + pose_y;	
	}
	return PeopleCloud;
}

/** Map Callback */
void semanticMappingPeople::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
	std_msgs::Header header = msg->header;
	nav_msgs::MapMetaData info = msg->info;

    

    
	maparray = new int*[info.width];
    for(int i = 0; i < info.width; ++i)
    {
        maparray[i] = new int[info.height];
    }

    for (int x = 0; x < info.width; x++)
	{
		for (int y = 0; y < info.height; y++)
		{
            int intensitycustom = (static_cast<int>(msg->data[x + info.width * y]));
            if (intensitycustom<0 || intensitycustom>5){ 
                maparray[x][y] = 100;
            }
            else {
                int counter2 = 0;
                int tol = 4;
                int startx = x-tol;
                int starty = y-tol; 
                int endx = x+tol+1;
                int endy = y+tol+1; 
                if (startx<0){startx = 0;}
                if (starty<0){starty = 0;}
                if (endx>info.width){endx = info.width;}
                if (endy>info.height){endy = info.height;}

                for (int u = startx ; u < endx ; u++)
                {
                    for (int v = starty ; v < endy; v++)
                    {                        
                        int intensitycustom2 = (static_cast<int>(msg->data[u + info.width * v]));
                        if (intensitycustom2<0 || intensitycustom2>5){ 
                            counter2++;
                        }
                    }
                }

                if (counter2==0){
                    maparray[x][y] = 0;
                } else {
                    maparray[x][y] = 100;
                }
            }            
		}
	}

	map_width = info.width;
    map_height = info.height;
	originx = info.origin.position.x;
	originy = info.origin.position.y;
}

/** Main function*/
int main(int argc, char **argv)
{
	ros::init(argc, argv, "velodyne_people");
	semanticMappingPeople filter;
	
    
	ros::spin();
	return 0;
}
