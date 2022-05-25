#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <iostream>
#include <iterator>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include "nav_msgs/Odometry.h"
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseArray.h>
#include <chrono>
#include <iostream>
#include <sys/time.h>
#include <ctime>
#define PI 3.14159265

using std::cout; using std::endl;
using std::chrono::duration_cast;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::chrono::system_clock;

class Agent {
	public:
	double x, y, alpha, speed, xOld, yOld;
	long int timesample, timesampleOld;

	Agent()
    {
        x = 0.0;
		y = 0.0;
        xOld = 0.0;
		yOld = 0.0;
		alpha = 0.0;
		speed = 0.0;
		timesample = 0.0;
		timesampleOld = 0.0;
    }

	Agent(double newPoseX, double newPoseY, double newPoseAlpha, double newPoseSpeed, double newPoseTime )
    {
        x = newPoseX;
		y = newPoseY;
        xOld = 0.0;
		yOld = 0.0;
		alpha = newPoseAlpha;
		speed = newPoseSpeed;
		timesample = newPoseTime;
		timesampleOld = 0.0;
    }

	void agentUpdate(double newPoseX, double newPoseY, double newPoseTime){
        xOld = x;
		yOld = y;
		timesampleOld = timesample;
		x = newPoseX;
		y = newPoseY;
		timesample = newPoseTime;
	}
};

class gazeboVaiables
{
public:
	gazeboVaiables();
    void actorsCallback(const gazebo_msgs::ModelStates::ConstPtr &msg);
	std::string topic_pub_actors, topic_sub_actors, frame_id;
	float initial_robotpose_x, initial_robotpose_y;
	std::vector<Agent> agents;


private:
	ros::NodeHandle node_, ns_;
	ros::Publisher people_pub;
	ros::Subscriber ActorSub, poseSub, mapSub;
};

gazeboVaiables::gazeboVaiables() : node_("~"),
					 initial_robotpose_x(22.334),
					 initial_robotpose_y(14.5822),
					 frame_id("map"),
					 topic_pub_actors("gazebo_actors"),
					 topic_sub_actors("/gazebo/model_states")
{
	/** Get parameters from the launch file */
	node_.param("initial_robotpose_x", initial_robotpose_x, initial_robotpose_x);
	node_.param("initial_robotpose_y", initial_robotpose_y, initial_robotpose_y);
	node_.param("frame_id", frame_id, frame_id);
	node_.param("topic_pub_actors", topic_pub_actors, topic_pub_actors);
	node_.param("topic_sub_actors", topic_sub_actors, topic_sub_actors);

	/** Define Subscriber */
	ActorSub = ns_.subscribe(topic_sub_actors, 1, &gazeboVaiables::actorsCallback, this);
	
	/** Define Publisher */
	people_pub = ns_.advertise<sensor_msgs::PointCloud2>(topic_pub_actors, 1, false);
}

void gazeboVaiables::actorsCallback(const gazebo_msgs::ModelStates::ConstPtr &msg){
	long int currentTime = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
	pcl::PointCloud<pcl::PointXYZ> PeopleCloud;
	for (int i = 0; i < msg->name.size(); i++) {
		if(!msg->name[i].find("actor")){
			int countCoincidences = 0;
			PeopleCloud.resize(PeopleCloud.size()+1);
			PeopleCloud.points[PeopleCloud.size()-1].x = msg->pose[i].position.x+initial_robotpose_x;
			PeopleCloud.points[PeopleCloud.size()-1].y = msg->pose[i].position.y+initial_robotpose_y;
			PeopleCloud.points[PeopleCloud.size()-1].z = 0; // Projection onto 2D

			Agent newAgent = Agent(PeopleCloud.points[PeopleCloud.size()-1].x, PeopleCloud.points[PeopleCloud.size()-1].y, 0, 0, currentTime);
			if (agents.size() == 0){
				agents.push_back(newAgent);
			} else {
				for (int j = 0; j < agents.size(); j++) {
					double distance = sqrt(pow(msg->pose[i].position.x+initial_robotpose_x-agents[j].x,2)+(pow(msg->pose[i].position.y+initial_robotpose_y-agents[j].y,2)));
					double timedif = (double)(currentTime-agents[0].timesample)/1000;
					double speedTmp = (double)distance/timedif;
					
					if(distance>0.02 && distance<0.055 && timedif>0.0) {
						agents[j].agentUpdate(msg->pose[i].position.x+initial_robotpose_x, msg->pose[i].position.y+initial_robotpose_y, currentTime);
						break;
					} else if(distance>=0.055){
						countCoincidences++;
					}
				}
			}

			if (countCoincidences==agents.size()){
				agents.push_back(newAgent);
			} 
		}
	}

	for (int j = 0; j < agents.size(); j++) {
		if (currentTime-agents[j].timesample>4000){
			agents.erase(agents.begin()+j);
		} else if (currentTime!=agents[j].timesample){
			//estimatePosition(j);
			std::cout << "Estimate\n";
		}
	}
	
	std::cout << "Agents: " << agents.size() << "\n";

	//Publish PeopleCloud
	sensor_msgs::PointCloud2 PeopleCloud_output;
	pcl::toROSMsg(PeopleCloud, PeopleCloud_output);
	PeopleCloud_output.header.frame_id = frame_id;
	people_pub.publish(PeopleCloud_output);	
}


/** Main function*/
int main(int argc, char **argv)
{
	ros::init(argc, argv, "gazebo_variables");
	gazeboVaiables gazebovariables;
	ros::spin();
	return 0;
}


// std::cout << "Distance: " << distance  << " - Time: " << timedif <<  " - Speed: " << speedTmp << " - i: " << i << " - j: " << j << "\n";
// if(speedTmp<1.2){
// 	std::cout << "Update"<< "\n";
// } else {
// 	std::cout << "New"<< "\n";
// }
