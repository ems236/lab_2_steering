#include <cstdlib>
#include <string>
#include <vector>
#include <math.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

using namespace std;
using geometry_msgs::Twist;
using std_msgs::Float64;
using std_msgs::Float32;

const string WRITE_VELOCITY_TOPIC = "cmd_vel";
const string READ_LASER_TOPIC = "laser_1";

//The x-axis is always forward
//Only turning angle is z axis
//Don't need to mess with a twist
double current_velocity;
double current_angular_velocity;

double desired_velocity;
double desired_angular_velocity;

vector<float> current_ranges;


string parse_desired_topic(int argc, char** argv)
{
    const string DEFAULT_TOPIC = "des_vel";
    string topic_name = DEFAULT_TOPIC;
    int opt;
    while ((opt = getopt(argc, (argv), "n:")) != -1) 
    {
        switch (opt) 
        {
            case 'n':
                topic_name = optarg;
                break;
            default:
                ROS_WARN("The -%c is not a recognized parameter\n", opt);
                break; 
        }
    }
    
    return topic_name;
}

void desired_velocity_callback(const Twist::ConstPtr& desired_velocity_twist)
{
    desired_velocity = desired_velocity_twist->linear.x;
    desired_angular_velocity = desired_velocity_twist->angular.z;
}

void laser_callback(const sensor_msgs::LaserScan::ConstPtr& laser_scan)
{
    double size_double = (laser_scan->angle_max - laser_scan->angle_min) / laser_scan->angle_increment;
    int size = round(size_double);
    
    current_ranges.clear();
    current_ranges.insert(current_ranges.end(), &(laser_scan->ranges)[0], &(laser_scan->ranges)[size]);
}

Twist twist_for(double linear, double angular)
{
    Twist new_twist;
    new_twist.linear.x = linear;
    new_twist.angular.z = angular;
    
    return new_twist;
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "lab_2_steering");
    
    string read_velocity_topic = parse_desired_topic(argc, argv);
    ROS_INFO("Reading velocity from topic %s\n", read_velocity_topic);
    ROS_INFO("Writing velocity to topic %s\n", WRITE_VELOCITY_TOPIC);
    
    //Let launch file handle remapping
    //Would be cool to print the remapping info
    ros::NodeHandle node_handle;
    
    ros::Publisher velocity_publisher = node_handle.advertise<Twist>(WRITE_VELOCITY_TOPIC, 200);
    ros::Subscriber velocity_subscriber = node_handle.subscribe(read_velocity_topic, 200, desired_velocity_callback);
    ros::Subscriber laser_subscriber = node_handle.subscribe(READ_LASER_TOPIC, 1000, laser_callback);
    
    //Main loop
    while(ros::ok())
    {
        
        //process all callbacks
        ros::spinOnce();
    }
    
    return 0;   
}

















