#include <cstdlib>
#include <string>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

using namespace std;

const string WRITE_VELOCITY_TOPIC = "cmd_vel";
const string READ_LASER_TOPIC = "laser_1";

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

void desired_velocity_callback(const geometry_msgs::Twist::ConstPtr& desired_velocity)
{
}

void laser_callback(const sensor_msgs::LaserScan::ConstPtr& laser_scan)
{
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lab_2_steering");
    
    string read_velocity_topic = parse_desired_topic(argc, argv);
    ROS_INFO("Reading velocity from topic %s\n", read_velocity_topic);
    ROS_INFO("Writing velocity to topic %s\n", WRITE_VELOCITY_TOPIC);
    
    //Could do scoping here if I wanted to call the constructor for that.
    //Let launch file handle remapping
    ros::NodeHandle node_handle;
    
    ros::Publisher velocity_publisher = node_handle.advertise<geometry_msgs::Twist>(WRITE_VELOCITY_TOPIC, 200);
    ros::Subscriber velocity_subscriber = node_handle.subscribe(read_velocity_topic, 200, desired_velocity_callback);
    ros::Subscriber laser_subscriber = node_handle.subscribe(READ_LASER_TOPIC, 1000, laser_callback);
    
    //Main loop
    while(ros::ok())
    {
        int x = 1;
    }
    
    return 0;   
}
