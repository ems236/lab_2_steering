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

#define PI 3.14519265

using namespace std;
using geometry_msgs::Twist;

const string WRITE_VELOCITY_TOPIC = "cmd_vel";
const string READ_LASER_TOPIC = "laser_1";

//I don't really know how to find this so it's hardcoded
//added a little padding
const double robot_radius = 0.2;

//The x-axis is always forward
//Only turning angle is z axis
//Don't need to mess with a twist
double current_velocity;
double current_angular_velocity;

double desired_velocity;
double desired_angular_velocity;

float start_angle;
float angle_step;
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

//records info and exits
void desired_velocity_callback(const Twist::ConstPtr& desired_velocity_twist)
{
    desired_velocity = desired_velocity_twist->linear.x;
    desired_angular_velocity = desired_velocity_twist->angular.z;
}

//records info and exits
void laser_callback(const sensor_msgs::LaserScan::ConstPtr& laser_scan)
{
    //Hopefully this doesn't get any dumb rounding errors
    //Feels like the message should just include a length?
    double size_double = (laser_scan->angle_max - laser_scan->angle_min) / laser_scan->angle_increment;
    int size = round(size_double);
    
    current_ranges.clear();
    current_ranges.insert(current_ranges.end(), &(laser_scan->ranges)[0], &(laser_scan->ranges)[size]);
}

int sign(double value)
{
    return value > 0 ? 1 : -1;
}

//Checks for any objects in the rectangle specifies
bool object_in_range(double min_y, double max_y, double min_x, double max_x)
{
    float current_angle = start_angle;
    for(vector<float>::iterator current = current_ranges.begin(); current != current_ranges.end(); ++current)
    {
        //forward is 0
        //for a raycast at theta. y = rsin theta, x = rcos theta
        if(!isinf(*current))
        {
            double y = *current * sin(current_angle);
            double x = *current * cos(current_angle);
            
            if((min_y <= y && y <= max_y) && (min_x <= x && max_x <= x))
            {
                return true;
            }
        }
       
        current_angle += angle_step;
    }
    
    return false;
}

bool can_move_direction(double linear, double angular)
{
    if(linear == 0)
    {
        return true;
    }
    //motion described by circle radius r = linear * (2pi / angular) / 2pi = linear / angular
    //angle over  =  (angular * time))
    // y change = r - rcos theta
    // x change = r - rsin theta

    //Calculate based on 1 second, which might be a little conservative.
    double time = 1.0;
    
    if(angular == 0)
    {
        return !object_in_range(-1 * robot_radius, robot_radius, -1 * robot_radius, time * linear);
    }
    
    double r = linear / angular;
    double theta = fmod(angular * time, 2 * PI);
    if(theta < 0)
    {
        theta = (2 * PI) + theta;
    }
    
    bool full_circle = fabs(angular * time) >= (2 * PI);
    
    double min_x, min_y = -1 * robot_radius;
    double max_x, max_y = robot_radius;
    double towards_y;
    
    if(theta < PI / 2)
    {
        max_x += r - (r * cos(theta));
        towards_y = r - (r * sin(theta));
    }
    
    else if(theta > (3 * PI / 2) || full_circle)
    {
        min_x -= r;
        max_x += r;
        towards_y = 2 * r;
    }
    else if(theta > PI)
    {
        max_x += r;
        towards_y = 2 * r;
    }
    else if(theta > (PI / 2))
    {
        max_x += r;
    }
    
    
    //Detetmine which y matters
    if(sign(angular) == 1)
    {
        max_y += towards_y;
    }
    else
    {
        min_y -= towards_y;
    }
    
    return !object_in_range(min_y, max_y, min_x, max_x);
}

bool can_move_desired()
{
    return can_move_direction(desired_velocity, desired_angular_velocity);
}

bool can_move_current()
{
    return can_move_direction(current_velocity, current_angular_velocity);
}

bool is_current_desired()
{
    return current_velocity == desired_velocity && desired_angular_velocity == current_angular_velocity;
}

bool can_move_straight(double speed)
{
    return can_move_direction(speed, 0);
}

Twist twist_for(double linear, double angular)
{
    Twist new_twist;
    new_twist.linear.x = linear;
    new_twist.angular.z = angular;
    
    return new_twist;
}

Twist avoid_obstacle_velocity()
{
    const double LOW_SPEED_THRESHOLD = 0.05;
    /*algorithm:
    while(can't move)
    {
        set angular pi / 4
        retest
        set angular -pi / 4
        retest
        
        half speed
    }
    */
    double new_speed = current_velocity;
    double new_angle = current_angular_velocity;
    bool found_new_velocity = false;
    while(!found_new_velocity && new_speed > LOW_SPEED_THRESHOLD)
    {
        if(can_move_direction(new_speed, new_angle + (PI / 4)))
        {
            return twist_for(new_speed, new_angle + (PI / 4));
        }
        
        if(can_move_direction(new_speed, new_angle - (PI / 4)))
        {
            return twist_for(new_speed, new_angle + (PI / 4));
        }
        
        new_speed /= 2;
    }
    
    return twist_for(0, PI / 2);
}

void steer()
{
    const int SPEED_STEP = 0.05;
    if(can_move_desired())
    {
        if(!is_current_desired())
        {
            //publish new twist; 
        }
        
        return;
    }
    
    double new_speed = current_velocity + SPEED_STEP;
    //prioritize not spinning in place
    if(can_move_straight(current_velocity) && can_move_straight(new_speed))
    {
        //update speed
    }
    
    if(can_move_current())
    {
       //try and speed up because that's fun
       if(can_move_direction(new_speed, current_angular_velocity))
       {
            //update speed;
       }
       return; 
    }
    
    //make up a new velocity
    Twist new_dir = avoid_obstacle_velocity();
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

















