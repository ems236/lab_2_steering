#include <cstdlib>
#include <string>
#include <vector>
#include <math.h>
#include <sstream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

#define PI 3.14519265

using namespace std;
using geometry_msgs::Twist;

const string WRITE_VELOCITY_TOPIC = "/robot1/cmd_vel";
const string READ_LASER_TOPIC = "/robot1/laser_1";

//I don't really know how to find this so it's hardcoded
//added a little padding
const double robot_radius = 0.2;
const double unpadded_radius = 0.15;

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
    
    ROS_INFO("Parsed %s as the desired velocity topic", topic_name.c_str());
    
    return topic_name;
}

//records info and exits
void desired_velocity_callback(const Twist::ConstPtr& desired_velocity_twist)
{
    desired_velocity = desired_velocity_twist->linear.x;
    desired_angular_velocity = desired_velocity_twist->angular.z;
    
    //ROS_INFO("Receiving des vel info %f %f", desired_velocity, desired_angular_velocity);
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
    
    start_angle = laser_scan->angle_min;
    angle_step = laser_scan->angle_increment;
    
    //ROS_INFO("Receiving laser info");
    /*float current_angle = start_angle;
    for(vector<float>::iterator current = current_ranges.begin(); current != current_ranges.end(); ++current)
    {
        ROS_INFO("Laser info: angle: %f distance: %f", current_angle, *current);
       
        current_angle += angle_step;
    }*/
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
            
            if(abs(current_angle) < 0.2)
            {
                //ROS_INFO("Checking for objects in box y[%f, %f], x[%f, %f]", min_y, max_y, min_x, max_x);
            }
            
            
            if((min_y <= y && y <= max_y) && (min_x <= x && x <= max_x))
            {
                ROS_INFO("Checking for objects in box y[%f, %f], x[%f, %f]", min_y, max_y, min_x, max_x);
                ROS_INFO("stopping for object at angle %f r %f. In x,y thats %f %f", current_angle, *current, x, y);
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
    // y change = r - rsin theta
    // x change = r - rcos theta

    //might be a little conservative.
    double time = 0.75;
    
    if(angular == 0)
    {
        return !object_in_range(-1 * unpadded_radius, unpadded_radius, -1 * unpadded_radius, unpadded_radius + time * linear);
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
        max_x += r - (r * sin(theta));
        towards_y = r - (r * cos(theta));
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
        towards_y = r - (r * cos(theta));
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
    //This isn't working. Change to every time you see a wall it stops and turns until it doesn't see a wall.
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
    
    ROS_INFO("Turning to avoid an obstacle");
    
    while(new_speed > LOW_SPEED_THRESHOLD)
    {
        if(can_move_direction(new_speed, new_angle + (PI / 4)))
        {
            ROS_INFO("Turning left to avoid an obstacle");
            return twist_for(new_speed, new_angle + (PI / 4));
        }
        
        if(can_move_direction(new_speed, new_angle - (PI / 4)))
        {
            ROS_INFO("Turning right to avoid an obstacle");
            return twist_for(new_speed, new_angle + (PI / 4));
        }
        
        ROS_INFO("Halving speed");
        new_speed /= 2;
    }
    
    
    ROS_INFO("Turning left and stopping motion to avoid an obstacle");
    return twist_for(0, PI / 2);
}

void publish_new_velocity(Twist new_vel, ros::Publisher* velocity_publisher)
{
    current_velocity = new_vel.linear.x;
    current_angular_velocity = new_vel.angular.z;
    ROS_INFO("Setting velocity to lin: %f ang: %f", new_vel.linear.x, new_vel.angular.z);
    velocity_publisher->publish(new_vel);
}

void steer(ros::Publisher* velocity_publisher)
{
    const double SPEED_STEP = 0.05;
    if(can_move_desired())
    {
        if(!is_current_desired())
        {
            //publish new twist;
            ROS_INFO("Setting velocity to current desired");
            publish_new_velocity(twist_for(desired_velocity, desired_angular_velocity), velocity_publisher);
        }
        
        return;
    }
    
    double new_speed = current_velocity + SPEED_STEP;
    //prioritize not spinning in place
   
    
    /*
    if(can_move_current())
    {
        ROS_INFO("Can move in current direction");
        
        
        //try and speed up because that's fun
        if(can_move_direction(new_speed, current_angular_velocity))
        {
            ROS_INFO("Increasing speed to %f", new_speed);
            //update speed;
            publish_new_velocity(twist_for(new_speed, current_angular_velocity), velocity_publisher);
            return;
        }
       
        
        
        return; 
    }
    */
    if(can_move_straight(current_velocity) && can_move_straight(new_speed))
    {
        ROS_INFO("Moving in a straight line at speed %f", new_speed);
        //update speed
        publish_new_velocity(twist_for(new_speed, 0), velocity_publisher);
        return;
    }
    
    
    //make up a new velocity
    ROS_INFO("Avoiding an obstacle");
    Twist new_dir = avoid_obstacle_velocity();
    publish_new_velocity(new_dir, velocity_publisher);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lab_2_steering");
    
    string read_velocity_topic = parse_desired_topic(argc, argv);
    ROS_INFO("Reading velocity from topic %s\n", read_velocity_topic.c_str());
    ROS_INFO("Writing velocity to topic %s\n", WRITE_VELOCITY_TOPIC.c_str());
    
    //Let launch file handle remapping
    //Would be cool to print the remapping info
    ros::NodeHandle node_handle;
    
    ros::Publisher velocity_publisher = node_handle.advertise<Twist>(WRITE_VELOCITY_TOPIC, 200);
    ros::Subscriber velocity_subscriber = node_handle.subscribe(read_velocity_topic, 200, desired_velocity_callback);
    ros::Subscriber laser_subscriber = node_handle.subscribe(READ_LASER_TOPIC, 1000, laser_callback);
    
    ros::Rate loop_rate(10);
    //Main loop
    while(ros::ok())
    {
        steer(&velocity_publisher);
        
        //process all callbacks
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;   
}

















