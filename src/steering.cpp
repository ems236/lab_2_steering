#include <cstdlib>
#include <string>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"

using namespace std;

int main(int argc, char** argv)
{
    

    ros::init(argc, argv, "lab_2_steering");
    
    //Could do scoping here if I wanted to call the constructor for that.
    ros::NodeHandle node_handle;
    
    //Main loop
    while(ros::ok())
    {
    }
    
    return 0;   
}

string parse_desired_topic()
{
    string topic_name;
    int opt;
    while ((opt = getopt(argc, (argv), "n:")) != -1) 
    {
        switch (opt) 
        {
            case 'n':
                topic_name = optarg;
                break;
            default:
                printf("The -%c is not a recognized parameter\n", opt);
            break; 
        }
    }
    
    return topic_name;
}
