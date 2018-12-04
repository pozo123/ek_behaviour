#include <ros/ros.h>
#include "std_msgs/String.h"
#include <geometry_msgs/Pose2D.h>
#include <stdlib.h>
#include <iostream>
#include <math.h>
#include <stdio.h>
#include "gazebo_msgs/LinkStates.h"

geometry_msgs::Pose2D ball;
//rate_hz assignment
double rate_hz = 30;

int main(int argc, char **argv){
    ros::init(argc,argv,"ball_simulation_node");
    ros::NodeHandle nh;
    ROS_INFO_STREAM("ball_simulation_node initialized");

    ros::Publisher pub_ball = nh.advertise<geometry_msgs::Pose2D>("/ball", rate_hz);

    //define the rate
    ros::Rate rate(rate_hz);

    while (ros::ok())
    {

        //lider es el mas cercano a la bola
        for(int i = 0; i < 45; i++){
            double j = i/10;
            ball.x = 4.5 - j;
            //publish the ball pos
            std::cout << "\nBall "<< i << ":\n" <<ball<< std::endl;
            pub_ball.publish(ball);
            ros::Duration(0.5).sleep();
        }
        
        ros::spinOnce();
        rate.sleep();    
    }
    return 0;
}