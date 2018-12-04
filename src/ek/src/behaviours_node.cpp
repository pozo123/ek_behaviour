#include <ros/ros.h>
#include "std_msgs/String.h"
#include <geometry_msgs/Pose2D.h>
#include <stdlib.h>
#include <iostream>
#include <math.h>
#include <stdio.h>
#include <turtlesim/Pose.h>
#include <signal.h>

#include <Eigen/Geometry>//Así?
#include <Eigen/Dense>//Así?
#include <Eigen/QR>//Así?
#include <Eigen/LU>//Así?

using namespace Eigen;

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"

#define PI 3.1415926535897932384626
#define NOROBOTS 5
//int NOROBOTS;
geometry_msgs::Pose2D target_position;
geometry_msgs::Pose2D ball;
double rate_hz = 50;
double altura_cancha = 6;//768;
double ancho_cancha = 9;//1024;

float field_ratio_x = ancho_cancha / 100;
float field_ratio_y = altura_cancha / (3700/52);

//coeficientes para funciones de formacion
float cte_x_p1_1=0.6;//alpha
float cte_x_p1_2=1.5;//m

float cte_y_p1_1=0.35;//d
float cte_y_p1_2=2.4;//n
float cte_y_p1_3=1.5;//1.5

float cte_x_p2_1=0.6;//beta

float cte_y_p2_1=1.5;//g
float cte_y_p2_2=0.25;//z
float cte_y_p2_3=-0.3;//k

float p1x;
float p1y;
float p2x;
float p2y;
float liderx;

int roles_bots[NOROBOTS];
//std::vector<int> roles_bots;

bool msg_exists = false;
void get_roles(const std_msgs::Int32MultiArray::ConstPtr& msg){
    int i = 0;
    for(std::vector<int>::const_iterator it = msg->data.begin(); it != msg->data.end(); ++it)
    {
        roles_bots[i] = *it;
        ROS_INFO_STREAM("roles_bots["<<i<<"]: "<<roles_bots[i]);    
        i++;
    }
    msg_exists = true;
    return;
}

void getBallPose(const geometry_msgs::Pose2D::ConstPtr& msg)
{
    ball.x=msg->x;
    ball.y=msg->y;
    ball.theta= atan2(ball.y,ball.x);
    ROS_INFO_STREAM("Ball: \n  x: "<<ball.x<<"\n  y: "<<ball.y<<"\n  theta: "<<ball.theta);    
}

void restringir_valores(){

    if(target_position.x < 0)
        target_position.x = 0;
    if(target_position.x > ancho_cancha/2)
        target_position.x = ancho_cancha/2;
    if(target_position.y < -altura_cancha/2)
        target_position.y = -altura_cancha/2;
    if(target_position.y > altura_cancha/2)
        target_position.y = altura_cancha/2;

}

int main(int argc, char **argv){
    ros::init(argc,argv,"behaviours_node");
    ros::NodeHandle nh;
    ROS_INFO_STREAM("behaviours_node_initialized");    
    //ros::NodeHandle nh_priv("~");

    //nh_priv.param<int> ("no_robots", NOROBOTS, 5);

    ros::Subscriber sub_ball_pos = nh.subscribe("/ball", 1, &getBallPose);
    ros::Subscriber sub_roles = nh.subscribe("/roles", 1, &get_roles); 

    std::vector<ros::Publisher> pub_target_v;
    for(int i = 0; i < (NOROBOTS); i++){
        std::stringstream target_pub;
        target_pub << "/EK" << (i+1) << "/target";
        pub_target_v.push_back(nh.advertise<geometry_msgs::Pose2D>(target_pub.str(), rate_hz));
    }

    ros::Rate rate(rate_hz);

    //ros::spinOnce();
	while (ros::ok())
	{

        MatrixXf rob(2,5);
        MatrixXf mult(2,5);
        MatrixXf mat_rot(2,2);

        liderx=sqrt(pow(ball.x,2)+pow(ball.y,2));
        p1x=(cte_x_p1_1 * ball.x + cte_x_p1_2);//Distancia pareja 1 a la porteria
        p1y=(pow(cte_y_p1_1*(ball.x - cte_y_p1_2),2) + cte_y_p1_3);//Distancia vertical pareja 1
        p2x=(cte_x_p2_1 * ball.x);//Distancia pareja 2 a la porteria
        p2y=(pow(cte_y_p2_1,(cte_y_p2_2 * ball.x)) + cte_y_p2_3);//Distancia vertical pareja 2
        mat_rot << cos(ball.theta), -sin(ball.theta),
                   sin(ball.theta), cos(ball.theta);
        //Crear matriz con valores para las target para cada uno de los roles, seguir las 5 funciones ya establecidas en matlab
        rob << liderx,p1x,p1x,p2x,p2x,
               0,p1y,-p1y,p2y,-p2y;
               ROS_INFO_STREAM("targets: "<<rob);
        mult = mat_rot * rob;
        if(msg_exists){
            for(int i = 0; i < (NOROBOTS); i++){
                for(int j = 0; j < (NOROBOTS); j++){
                    if((roles_bots[j]) == i){
                        target_position.x=mult(0,j);
                        target_position.y=mult(1,j);
                        restringir_valores();
                        std::cout << "target_position " <<i<<": \n"<<target_position<< std::endl;
                        pub_target_v[i].publish(target_position);
                    }
                }
            }
        }
		
		ros::spinOnce();
		rate.sleep();
        
	}
    return 0;
}