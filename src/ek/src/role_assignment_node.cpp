#include <ros/ros.h>
#include "std_msgs/String.h"
#include <geometry_msgs/Pose2D.h>
#include <stdlib.h>
#include <iostream>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/QR>
#include <Eigen/LU>
#include <math.h>
#include <stdio.h>
#include "gazebo_msgs/LinkStates.h"

#include <stdio.h>
#include <stdlib.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

#include "std_msgs/Int32MultiArray.h"

#include <algorithm>
#include <iostream>
#include <vector>
#include <cmath>

using namespace Eigen;

#define NOROBOTS 5
//int NOROBOTS;

std::vector<geometry_msgs::Pose2D> coordenadas_robots(NOROBOTS);
geometry_msgs::Pose2D ball;
//rate_hz assignment
double rate_hz = 30;
double altura_cancha = 6;//768;
double ancho_cancha = 9;//1024;

//Callbacks para las posiciones
void updateBall(const geometry_msgs::Pose2D::ConstPtr& msg)
{
    ball.x=msg->x;
    ball.y=msg->y;
}

void updateRobot(const geometry_msgs::Pose2D::ConstPtr& msg, const int& num_bot)
{
    coordenadas_robots[num_bot].x=msg->x;
    coordenadas_robots[num_bot].y=msg->y;
    coordenadas_robots[num_bot].theta=msg->theta;
}


int main(int argc, char **argv){
    ros::init(argc,argv,"roles_assignment_node");
    ros::NodeHandle nh;
    ROS_INFO_STREAM("roles_assignment_node initialized");
    //ros::NodeHandle nh_priv("~");

    //nh_priv.param<int> ("no_robots", NOROBOTS, 5);

    ros::Publisher pub_roles = nh.advertise<std_msgs::Int32MultiArray>("/roles", rate_hz);

    //Topics to acquire robot and ball position.
    ros::Subscriber sub_ball_pos = nh.subscribe("/ball", 1, updateBall);
    std::vector<ros::Subscriber> sub_positions;
    for(int i = 0; i < (NOROBOTS); i++){
        std::stringstream sub_pos;
        sub_pos << "/EK" << (i+1) << "/pos";
        sub_positions.push_back(
        nh.subscribe<geometry_msgs::Pose2D>(sub_pos.str(), 1,
        boost::bind(updateRobot, _1, i)));
    }

    //define the rate
    ros::Rate rate(rate_hz);

    while (ros::ok())
    {
        std_msgs::Int32MultiArray roles;

    //FALTA Meter a los robots en una matriz de la forma r=[r1x r1y r2x r2y r3x r3y r4x r4y r5x r5y];
        double inf=100000;
        double inf2=1000000; //inf2>inf1
        double dist;
        double minL=inf;
        double max=0;
        double min=inf2;
        VectorXf distancias(4);

        int bot_rol_lider;
        int bot_rol_1;
        int bot_rol_2;
        int bot_rol_3;
        int bot_rol_4;

        geometry_msgs::Pose2D punto1;
        geometry_msgs::Pose2D punto2;
        geometry_msgs::Pose2D punto3;
        geometry_msgs::Pose2D punto4;

        punto1.x=3*ancho_cancha/8;
        punto1.y=altura_cancha/4;
        punto2.x=3*ancho_cancha/8;
        punto2.y=-altura_cancha/4;
        punto3.x=ancho_cancha/8;
        punto3.y=altura_cancha/4;
        punto4.x=ancho_cancha/8;
        punto4.y=-altura_cancha/4;

        //lider es el mas cercano a la bola
        for(int i1 = 0; i1 < (NOROBOTS); i1++){
            dist=sqrt(pow(ball.x-coordenadas_robots[i1].x,2)+pow(ball.y-coordenadas_robots[i1].y,2));
            if(dist<minL){
                minL=dist;
                bot_rol_lider=i1;
            }
        }

        //eliminar el lider del vector, para no considerarlo en la asignacion de otros roles
        coordenadas_robots.erase(coordenadas_robots.begin()+bot_rol_lider);
        //if(NOROBOTS == 5){
            for(int i2 = 0; i2 < (NOROBOTS - 1); i2++){
                for(int i3 = 0; i3 < (NOROBOTS - 1); i3++){
                    if(i2!=i3){
                        for(int i4 = 0; i4 < (NOROBOTS - 1); i4++){
                            if(i4!=i2 && i4!= i3){
                                for(int i5 = 0; i5 < (NOROBOTS - 1); i5++){
                                    if(i5!=i2 && i5!=i3 && i5!=i4){
                                        distancias << sqrt(pow(punto1.x-coordenadas_robots[i2].x,2)+pow(punto1.y-coordenadas_robots[i2].y,2)),
                                                      sqrt(pow(punto2.x-coordenadas_robots[i3].x,2)+pow(punto2.y-coordenadas_robots[i3].y,2)),
                                                      sqrt(pow(punto3.x-coordenadas_robots[i4].x,2)+pow(punto3.y-coordenadas_robots[i4].y,2)),
                                                      sqrt(pow(punto4.x-coordenadas_robots[i5].x,2)+pow(punto4.y-coordenadas_robots[i5].y,2));
                                        max=distancias.maxCoeff();
                                        if(max<min){
                                            min=max;
                                            bot_rol_1=i2;
                                            bot_rol_2=i3;
                                            bot_rol_3=i4;
                                            bot_rol_4=i5;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }

            //Introduces Lider
            if(bot_rol_1 >= bot_rol_lider)
                bot_rol_1++;
            if(bot_rol_2 >= bot_rol_lider)
                bot_rol_2++;
            if(bot_rol_3 >= bot_rol_lider)
                bot_rol_3++;
            if(bot_rol_4 >= bot_rol_lider)
                bot_rol_4++;
        /*} else {
            bot_rol_lider = 0;
            bot_rol_1 = 1;
            bot_rol_2 = 2;
            bot_rol_3 = 3;
            bot_rol_4 = 4;
        }*/
        roles.data.push_back(bot_rol_lider);
        roles.data.push_back(bot_rol_1);
        roles.data.push_back(bot_rol_2);
        roles.data.push_back(bot_rol_3);
        roles.data.push_back(bot_rol_4);

        //publish the roles vector
        std::cout << "\nRoles: "<<roles<< std::endl;
        pub_roles.publish(roles);
        
        ros::spinOnce();
        rate.sleep();    
    }
    return 0;
}