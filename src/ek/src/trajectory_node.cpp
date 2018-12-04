//Generar tryectoria no lineal sino dependiendo de los obstaculos
//Ver por que solo lo hace una vez y no publica en com
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Int8.h>
#include <stdlib.h>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/QR>
#include <Eigen/LU>
#include <math.h>
#include <stdio.h>
#include <turtlesim/Pose.h>
#include <Eigen/Geometry>
#include <signal.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include "boost/bind.hpp"
#include <cmath>

using namespace Eigen;

#define CIRCLE_RADIUS 250

#define EPSILON_DISTANCE .08

#define robot_radius 0.1

#define NOROBOTS 5
//int NOROBOTS;

bool initialize = false;
//geometry_msgs::Pose2D robot_position;//Hacerlos 5? o un vector?
std::vector<geometry_msgs::Pose2D> robot_position(NOROBOTS);
//geometry_msgs::Pose2D target_position;//Hacerlos 5? o un vector?
std::vector<geometry_msgs::Pose2D> target_position(NOROBOTS);
//geometry_msgs::Pose2D ball;

double rate_hz = 50;
ros::Publisher pub_vel;
ros::Publisher pub_target;
double sum_error;
double cruise_speed = 20;

//TODO: although the topics use geometry_msgs::Pose2D, we should use vectors instead. The code will become much clearer
void get_robot_pose(const geometry_msgs::Pose2DConstPtr& msg, const int& num_bot) {
    robot_position[num_bot].x = msg -> x;
    robot_position[num_bot].y = msg -> y;
    robot_position[num_bot].theta = msg -> theta; 
}

void get_target_pose(const geometry_msgs::Pose2DConstPtr& msg, const int& num_bot) {
    target_position[num_bot].x = msg -> x;
    target_position[num_bot].y = msg -> y;
    target_position[num_bot].theta = msg -> theta; 
    //ROS_INFO_STREAM("Target position ===>" << " ( " << target_position.x << " , " << target_position.y << " , " << target_position.theta << " )");
}


bool isGoalFar(geometry_msgs::Pose2D p_start, geometry_msgs::Pose2D p_goal) {
    double epsilon_x, epsilon_y;
    epsilon_x = EPSILON_DISTANCE;
    epsilon_y = EPSILON_DISTANCE;
    double distance_x = abs(p_goal.x - p_start.x);
    double distance_y = abs(p_goal.y - p_start.y);
    if (distance_x > epsilon_x || distance_y > epsilon_y)
        return true;    
    else
        return false;
}

geometry_msgs::Pose2D generateConstantVelocity(double constant_speed, geometry_msgs::Pose2D p_start, geometry_msgs::Pose2D p_goal){
    
    // Compute direction to goal
    Vector2d p_start_vector(p_start.x, p_start.y);//, p_start.theta);
    Vector2d p_goal_vector(p_goal.x, p_goal.y);// p_goal.theta);
    Vector2d goal_direction_vector = p_goal_vector - p_start_vector;
    
    // Compute speed in the direction to goal
    Vector2d velocity_vector = constant_speed * (goal_direction_vector / goal_direction_vector.norm());
 
   	geometry_msgs::Pose2D velocity;

    velocity.x = velocity_vector.x();
    velocity.y = velocity_vector.y();

    double z1, z2, z_final;//, start, goal;
    // if(p_start.theta < 0)
    //     start = 2 * M_PI - p_start.theta;
    // else
    //     start = p_start.theta;

    double start = p_start.theta + M_PI ; //Por que + pi?
    double goal = p_goal.theta;

    z1 = fabs( goal - start);//start - goal) ;
    z2 = 2 * M_PI - z1 ;
    
    if( z1 < z2 )
    {
    	z_final = z1;
    } else {
    	z_final = -z2;
    }

    //double error = goal - start;
    //double kp = 1.25;//Revisar constantes?
    //double ki = 0.5;//Revisar constantes?
    //sum_error += error / rate_hz; //No habria que regresarlo a 0 en algun punto?
    //z_final = kp * error + sum_error * ki;
    velocity.theta = z_final /  100 ;
    ROS_INFO_STREAM("velocity: " << velocity);
    return velocity;
}

//p1 es el de referencia y p2 es el otro (regresa p2 con respecto a p1)
float angleBetweenPoints(float p1x, float p1y, float p2x, float p2y){
    float angle = atan2(p2y - p1y,p2x - p1x);//de -pi a pi
    return angle;
}

geometry_msgs::Pose2D avoidObstacles(geometry_msgs::Pose2D constant_vel, int rob_no){
    //Tiene que ser recursivo, si choca con alguno recalculas pero hay que revisar si ese no choca
    float theta;
    geometry_msgs::Pose2D p_alpha;//AGUAS! si p_alpha<180 y p_beta>180 p_beta es negativo y menor que p_alpha
    geometry_msgs::Pose2D p_beta;//AGUAS! si p_alpha<180 y p_beta>180 p_beta es negativo y menor que p_alpha
    geometry_msgs::Pose2D rob_b;
    geometry_msgs::Pose2D result;
    result.x = constant_vel.x;
    result.y = constant_vel.y;
    float desired_angle = atan2(constant_vel.y, constant_vel.x);
    for(int i = 0; i < NOROBOTS; i++){
        if(i != rob_no){
            rob_b = robot_position[i];
            theta = angleBetweenPoints(robot_position[rob_no].x,robot_position[rob_no].y,rob_b.x,rob_b.y);
            float comp_x = 2 * robot_radius * cos(theta - M_PI/2);
            float comp_y = 2 * robot_radius * sin(theta - M_PI/2);
            p_alpha.x = rob_b.x + comp_x;
            p_alpha.y = rob_b.y + comp_y;
            p_beta.x = rob_b.x - comp_x;
            p_beta.y = rob_b.y - comp_y;
            float angulo_alpha = angleBetweenPoints(robot_position[rob_no].x,robot_position[rob_no].y,p_alpha.x,p_alpha.y);
            float angulo_beta = angleBetweenPoints(robot_position[rob_no].x,robot_position[rob_no].y,p_beta.x,p_beta.y);
            //desired angle cae en el ROV
            if(angulo_alpha < desired_angle && desired_angle < angulo_beta){
                if(theta < desired_angle){
                    result.theta = angulo_beta;
                } else {
                    result.theta = angulo_alpha;
                }
                return avoidObstacles(result, rob_no);
            } else {
                //Revisar para caso en el que 180 esta en ROV therefore el angulo max es negativo y el otro positivo
                if(angulo_alpha > 0 && angulo_beta < 0 && (desired_angle < angulo_beta || desired_angle > angulo_alpha)){
                    if(theta < 0){
                        if(desired_angle < angulo_beta && desired_angle > theta){
                            result.theta = angulo_beta;
                        } else {
                            result.theta = angulo_alpha;
                        }
                    } else {
                        if(desired_angle > angulo_alpha && desired_angle < theta){
                            result.theta = angulo_alpha;
                        } else {
                            result.theta = angulo_beta;
                        }
                    }
                    return avoidObstacles(result, rob_no);
                } else {
                    return constant_vel;
                }
            }
        }
    }
}

geometry_msgs::Pose2D rotateVelocity(geometry_msgs::Pose2D velocity, double rotation_angle)
{
    // Compute direction to goal
    Vector3d original_vector(velocity.x, velocity.y, velocity.theta);
    

   	geometry_msgs::Pose2D velocity_new;

    velocity_new.x = original_vector.x() * cos(rotation_angle) - original_vector.y() * sin(rotation_angle);
    velocity_new.y = original_vector.x() * sin(rotation_angle) + original_vector.y() * cos(rotation_angle);
    velocity_new.theta = original_vector.z();

    return velocity_new;
}

// Keep velocity under the allowed robot limits
// geometry_msgs::Pose2D boundVelocity(geometry_msgs::Pose2D velocity)
// {
//     // Higher speed bounds
//     double max_linear_speed = cruise_speed;
//     double min_linear_speed = 0;
//     double max_angular_speed = 200;
//     double min_angular_speed = 0;
    
//     if (velocity.x > max_linear_speed)
//         velocity.x = max_linear_speed;
//     else if (velocity.x < -max_linear_speed)
//         velocity.x = -max_linear_speed;
//     if (velocity.y > max_linear_speed)
//         velocity.y = max_linear_speed;
//     else if (velocity.y < -max_linear_speed)
//         velocity.y = -max_linear_speed;
//     if (velocity.theta > max_angular_speed)
//         velocity.theta = max_angular_speed;
//     else if (velocity.theta < -max_angular_speed)
//         velocity.theta = -max_angular_speed;
    
//     // Lower speed bounds
//     if (velocity.x > 0 && velocity.x < min_linear_speed)
//         velocity.x = min_linear_speed;
//     else if (velocity.x < 0 && velocity.x > -min_linear_speed)
//         velocity.x = -min_linear_speed;
//     if (velocity.y > 0 && velocity.y < min_linear_speed)
//         velocity.y = min_linear_speed;
//     else if (velocity.y < 0 && velocity.y > -min_linear_speed)
//         velocity.y = -min_linear_speed;
//     if (velocity.theta >0 && velocity.theta < min_angular_speed)
//         velocity.theta = min_angular_speed;
//     else if (velocity.theta < 0 && velocity.theta > - min_angular_speed )
//         velocity.theta = -min_angular_speed;

//     return velocity;
    
// }

//Convierte las velocida calculada a los mensajes que se van a enviar
std_msgs::String composeVelocityMessage(geometry_msgs::Pose2D mensaje, int8_t action)
{
    char soh  = 1, etb = 23;
   // ROS_INFO_STREAM("Message to send antes mult "<<mensaje.linear.x<<" "<<mensaje.linear.y);
    double cte = 1.5;
    //ROS_INFO_STREAM("Message to send in parts: "<<mensaje.linear.x);
    //Construccion del mensaje X
    char msg_x[4];
    int mens_x = abs(mensaje.x);
    int un_x = mens_x % 10;
    int dec_x = (mens_x/10)%10;
    int cent_x = (mens_x/100)%10;
    
    if (mensaje.x>=0)
        sprintf(msg_x,"+%d%d%d",cent_x,dec_x,un_x);
    else
        sprintf(msg_x,"-%d%d%d",cent_x,dec_x,un_x);
    
    //Construccion del mensaje Y
    char msg_y[4];
    int mens_y = abs(mensaje.y);
    int un_y = mens_y % 10;
    int dec_y = (mens_y/10)%10;
    int cent_y = (mens_y/100)%10;
    if (mensaje.y>=0)
        sprintf(msg_y,"+%d%d%d",cent_y,dec_y,un_y);
    else
        sprintf(msg_y,"-%d%d%d",cent_y,dec_y,un_y);
    
    //Construccion del mensaje Z
    char msg_z[4];
    int mens_z = abs(mensaje.theta);
    int un_z = mens_z % 10;
    int dec_z = (mens_z/10)%10;
    int cent_z = (mens_z/100)%10;
    if (mensaje.theta>=0)
        sprintf(msg_z,"+%d%d%d",cent_z,dec_z,un_z);
    else
        sprintf(msg_z,"-%d%d%d",cent_z,dec_z,un_z);
    char message_to_send[24];
    sprintf(message_to_send, "%c%s%s%s%c%c", soh,msg_x,msg_y,msg_z,action,etb);

    //sprintf(message_to_send, "%c-200+000+000E%c", soh,etb);
    std_msgs::String msg;
    msg.data = message_to_send;
    char message_to_print[21];
    sprintf(message_to_print, "%s%s%s",msg_x,msg_y,msg_z);
   // ROS_INFO_STREAM("Message to send:"<<message_to_print);
    return msg;
}

void mySigintHandler(int sig)
{
  // Do some custom action.
	geometry_msgs::Pose2D last_msg;
  // For example, publish a stop message to some other nodes.
  	last_msg.x = 0;
	last_msg.y = 0;
	last_msg.theta = 0;
	std_msgs::String velocity_msg = composeVelocityMessage(last_msg, 64);
	ROS_INFO_STREAM("Sending last message...");
	pub_vel.publish(velocity_msg);
  // All the default sigint handler does is call shutdown()
  	ros::shutdown();
}


bool is_orientation_not_correct(geometry_msgs::Pose2D p_start, geometry_msgs::Pose2D p_goal) 
{
    double epsilon_z;//, epsilon_y;
    epsilon_z = 0.1;
    double distance_z = abs(p_goal.theta - p_start.theta);
    // double distance_y = abs(p_goal.y - p_start.y);
    if (distance_z > epsilon_z )
        return true;
    else
        return false;
}

int main(int argc, char **argv){
    ros::init(argc,argv,"trajectory_node");
    ros::NodeHandle nh;
    //ros::NodeHandle nh_priv("~");
    ROS_INFO_STREAM("robot_trajectory_node initialized");
	ROS_INFO_STREAM(ros::this_node::getName());

    //nh_priv.param<int> ("no_robots", NOROBOTS, 5);
//   for(int i = 0; i < (NOROBOTS - 1); i++){
//        std::stringstream yss, bss;
//        yss << "y_r" << i;
//        bss << "b_r" << i;
//        vision_pub_yr.push_back(nh.advertise<geometry_msgs::Pose2D>(yss.str(), RATE_HZ));
//        vision_pub_br.push_back(nh.advertise<geometry_msgs::Pose2D>(bss.str(), RATE_HZ));
//}
//// Blue robot info:
//                int robots_blue_n =  detection.robots_blue_size();
//                for (int i = 0; i < robots_blue_n; i++) {
//                    SSL_DetectionRobot robot = detection.robots_blue(i);
//                    ROS_DEBUG_STREAM("-Robot(B) (" << (i+1) << "/" << robots_blue_n << "): ");
//                    if (robot.has_robot_id() ) {
//                        ROS_DEBUG_STREAM("ID=" << robot.robot_id());
//                        ROS_DEBUG_STREAM(" POS<" << robot.x() << "," << robot.y() << "> ");
//                        geometry_msgs::Pose2D msg;
//                        msg.x = robot.x();
//                        msg.y = robot.y();
//                        if (robot.has_orientation()) {
//                            ROS_DEBUG_STREAM("ANGLE=" << robot.orientation() << " ");
//                            msg.theta = robot.orientation();
//                        }
//                        vision_pub_br[robot.robot_id()].publish(msg);
//                    }
//}
    //pub_vel = nh.advertise<std_msgs::String>(vel_topic_name, rate_hz);
    std::vector<ros::Publisher> velocity_pub;
    for(int i = 0; i < NOROBOTS; i++){ 
        std::stringstream pub_vel;
        pub_vel << "/EK" << (i+1) << "/com_101";
        velocity_pub.push_back(nh.advertise<geometry_msgs::Pose2D>(pub_vel.str(), rate_hz));
    }

	//Topics to acquire robot and ball position (from the vision node) //queue_size = 1 to deal only with the most recent positions
    //ros::Subscriber sub_attacker_pos = nh.subscribe(argv[1], 1, &get_robot_pose); 
    std::vector<ros::Subscriber> sub_positions;
    for(int i = 0; i < NOROBOTS; i++){ 
        std::stringstream sub_pos;
        sub_pos << "/EK" << (i+1) << "/pos";
        ROS_INFO_STREAM("Subscribed to " + sub_pos.str());
        sub_positions.push_back(
        nh.subscribe<geometry_msgs::Pose2D>(sub_pos.str(), 1, 
        boost::bind(get_robot_pose, _1, i)));
    }
	//ros::Subscriber sub = nh.subscribe<PointCloud>(target_topic, 1, &get_next_pose);
    std::vector<ros::Subscriber> sub_targets;
    for(int i = 0; i < NOROBOTS; i++){ 
        std::stringstream sub_tar;
        sub_tar << "/EK" << (i+1) << "/target";
        ROS_INFO_STREAM("Subscribed to " + sub_tar.str());
        sub_targets.push_back(
        nh.subscribe<geometry_msgs::Pose2D>(sub_tar.str(), 1, 
        boost::bind(get_target_pose, _1, i)));
        //Hay que jalarlo de /gazebo/model_states pero no se como sacar info en particular de ahi
        //OOOOO aprender a publicar desde el plugin a un topico en particular
    }

    pub_target = nh.advertise<geometry_msgs::Pose2D>("/target_topic_debug", rate_hz);

 	signal(SIGINT, mySigintHandler);
    //Mensaje que publicara las trayectorias
    geometry_msgs::Pose2D desired_velocity;
    
    ros::Rate rate(rate_hz);
    
    ROS_INFO_STREAM("Starting");
	while (ros::ok())
    {
        
        for(int i = 0; i < NOROBOTS; i++){
            //pub_target.publish(target_position[i]);
            ROS_INFO_STREAM(i);
            if (isGoalFar(robot_position[i], target_position[i])) {

                desired_velocity = generateConstantVelocity(cruise_speed, robot_position[i], target_position[i]);
                desired_velocity = avoidObstacles(desired_velocity,i);
            } else { // do not move
                if( is_orientation_not_correct(robot_position[i], target_position[i]))
                {
                    desired_velocity = generateConstantVelocity(cruise_speed, robot_position[i], target_position[i]);
                    desired_velocity.x = 0;
                    desired_velocity.y = 0;
                } else {
                    desired_velocity.x = 0;
                    desired_velocity.y = 0;
                    desired_velocity.theta = 0;            
                }
            }

            desired_velocity = rotateVelocity(desired_velocity, -robot_position[i].theta);
            //desired_velocity = boundVelocity(desired_velocity);

            velocity_pub[i].publish(desired_velocity);
            //std_msgs::String velocity_msg = composeVelocityMessage(desired_velocity, action);
            //pub_vel.publish(velocity_msg);
            pub_target.publish(desired_velocity);
        }   
        

        ros::spinOnce();
        rate.sleep();
	}
    return 0;
}
