#ifndef _ek_PLUGIN_HH_
#define _ek_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <thread>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <gazebo_plugins/PubQueue.h>
#include <Eigen/Geometry>//Así?
#include <Eigen/Dense>//Así?
#include <Eigen/QR>//Así?
#include <Eigen/LU>//Así?

using namespace Eigen;

namespace gazebo
{
  /// \brief A plugin to control a Velodyne sensor.
  class ek_plugin : public ModelPlugin
  {
    /// \brief Constructor
    public: 
      ek_plugin();

      ~ek_plugin();



    /// \brief The load function is called by Gazebo when the plugin is
    /// inserted into simulation
    /// \param[in] _model A pointer to the model that this plugin is
    /// attached to.
    /// \param[in] _sdf A pointer to the plugin's SDF element.
      virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Set the velocity of the Velodyne
    /// \param[in] _vel New target velocity
      void SetPosition(const double &_pos);
    /// \brief Handle an incoming message from ROS
    /// \param[in] _msg A float value that is used to set the velocity
    /// of the Velodyne.
      //void OnRosMsg_steering(const std_msgs::Int16ConstPtr &_msg);

      void OnRosMsg_vel(const geometry_msgs::Pose2DConstPtr &_msg);
      
      void OnUpdate(const common::UpdateInfo & /*_info*/);
      Vector4f BalanceSpeeds(Vector4f);
    /// \brief ROS helper function that processes messages
    private: 
      void QueueThread();

      void ek_connect();

      void ek_disconnect();

      //float position;

      //float vel_x;
      //float vel_y;
      //float vel_theta;
      bool msg_exists;

      ros::Publisher pub_;
      PubQueue<geometry_msgs::Pose2D>::Ptr pub_queue_;
      
      PubMultiQueue pmq;
      
      event::ConnectionPtr updateConnection;
    /// \brief Pointer to the model.
      physics::ModelPtr model;

    /// \brief Pointer to the joint.
      physics::JointPtr joint_w1;
      physics::JointPtr joint_w2;
      physics::JointPtr joint_w3;
      physics::JointPtr joint_w4;


    /// \brief A PID controller for the joint.
      common::PID pid_w1;
      common::PID pid_w2;
      common::PID pid_w3;
      common::PID pid_w4;

      VectorXf vel_bot;
      float x_bot;
      float y_bot;
      float theta_bot;
    /// \brief A node use for ROS transport
      std::unique_ptr<ros::NodeHandle> rosNode;

      common::Time prevUpdateTime;
    /// \brief A ROS subscriber
      ros::Subscriber rosSub;
      //ros::Subscriber rosSub_vel;

      ros::NodeHandle _nh;
        
    /// \brief A ROS callbackqueue that helps process messages
      ros::CallbackQueue rosQueue;
        
    /// \brief A thread the keeps running the rosQueue
      std::thread rosQueueThread;

  };
        
}
#endif
