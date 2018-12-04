//Hacer el .h
//Hacer el install (?)
//Subscripciones, aguas
//CMakeLists

//63-66 cambiar constantes 
//Checar como declarar los eigen en include

#include "ek_plugin.h"

#define ADJ_FACT 1 / 12 // original 1/50
#define VEL_MAX 2.5
using namespace Eigen;

double rate_hz = 50;

namespace gazebo
{
  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(ek_plugin)


  ek_plugin::ek_plugin()
  {

  }

  ek_plugin::~ek_plugin()
  {

  }

  /// \brief The load function is called by Gazebo when the plugin is
  /// inserted into simulation
  /// \param[in] _model A pointer to the model that this plugin is
  /// attached to.
  /// \param[in] _sdf A pointer to the plugin's SDF element.
  void ek_plugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {
      msg_exists = false;
      //printf("AT LOAD\n");
    // Safety check
    if (_model->GetJointCount() == 0)
    {
      std::cerr << "Invalid joint count, ek_plugin not loaded\n";
      return;
    }

     this->pmq.startServiceThread();

    // Store the model pointer for convenience.
    this->model = _model;

    //const std::string val = "Printing from plugin...";
    // Get the first joint. We are making an assumption about the model
    // having one joint that is the rotational joint.
    this->joint_w1 = _model->GetJoint("joint_chassis_omniwheel1");
    this->joint_w2 = _model->GetJoint("joint_chassis_omniwheel2");
    this->joint_w3 = _model->GetJoint("joint_chassis_omniwheel3");
    this->joint_w4 = _model->GetJoint("joint_chassis_omniwheel4");

    this->joint_w1->SetEffortLimit(0,50000);
    this->joint_w2->SetEffortLimit(0,50000);
    this->joint_w3->SetEffortLimit(0,50000);
    this->joint_w4->SetEffortLimit(0,50000);
    //std::cout << val << std::endl;

    //std::cout << this->joint->GetName() << std::endl;


    // Setup a P-controller, with a gain of 0.1.
    this->pid_w1 = common::PID(1,0.1,0.3);//3, 1, 0.1);
    this->pid_w2 = common::PID(1,0.1,0.3);//3, 1, 0.1);
    this->pid_w3 = common::PID(1,0.1,0.3);//3, 1, 0.1);
    this->pid_w4 = common::PID(1,0.1,0.3);//3, 1, 0.1);
    // Apply the P-controller to the joint.


    this->model->GetJointController()->SetPositionPID(
      this->joint_w1->GetScopedName(), this->pid_w1);
    this->model->GetJointController()->SetPositionPID(
      this->joint_w2->GetScopedName(), this->pid_w2);
    this->model->GetJointController()->SetPositionPID(
      this->joint_w3->GetScopedName(), this->pid_w3);
    this->model->GetJointController()->SetPositionPID(
      this->joint_w4->GetScopedName(), this->pid_w4);

    //this->position = 0;
    //this->vel = 0;

    if (!ros::isInitialized())
    {
      int argc = 0;
      char **argv = NULL;
      ros::init(argc, argv, "gazebo_client",
        ros::init_options::NoSigintHandler);
    }

        // Create our ROS node. This acts in a similar manner to
        // the Gazebo node
    this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

        // Create a named topic, and subscribe to it.
    ros::SubscribeOptions so =
    ros::SubscribeOptions::create<geometry_msgs::Pose2D>(
      "/" + this->model->GetName() + "/com_101",
      1,
      boost::bind(&ek_plugin::OnRosMsg_vel, this, _1),
      ros::VoidPtr(), &this->rosQueue);
    this->rosSub = this->rosNode->subscribe(so);

    // Spin up the queue helper thread.
    this->rosQueueThread =
    std::thread(std::bind(&ek_plugin::QueueThread, this));

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&ek_plugin::OnUpdate, this, _1));

     ros::AdvertiseOptions ao =
      ros::AdvertiseOptions::create<geometry_msgs::Pose2D>(
      "/" + this->model->GetName() + "/pos", 1,
      boost::bind(&ek_plugin::ek_connect, this),
      boost::bind(&ek_plugin::ek_disconnect, this),
      ros::VoidPtr(), NULL);

      this->pub_ = this->rosNode->advertise(ao);
      
      this->pub_queue_ = this->pmq.addPub<geometry_msgs::Pose2D>();
  }

  /// \brief Set the velocity of the Velodyne
  /// \param[in] _vel New target velocity
  void ek_plugin::ek_connect()
  {
    printf("At: %s\n",__PRETTY_FUNCTION__);
  }

  void ek_plugin::ek_disconnect()
  {
    printf("At: %s\n",__PRETTY_FUNCTION__);    
  }

  void ek_plugin::SetPosition(const double &_pos)
  {

  }

  /// \brief Handle an incoming message from ROS
  /// \param[in] _msg A float value that is used to set the velocity
  /// of the Velodyne.

  void ek_plugin::OnRosMsg_vel(const geometry_msgs::Pose2DConstPtr &_msg)
  {
    //std::cout << "OnRosMsg_vel activado\n";
    // jalar los datos en pose2d
    //this->vel_x = _msg->data.x;
    //this->vel_y = _msg->data.y;
    //this->vel_theta = _msg->data.theta;

    this->msg_exists = true;
    //std::cout << "msg_exists activado\n";
    this->x_bot = _msg->x;
    this->y_bot = _msg->y;
    this->theta_bot = _msg->theta;
    //std::cout << "OnRosMsg_vel terminado\n";
    //std::cout << "On OnRosMsg_vel (new): \n x: " << x_bot << "\n y: " << y_bot << "\n theta: " << theta_bot <<"\n";
  }

  Vector4f ek_plugin::BalanceSpeeds(Vector4f w)
  {
    float max_abs;
    if(std::abs(w.maxCoeff())>std::abs(w.minCoeff()))
      max_abs = std::abs(w.maxCoeff());
    else
      max_abs = std::abs(w.minCoeff());
    if(max_abs>VEL_MAX){
      float factor = max_abs/VEL_MAX;
      Vector4f w_limited = w/factor;
      //w_limited(0) = 0;
      //w_limited(1) = 0;
      return w_limited;
    } else {
      return w;
    }
  }

  void ek_plugin::OnUpdate(const common::UpdateInfo &)
  {    
    //std::cout << "OnUpdate activado \n";
    // compute the steptime for the PID
    common::Time currTime = this->model->GetWorld()->GetSimTime();
    common::Time stepTime = currTime - this->prevUpdateTime;
    this->prevUpdateTime = currTime;

    // set the current position of the joint, and the target position, 
    // and the maximum effort limit
    //float pos_target = this->position; // Steering
    //float pos_curr = this->joint->GetAngle(0).Degree();
    // x4 para cada llanta
    if(msg_exists){
        //std::cout << "Entro al msg_exists\n";
        MatrixXf vel_transform(4,3);//Hay que poner "Eigen::"?
        vel_transform << 0.8090,0.5878,85,
                         -0.8090,0.5878,85,
                         -.6691,-0.7431,85,
                         0.6691,-0.7431,85;
        VectorXf vel_bot(3);
        vel_bot << this->x_bot, this->y_bot, this->theta_bot;
        //std::cout <<"\n vel_bot_target: \n" << vel_bot;
        Vector4f vel_target;
        vel_target = vel_transform * vel_bot;
        //std::cout <<"\n vel_target: \n" << vel_target;
        Vector4f vel_target_limited = BalanceSpeeds(vel_target);
        //std::cout <<"\n vel_target_limited: \n" << vel_target_limited;
        float vel_curr_w1 = this->joint_w1->GetVelocity(0);
        float vel_curr_w2 = this->joint_w2->GetVelocity(0);
        float vel_curr_w3 = this->joint_w3->GetVelocity(0);
        float vel_curr_w4 = this->joint_w4->GetVelocity(0);
        //std::cout << "\n vel_curr: \nw1: " <<vel_curr_w1<<"\nw2: "<<vel_curr_w2<<"\nw3: "<<vel_curr_w3<<"\nw4: "<<vel_curr_w4;
        // calculate the error between the current position and the target one
        //double pos_err = pos_curr - pos_target;
        double vel_err_w1 = vel_curr_w1 - vel_target_limited(0);
        double vel_err_w2 = vel_curr_w2 - vel_target_limited(1);
        double vel_err_w3 = vel_curr_w3 - vel_target_limited(2);
        double vel_err_w4 = vel_curr_w4 - vel_target_limited(3);

        // compute the effort via the PID, which you will apply on the joint
        //double effort_cmd = this->pid.Update(pos_err, stepTime);
        double effort_cmd_v1 = this->pid_w1.Update(vel_err_w1, stepTime);
        double effort_cmd_v2 = this->pid_w2.Update(vel_err_w2, stepTime);
        double effort_cmd_v3 = this->pid_w3.Update(vel_err_w3, stepTime);
        double effort_cmd_v4 = this->pid_w4.Update(vel_err_w4, stepTime);

        //std::cout << "\n Effr: \nw1: " <<effort_cmd_v1<<"\nw2: "<<effort_cmd_v2<<"\nw3: "<<effort_cmd_v3<<"\nw4: "<<effort_cmd_v4;
        //std::cout << "\n Effr: \nw1: " <<vel_err_w1<<"\nw2: "<<vel_err_w2<<"\nw3: "<<vel_err_w3<<"\nw4: "<<vel_err_w4;
        // apply the force on the joint
        //this->joint->SetForce(0, effort_cmd);
        this->joint_w1->SetForce(0, effort_cmd_v1);
        this->joint_w2->SetForce(0, effort_cmd_v2);
        this->joint_w3->SetForce(0, effort_cmd_v3);
        this->joint_w4->SetForce(0, effort_cmd_v4);
        //this->joint_w1->SetForce(0, vel_err_w1);
        //this->joint_w2->SetForce(0, vel_err_w2);
        //this->joint_w3->SetForce(0, vel_err_w3);
        //this->joint_w4->SetForce(0, vel_err_w4);
    }
        //Para checar, publica
        double x,y,z;
        gazebo::math::Pose pose;
        pose = this->model->GetWorldPose();
        math::Vector3 v(0, 0, 0);

        geometry_msgs::Pose2D pose_msg;
        pose_msg.x = pose.pos.x;
        pose_msg.y = pose.pos.y;
        pose_msg.theta = pose.rot.GetYaw();

        //this->pub_queue_->push(pose_msg, this->pub_);
        //ROS_INFO_STREAM("pose_msg: " << pose_msg);
        this->pub_.publish(pose_msg);
        
        //ros::NodeHandle nh;
        //ros::Publisher position_pub;
        //position_pub = nh.advertise<geometry_msgs::Pose2D>(pose_msg, rate_hz);


  }
      
  /// \brief ROS helper function that processes messages
  void ek_plugin::QueueThread()
  {
    static const double timeout = 0.01;
    while (this->rosNode->ok())
    {
      this->rosQueue.callAvailable(ros::WallDuration(timeout));
    }
  }      
}
