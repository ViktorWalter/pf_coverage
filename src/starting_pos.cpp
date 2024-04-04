#include <iostream>
#include <vector>
#include <chrono>
#include <random>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/impl/utils.h>
#include <functional>
#include <memory>
#include <string>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/SVD>
#include <cstdlib>
#include <cmath>
#include <algorithm>
#include <iterator>
#include <netinet/in.h>
#include <sys/types.h>
#include <stdlib.h>
#include <time.h>
#include <fstream>
#include <stdio.h>
#include <unistd.h>
#include <sys/stat.h>
#include <math.h>
#include <signal.h>
#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <mrs_msgs/PoseWithCovarianceArrayStamped.h>
#include <mrs_msgs/PoseWithCovarianceIdentified.h>
#include <mrs_msgs/VelocityReferenceStamped.h>

#define M_PI   3.14159265358979323846  /*pi*/

using namespace std::chrono_literals;
using std::placeholders::_1;


sig_atomic_t volatile node_shutdown_request = 0;    //signal manually generated when ctrl+c is pressed


class Controller
{
public:
    Controller(): nh_priv_("~")
    {
        this->nh_priv_.getParam("ROBOTS_NUM", ROBOTS_NUM);

        // ID of the controlled robot
        this->nh_priv_.getParam("ROBOT_ID", ROBOT_ID);
        
        //Range di percezione singolo robot (= metà lato box locale)
        this->nh_priv_.getParam("ROBOT_RANGE", ROBOT_RANGE);
        this->nh_priv_.getParam("ROBOT_FOV", ROBOT_FOV);

        this->nh_priv_.getParam("XGOAL", XGOAL);
        this->nh_priv_.getParam("YGOAL", YGOAL);
        this->nh_priv_.getParam("ZGOAL", ZGOAL);
        this->nh_priv_.getParam("THGOAL", THGOAL);
        
        ROBOT_FOV_rad = ROBOT_FOV /180 * M_PI;

        p_i = Eigen::Vector3d::Zero();
        p_j = 100 * Eigen::Vector2d::Ones();

        odomSub_ = nh_.subscribe<nav_msgs::Odometry>("/uav" + std::to_string(ROBOT_ID) + "/ground_truth", 1, std::bind(&Controller::odomCallback, this, std::placeholders::_1));
        pub_ = nh_.advertise<mrs_msgs::VelocityReferenceStamped>("/uav" + std::to_string(ROBOT_ID) + "/control_manager/velocity_reference", 1);
        // timer_ = nh_.createTimer(ros::Duration(0.1), std::bind(&Controller::timer_callback, this));
    }
    
    ~Controller()
    {
        std::cout<<"DESTROYER HAS BEEN CALLED"<<std::endl;
    }

    void stop();
    void odomCallback(const nav_msgs::Odometry::ConstPtr msg);
    bool insideFOV(Eigen::VectorXd q, Eigen::VectorXd q_obs, double fov, double r_sens);
    // void timer_callback();

private:
    int ROBOT_ID = 0;
    int ROBOTS_NUM = 2;
    double ROBOT_FOV = 160.0;
    double ROBOT_FOV_rad;
    double ROBOT_RANGE = 10.0;

    double XGOAL = 0.0;
    double YGOAL = -4.0;
    double ZGOAL = 2.0;
    double THGOAL = 0.5*M_PI;

    Eigen::Vector3d p_i;
    Eigen::Vector2d p_j;
    
    
    ros::NodeHandle nh_;
    ros::NodeHandle nh_priv_;
    ros::Subscriber neighSub_;
    ros::Subscriber odomSub_;
    std::vector<ros::Subscriber> realposeSub_;
    ros::Publisher pub_;
    ros::Timer timer_;

};

void Controller::stop()
{
    //if (signum == SIGINT || signum == SIGKILL || signum ==  SIGQUIT || signum == SIGTERM)
    ROS_INFO("shutting down the controller, stopping the robots, closing the graphics window");
    // if ((GRAPHICS_ON) && (this->app_gui->isOpen())){
    //     this->app_gui->close();
    // }
    // this->timer_->cancel();
    ros::Duration(0.1).sleep();

    ROS_INFO("controller has been closed and robots have been stopped");
    ros::Duration(0.1).sleep();

    ros::shutdown();
}


void Controller::odomCallback(const nav_msgs::Odometry::ConstPtr msg)
{
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double z = msg->pose.pose.position.z;

    tf2::Quaternion q(
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    double th = yaw;

    double K = 0.5;

    // Publish control input
    mrs_msgs::VelocityReferenceStamped vel_msg;
    // vel_msg.header.frame_id = "uav" + std::to_string(ROBOT_ID) + "/local_origin";
    // vel_msg.header.frame_id = "/hummingbird" + std::to_string(ROBOT_ID) + "/base_link";
    vel_msg.reference.velocity.x = K * (XGOAL-x);
    vel_msg.reference.velocity.y = K * (YGOAL-y);
    vel_msg.reference.velocity.z = K * (ZGOAL-z);
    vel_msg.reference.altitude = 2.0;
    vel_msg.reference.heading_rate = K * (THGOAL - th);
    vel_msg.reference.use_altitude = true;
    vel_msg.reference.use_heading_rate = true;
    this->pub_.publish(vel_msg);

    

    // std::cout << "I'm in odomCallback" << "\n";
    // std::cout << "Robot position: " << this->pose_x(ROBOT_ID) << ", " << this->pose_y(ROBOT_ID) << ", " << this->pose_theta(ROBOT_ID) << "\n";
}

bool Controller::insideFOV(Eigen::VectorXd q, Eigen::VectorXd q_obs, double fov, double r_sens)
{
    /* ----------------------------------------------
    Check if q_obs is inside the field of view of q
    q = [x, y, th] : robot pose
    q_obs = [x, y] : neighbor pose (orientation ignored)
    fov = field of view [rad]
    r_sens = sensing range [m]
    
    th_obs = neighbor bearing wrt [0,0,0]
    th_diff = neighbor bearing wrt robot reference frame
    -------------------------------------------------*/

    double dist = sqrt(pow(q_obs(0)-q(0),2) + pow(q_obs(1)-q(1),2));
    double th = q(2);
    double th_obs = atan2(q_obs(1)-q(1), q_obs(0)-q(0));
    double th_rel = th_obs - th;

    double fov_rad = fov*M_PI/180;

    if (th_rel > M_PI)
        th_rel -= 2*M_PI;
    else if (th_rel < -M_PI)
        th_rel += 2*M_PI;

    if (th_rel > 0.5*fov_rad || th_rel < -0.5*fov_rad || dist > r_sens)
        return false;
    else
        return true;
}




std::shared_ptr<Controller> globalobj_signal_handler;     //the signal function requires only one argument {int}, so the class and its methods has to be global to be used inside the signal function.
void nodeobj_wrapper_function(int){
    std::cout<<"signal handler function CALLED"<<std::endl;
    node_shutdown_request = 1;
}

int main(int argc, char **argv)
{
    signal(SIGINT, nodeobj_wrapper_function);

    ros::init(argc, argv, "vision_sim", ros::init_options::NoSigintHandler);
    auto node = std::make_shared<Controller>();

    // globalobj_signal_handler = node;    //to use the ros function publisher, ecc the global pointer has to point to the same node object.


    // rclcpp::spin(node);

    // rclcpp::sleep_for(100000000ns);
    // rclcpp::shutdown();

    while(!node_shutdown_request)
    {
        ros::spinOnce();
    }
    node->stop();

    if (ros::ok())
    {
        ROS_WARN("ROS HAS NOT BEEN PROPERLY SHUTDOWN, it is being shutdown again.");
        ros::shutdown();
    }

    return 0;
}