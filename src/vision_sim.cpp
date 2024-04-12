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
        
        ROBOT_FOV_rad = ROBOT_FOV /180 * M_PI;

        p_i = Eigen::Vector3d::Zero();

        p_j.resize(3, ROBOTS_NUM);
        p_j.setZero();
        p_j_real.resize(3, ROBOTS_NUM);
        p_j_real.setZero();

        for (int i = 0; i < ROBOTS_NUM; i++)
        {
            odomSubs_.push_back(nh_.subscribe<nav_msgs::Odometry>("/uav" + std::to_string(i) + "/estimation_manager/odom_main", 1, std::bind(&Controller::odomCallback, this, std::placeholders::_1, i)));
        }

        // odomSub_ = nh_.subscribe<nav_msgs::Odometry>("/uav" + std::to_string(ROBOT_ID) + "/estimation_manager/odom_main", 1, std::bind(&Controller::odomCallback, this, std::placeholders::_1));
        neighSub_ = nh_.subscribe<mrs_msgs::PoseWithCovarianceArrayStamped>("/uav" + std::to_string(ROBOT_ID) + "/uvdar/measuredPoses", 1, std::bind(&Controller::neighCallback, this, std::placeholders::_1));
        pub_ = nh_.advertise<mrs_msgs::PoseWithCovarianceArrayStamped>("uav" + std::to_string(ROBOT_ID) + "/fake_vision", 1);
        fakepub_ = nh_.advertise<mrs_msgs::PoseWithCovarianceArrayStamped>("uav" + std::to_string(ROBOT_ID) + "/ground_truth/fake_vision", 1);
        timer_ = nh_.createTimer(ros::Duration(0.1), std::bind(&Controller::timer_callback, this));
    }
    
    ~Controller()
    {
        std::cout<<"DESTROYER HAS BEEN CALLED"<<std::endl;
    }

    void stop();
    void neighCallback(const mrs_msgs::PoseWithCovarianceArrayStamped::ConstPtr msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr msg, int i);
    bool insideFOV(Eigen::VectorXd q, Eigen::VectorXd q_obs, double fov, double r_sens);
    void timer_callback();

private:
    int ROBOT_ID = 0;
    int ROBOTS_NUM = 4;
    double ROBOT_FOV = 160.0;
    double ROBOT_FOV_rad;
    double ROBOT_RANGE = 10.0;

    Eigen::Vector3d p_i;
    Eigen::MatrixXd p_j;
    Eigen::MatrixXd p_j_real;
    
    
    ros::NodeHandle nh_;
    ros::NodeHandle nh_priv_;
    ros::Subscriber neighSub_;
    std::vector<ros::Subscriber> odomSubs_;
    std::vector<ros::Subscriber> realposeSub_;
    ros::Publisher pub_;
    ros::Publisher fakepub_;
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


void Controller::neighCallback(const mrs_msgs::PoseWithCovarianceArrayStamped::ConstPtr msg)
{
    std::vector<bool> received(ROBOTS_NUM, false);
    mrs_msgs::PoseWithCovarianceArrayStamped msg_filtered;
    for (int j = 0; j < msg->poses.size(); j++)
    {
        int id = msg->poses[j].id;
        received[id] = true;
        p_j(0, j) = msg->poses[j].pose.position.x;
        p_j(1, j) = msg->poses[j].pose.position.y;
        Eigen::MatrixXd cov;
        cov.resize(2,2);
        cov << msg->poses[j].covariance[0], msg->poses[j].covariance[1], //msg->poses[j].covariance[2],
                msg->poses[j].covariance[6], msg->poses[j].covariance[7]; //msg->poses[j].covariance[8];
                // msg->poses[j].covariance[12], msg->poses[j].covariance[13], msg->poses[j].covariance[14];
        std::cout << "Cov matrix: \n" << cov << std::endl;
        Eigen::EigenSolver<Eigen::MatrixXd> es(cov);
        Eigen::VectorXd eigenvalues  = es.eigenvalues().real();
        std::cout << "EigenValues of the cov matrix : " << eigenvalues.transpose() << std::endl; 
        bool discard = (eigenvalues.array() > 50.0).any();
        if (insideFOV(p_i, p_j.col(j).head(2), ROBOT_FOV, ROBOT_RANGE))
        // if (true)
        {
            std::cout << "Robot " << id << " detected in " << p_j.col(j).transpose() << std::endl;
            msg_filtered.poses.push_back(msg->poses[j]);
        } else
        {
            mrs_msgs::PoseWithCovarianceIdentified fake_msg;
            fake_msg.id = id;
            fake_msg.pose.position.x = 100.0;
            fake_msg.pose.position.y = 100.0;
            std::cout << "Robot " << id << " not detected. \n";
            msg_filtered.poses.push_back(fake_msg);
        }
    }

    for (int i = 0; i < received.size(); i++)
    {
        if (!received[i])
        {
            mrs_msgs::PoseWithCovarianceIdentified fake_msg;
            fake_msg.id = i;
            fake_msg.pose.position.x = 100.0;
            fake_msg.pose.position.y = 100.0;
            std::cout << "Robot " << i << " not received. \n";
            msg_filtered.poses.push_back(fake_msg);
        }
    }

    this->pub_.publish(msg_filtered);
    // std::cout << "Global position of robots: \n" << p_j << std::endl;
}

void Controller::odomCallback(const nav_msgs::Odometry::ConstPtr msg, int i)
{
    std::vector<bool> received(ROBOTS_NUM, false);
    mrs_msgs::PoseWithCovarianceArrayStamped msg_filtered;
    p_j_real(0, i) = msg->pose.pose.position.x;
    p_j_real(1, i) = msg->pose.pose.position.y;
    tf2::Quaternion q(
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    p_j_real(2, i) = yaw;

    /*for (int j = 0; j < msg->poses.size(); j++)
    {
        int id = msg->poses[j].id;
        received[id] = true;
        p_j(0,i) = msg->poses[j].pose.position.x;
        p_j(1,i) = msg->poses[j].pose.position.y;
        Eigen::MatrixXd cov;
        cov.resize(2,2);
        cov << msg->poses[j].covariance[0], msg->poses[j].covariance[1], //msg->poses[j].covariance[2],
                msg->poses[j].covariance[6], msg->poses[j].covariance[7]; //msg->poses[j].covariance[8];
                // msg->poses[j].covariance[12], msg->poses[j].covariance[13], msg->poses[j].covariance[14];
        std::cout << "Cov matrix: \n" << cov << std::endl;
        Eigen::EigenSolver<Eigen::MatrixXd> es(cov);
        Eigen::VectorXd eigenvalues  = es.eigenvalues().real();
        std::cout << "EigenValues of the cov matrix : " << eigenvalues.transpose() << std::endl; 
        bool discard = (eigenvalues.array() > 50.0).any();
        if (insideFOV(p_i, p_j, ROBOT_FOV, ROBOT_RANGE))
        // if (true)
        {
            std::cout << "Robot " << id << " detected in " << p_j.transpose() << std::endl;
            msg_filtered.poses.push_back(msg->poses[j]);
        } else
        {
            mrs_msgs::PoseWithCovarianceIdentified fake_msg;
            fake_msg.id = id;
            fake_msg.pose.position.x = 100.0;
            fake_msg.pose.position.y = 100.0;
            std::cout << "Robot " << id << " not detected. \n";
            msg_filtered.poses.push_back(fake_msg);
        }
    }

    for (int i = 0; i < received.size(); i++)
    {
        if (!received[i])
        {
            mrs_msgs::PoseWithCovarianceIdentified fake_msg;
            fake_msg.id = i;
            fake_msg.pose.position.x = 100.0;
            fake_msg.pose.position.y = 100.0;
            std::cout << "Robot " << i << " not received. \n";
            msg_filtered.poses.push_back(fake_msg);
        }
    }

    this->pub_.publish(msg_filtered);*/
    // std::cout << "Global position of robots: \n" << p_j << std::endl;
}

void Controller::timer_callback()
{
    mrs_msgs::PoseWithCovarianceArrayStamped msg_filtered;
    Eigen::VectorXd robot = p_j_real.col(ROBOT_ID);
    Eigen::MatrixXd R;
    R.resize(2,2);
    R << cos(robot(2)), -sin(robot(2)),
            sin(robot(2)), cos(robot(2));
    for (int i = 0; i < ROBOTS_NUM; i++)
    {
        if (i != ROBOT_ID)
        {
            Eigen::Vector2d p_j_i;
            double dx = p_j_real(0, i) - robot(0);
            double dy = p_j_real(1, i) - robot(1);
            p_j_i(0) = dx * cos(robot(2)) + dy * sin(robot(2));
            p_j_i(1) = -dx * sin(robot(2)) + dy * cos(robot(2));
            if (insideFOV(Eigen::Vector3d::Zero(), p_j_i, ROBOT_FOV, ROBOT_RANGE))
            {
                // Eigen::VectorXd p_j_i = R.transpose() * (p_j_real.col(i).head(2) - robot.head(2));
                mrs_msgs::PoseWithCovarianceIdentified det_msg;
                det_msg.id = i;
                det_msg.pose.position.x = p_j_i(0);
                det_msg.pose.position.y = p_j_i(1);
                msg_filtered.poses.push_back(det_msg);
            } else
            {
                mrs_msgs::PoseWithCovarianceIdentified fake_msg;
                fake_msg.id = i;
                fake_msg.pose.position.x = 100.0;
                fake_msg.pose.position.y = 100.0;
                // std::cout << "Robot " << i << " not received. \n";
                msg_filtered.poses.push_back(fake_msg);
            }
        } else
        {
            mrs_msgs::PoseWithCovarianceIdentified fake_msg;
            fake_msg.id = i;
            fake_msg.pose.position.x = 100.0;
            fake_msg.pose.position.y = 100.0;
            // std::cout << "Robot " << i << " not received. \n";
            msg_filtered.poses.push_back(fake_msg);
        }
        
    }

    this->fakepub_.publish(msg_filtered);
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
    ros::Duration sleeper(0.01);

    while(!node_shutdown_request)
    {
        ros::spinOnce();
        sleeper.sleep();
    }
    node->stop();

    if (ros::ok())
    {
        ROS_WARN("ROS HAS NOT BEEN PROPERLY SHUTDOWN, it is being shutdown again.");
        ros::shutdown();
    }

    return 0;
}
