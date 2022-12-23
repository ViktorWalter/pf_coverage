// STL
#include <iostream>
#include <vector>
#include <chrono>
#include <random>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/impl/utils.h>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/SVD>
#include <cstdlib>
#include <cmath>
#include <netinet/in.h>
#include <sys/types.h>
#include <stdlib.h>
#include <time.h>
#include <fstream>
#include <stdio.h>
#include <unistd.h>
#include <sys/stat.h>
// My includes
#include "pf_coverage/FortuneAlgorithm.h"
#include "pf_coverage/Voronoi.h"
#include "pf_coverage/Graphics.h"
// ROS includes
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/channel_float32.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/bool.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "turtlebot3_msgs/msg/num.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

//Robots parameters ------------------------------------------------------
const double MAX_ANG_VEL = 0.3;
const double MAX_LIN_VEL = 0.5;         //set to turtlebot max velocities
const double b = 0.025;                 //for differential drive control (only if we are moving a differential drive robot (e.g. turtlebot))
//------------------------------------------------------------------------
const bool centralized_centroids = false;   //compute centroids using centralized computed voronoi diagram
const float CONVERGENCE_TOLERANCE = 0.4;
//------------------------------------------------------------------------
const int shutdown_timer = 5;           //count how many seconds to let the robots stopped before shutting down the node


class Controller : public rclcpp::Node
{

public:
    Controller() : Node("coverage_distributed_neighmaster")
    {
        //------------------------------------------------- ROS parameters ---------------------------------------------------------
        this->declare_parameter<int>("ROBOTS_NUM", 3);
        this->get_parameter("ROBOTS_NUM", ROBOTS_NUM);

        //Range di percezione singolo robot (= metà lato box locale)
        this->declare_parameter<double>("ROBOT_RANGE", 3.5);
        this->get_parameter("ROBOT_RANGE", ROBOT_RANGE);
        
        // Parameters for Gaussian
        this->declare_parameter<bool>("GAUSSIAN_DISTRIBUTION", 1);
        this->get_parameter("GAUSSIAN_DISTRIBUTION", GAUSSIAN_DISTRIBUTION);
        this->declare_parameter<double>("PT_X", 4.0);
        this->get_parameter("PT_X", PT_X);
        this->declare_parameter<double>("PT_Y", 4.0);
        this->get_parameter("PT_Y", PT_Y);
        this->declare_parameter<double>("VAR", 2.0);
        this->get_parameter("VAR", VAR);

        //view graphical voronoi rapresentation - bool
        this->declare_parameter<bool>("GRAPHICS_ON", true);
        this->get_parameter("GRAPHICS_ON", GRAPHICS_ON);

        // Area parameter
        this->declare_parameter<double>("AREA_SIZE_x", 20.0);
        this->get_parameter("AREA_SIZE_x", AREA_SIZE_x);
        this->declare_parameter<double>("AREA_SIZE_y", 20.0);
        this->get_parameter("AREA_SIZE_y", AREA_SIZE_y);
        this->declare_parameter<double>("AREA_LEFT", -10.0);
        this->get_parameter("AREA_LEFT", AREA_LEFT);
        this->declare_parameter<double>("AREA_BOTTOM", -10.0);
        this->get_parameter("AREA_BOTTOM", AREA_BOTTOM);

        //Robot ID
        this->declare_parameter<int>("Robot_ID", 0);
        this->get_parameter("Robot_ID", Robot_ID);
        //-----------------------------------------------------------------------------------------------------------------------------------
        //--------------------------------------------------- Subscribers and Publishers ----------------------------------------------------
        for (int i = 0; i < ROBOTS_NUM; i++)
        {
            poseSub_.push_back(this->create_subscription<nav_msgs::msg::Odometry>("/turtlebot" + std::to_string(i) + "/odom", 1, [this, i](nav_msgs::msg::Odometry::SharedPtr msg) {this->poseCallback(msg,i);}));
            neighposePub_.push_back(this->create_publisher<turtlebot3_msgs::msg::Num>("/supervisor/turtlebot" + std::to_string(i) + "/pose", 1));
        }
        timer_ = this->create_wall_timer(100ms, std::bind(&Controller::Coverage, this));
        //rclcpp::on_shutdown(std::bind(&Controller::stop,this));

        //----------------------------------------------------------- init Variables ---------------------------------------------------------
        pose_x = Eigen::VectorXd::Zero(ROBOTS_NUM);
        pose_y = Eigen::VectorXd::Zero(ROBOTS_NUM);
        pose_theta = Eigen::VectorXd::Zero(ROBOTS_NUM);
        time(&this->timer_init_count);
        time(&this->timer_final_count);
        //------------------------------------------------------------------------------------------------------------------------------------

        //----------------------------------------------- Graphics window -----------------------------------------------
        //---------------------------------------------------------------------------------------------------------------
    }
    ~Controller()
    {
        std::cout<<"DESTROYER HAS BEEN CALLED"<<std::endl;
    }

    //void stop(int signum);
    void stop();
    void test_print();
    void poseCallback(const nav_msgs::msg::Odometry::SharedPtr msg, int j);
    void Coverage();
    geometry_msgs::msg::Twist Diff_drive_compute_vel(double vel_x, double vel_y, double alfa);

    //Graphics -- draw coverage
    void check_window_event();

private:
    int ROBOTS_NUM;
    int Robot_ID;
    double ROBOT_RANGE;

    double vel_linear_x, vel_angular_z;
    Eigen::VectorXd pose_x;
    Eigen::VectorXd pose_y;
    Eigen::VectorXd pose_theta;
    std::vector<Vector2<double>> seeds_xy;

    //------------------------- Publishers and subscribers ------------------------------
    std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> poseSub_;
    std::vector<rclcpp::Publisher<turtlebot3_msgs::msg::Num>::SharedPtr> neighposePub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr joySub_;
    rclcpp::TimerBase::SharedPtr timer_;
    //-----------------------------------------------------------------------------------

    //Rendering with SFML
    //------------------------------ graphics window -------------------------------------
    //------------------------------------------------------------------------------------

    //---------------------------- Environment definition --------------------------------
    double AREA_SIZE_x;
    double AREA_SIZE_y;
    double AREA_LEFT;
    double AREA_BOTTOM;
    //------------------------------------------------------------------------------------

    //---------------------- Gaussian Density Function parameters ------------------------
    bool GAUSSIAN_DISTRIBUTION;
    double PT_X;
    double PT_Y;
    double VAR;
    //------------------------------------------------------------------------------------

    //graphical view - ON/OFF
    bool GRAPHICS_ON;

    //timer - check how long robots are being stopped
    time_t timer_init_count;
    time_t timer_final_count;

};

void Controller::stop()
{
}

void Controller::poseCallback(const nav_msgs::msg::Odometry::SharedPtr msg, int j)
{
    this->pose_x(j) = msg->pose.pose.position.x;
    this->pose_y(j) = msg->pose.pose.position.y;

    tf2::Quaternion q(
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    this->pose_theta(j) = yaw;
}

void Controller::Coverage()
{
    //Parameters
    //double min_dist = 0.4;         //avoid robot collision
    int K_gain = 1;                  //Lloyd law gain

    //Variables
    double vel_x=0, vel_y=0;
    std::vector<Vector2<double>> seeds;
    Vector2<double> centroid;
    Vector2<double> vel;

    Box<double> AreaBox{AREA_LEFT, AREA_BOTTOM, AREA_SIZE_x + AREA_LEFT, AREA_SIZE_y + AREA_BOTTOM};
    Box<double> RangeBox{-ROBOT_RANGE, -ROBOT_RANGE, ROBOT_RANGE, ROBOT_RANGE};

    
    for (int i = 0; i < ROBOTS_NUM; ++i)
    {
        turtlebot3_msgs::msg::Num msgs;

        geometry_msgs::msg::PoseStamped msg;

        msg.pose.position.x = this->pose_x[i];
        msg.pose.position.y = this->pose_y[i];

        msg.pose.orientation.z = this->pose_theta[i];
        //tf2::Quaternion q;
        //q.setRPY(0.0, 0.0, 45);
        ////q.normalize();
        //std::cout<<"quaternion::::"<<q.w()<<", "<<q.x()<<", "<<q.y()<<", "<<q.z()<<std::endl;

        msgs.locations.push_back(msg);

        for (int j = 0; j < ROBOTS_NUM; ++j)
        {
            if (i != j)
            {
                Vector2<double> distance_vect = {(this->pose_x[i] - this->pose_x[j]), (this->pose_y[i] - this->pose_y[j])};
                if (distance_vect.getNorm() <= ROBOT_RANGE)
                {
                    geometry_msgs::msg::PoseStamped msg;
                    msg.pose.position.x = this->pose_x[j];
                    msg.pose.position.y = this->pose_y[j];
                    msgs.locations.push_back(msg);
                }
            }
        }

        std::cout<<"turtlebot: "<<i<<", "<<msgs.locations.size()<<std::endl;
        this->neighposePub_[i]->publish(msgs);
    }
}


//alternatively to a global variable to have access to the method you can make STATIC the class method interested, 
//but some class function may not be accessed: "this->" method cannot be used

std::shared_ptr<Controller> globalobj_signal_handler;     //the signal function requires only one argument {int}, so the class and its methods has to be global to be used inside the signal function.
void nodeobj_wrapper_function(int){
    std::cout<<"signal handler function CALLED"<<std::endl;
    globalobj_signal_handler->stop();
}

int main(int argc, char **argv)
{
    signal(SIGINT, nodeobj_wrapper_function);

    rclcpp::init(argc, argv);
    auto node = std::make_shared<Controller>();

    globalobj_signal_handler = node;    //to use the ros function publisher, ecc the global pointer has to point to the same node object.

    //---------------------------Create a callable pointer funcion from outside the class object------------------------

    //auto stop_controller_callable = std::mem_fun(&Controller::stop);
    //Controller controller_obj;
    //stop_controller_callable(&controller_obj, 12231);

    //il placeholder indica se e quanti elementi ci sono in ingresso alla funzione/metodo dato a std::bind
    //std::function<void(int)> close_controller = std::bind(&Controller::stop, &controller_obj, std::placeholders::_1);

    //right solution to bind a function !!
    //Controller controller_obj;
    //std::function<void()> close_controller = std::bind(&Controller::stop, &controller_obj);

    //std::function<void()> test = std::bind(&Controller::test_print, &controller_obj);
    //-------------------------------------------------------------------------------------------------------------------

    //rclcpp::on_shutdown(close_controller);

    rclcpp::spin(node);

    rclcpp::sleep_for(100000000ns);
    rclcpp::shutdown();

    return 0;
}
