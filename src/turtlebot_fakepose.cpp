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
#include <algorithm>
#include <netinet/in.h>
#include <sys/types.h>
#include <stdlib.h>
#include <time.h>
#include <fstream>
#include <stdio.h>
#include <unistd.h>
#include <sys/stat.h>
#include <math.h>
// SFML
// #include <SFML/Graphics.hpp>
// #include <SFML/OpenGL.hpp>
// My includes
#include "pf_coverage/FortuneAlgorithm.h"
#include "pf_coverage/Voronoi.h"
// #include "Graphics.h"
// ROS includes
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "sensor_msgs/msg/channel_float32.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/bool.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "turtlebot3_msgs/msg/gaussian.hpp"
#include "turtlebot3_msgs/msg/gmm.hpp"
// #include "std_srvs/srv/empty.hpp"

#define M_PI   3.14159265358979323846  /*pi*/

using namespace std::chrono_literals;
using std::placeholders::_1;


bool IsPathExist(const std::string &s)
{
  struct stat buffer;
  return (stat (s.c_str(), &buffer) == 0);
}


class Supervisor : public rclcpp::Node
{

public:
    Supervisor() : Node("optitrack_emulator")
    {
        //------------------------------------------------- ROS parameters ---------------------------------------------------------
        this->declare_parameter<int>("ROBOTS_NUM", 4);
        this->get_parameter("ROBOTS_NUM", ROBOTS_NUM);

        //Range di percezione singolo robot (= metà lato box locale)
        this->declare_parameter<double>("ROBOT_RANGE", 4.0);
        this->get_parameter("ROBOT_RANGE", ROBOT_RANGE);
        
        // Field of view (degrees)
        this->declare_parameter<double>("ROBOT_FOV", 120.0);
        this->get_parameter("ROBOT_FOV", ROBOT_FOV);
        
        //view graphical voronoi rapresentation - bool
        // this->declare_parameter<bool>("GRAPHICS_ON", true);
        // this->get_parameter("GRAPHICS_ON", GRAPHICS_ON);

        // Area parameter
        this->declare_parameter<double>("AREA_SIZE_x", 10);
        this->get_parameter("AREA_SIZE_x", AREA_SIZE_x);
        this->declare_parameter<double>("AREA_SIZE_y", 10);
        this->get_parameter("AREA_SIZE_y", AREA_SIZE_y);
        this->declare_parameter<double>("AREA_LEFT", -5);
        this->get_parameter("AREA_LEFT", AREA_LEFT);
        this->declare_parameter<double>("AREA_BOTTOM", -5);
        this->get_parameter("AREA_BOTTOM", AREA_BOTTOM);


        this->declare_parameter<bool>("SAVE_POS", false);
        this->get_parameter("SAVE_POS", SAVE_POS);



        // -------------------------------- ROS publishers and subscribers --------------------------------
        for (int i = 0; i < ROBOTS_NUM; i++)
        {
            fakeposePub_.push_back(this->create_publisher<geometry_msgs::msg::PoseStamped>("/vrpn_client_node/turtle" + std::to_string(i) + "/pose", 1));
        }

        // service = this->create_service<std_srvs::srv::Empty>("rotate", &Supervisor::srv_callback, this);

        timer_ = this->create_wall_timer(200ms, std::bind(&Supervisor::emulate_vision, this));

        //----------------------------------------------------------- init Variables ---------------------------------------------------------
        pose_x = Eigen::VectorXd::Zero(ROBOTS_NUM);
        pose_y = Eigen::VectorXd::Zero(ROBOTS_NUM);
        pose_theta = Eigen::VectorXd::Zero(ROBOTS_NUM);
        time(&this->timer_init_count);
        time(&this->timer_final_count);

        x_arr = {0.5, 2.0, 2.0, 3.0};
        y_arr = {0.5, 1.0, -1.0, 0.0};
        theta_arr = {3.14, 0.0, 0.0, 0.0};

        ROBOT_FOV_rad = ROBOT_FOV*M_PI/180;

        if(SAVE_POS)
        {
            open_log_file();
        }
        
    }
        

    ~Supervisor()
    {
        std::cout<<"DESTROYER HAS BEEN CALLED"<<std::endl;
        if(SAVE_POS)
        {
            close_log_file();
            std::cout << "LOG FILE HAS BEEN CLOSED" << std::endl;
        }
    }

    void stop();
    void emulate_vision();
    // void srv_callback(const std::shared_ptr<std_srvs::srv::Empty::Request> request, std::shared_ptr<std_srvs::srv::Empty::Response> response);

    //open write and close LOG file
    void open_log_file();
    void write_log_file(std::string text);
    void close_log_file();



private:

    int ROBOTS_NUM;
    double ROBOT_RANGE;
    double ROBOT_FOV;
    double ROBOT_FOV_rad;
    bool SAVE_POS;
    Eigen::VectorXd pose_x;
    Eigen::VectorXd pose_y;
    Eigen::VectorXd pose_theta;

    std::vector<double> x_arr;
    std::vector<double> y_arr;
    std::vector<double> theta_arr;

    //------------------------- Publishers and subscribers ------------------------------
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> fakeposePub_;
    // rclcpp::Service<std_srvs::srv::Empty>::SharedPtr service;
    //-----------------------------------------------------------------------------------

    // ------------------------ Environment definition ------------------------
    double AREA_SIZE_x;
    double AREA_SIZE_y;
    double AREA_LEFT;
    double AREA_BOTTOM;


    //timer - check how long robots are being stopped
    time_t timer_init_count;
    time_t timer_final_count;

    //ofstream on external log file
    std::ofstream log_file;
    long unsigned int log_line_counter=0;
};

void Supervisor::open_log_file()
{
    std::time_t t = time(0);
    struct tm * now = localtime(&t);
    char buffer [80];

    char *dir = get_current_dir_name();
    std::string dir_str(dir);

    if (IsPathExist(dir_str + "/pf_logs"))     //check if the folder exists
    {
        strftime (buffer,80,"/pf_logs/%Y_%m_%d_%H-%M_logfile.txt",now);
    } else {
        system(("mkdir " + (dir_str + "/pf_logs")).c_str());
        strftime (buffer,80,"/pf_logs/%Y_%m_%d_%H-%M_logfile.txt",now);
    }

    std::cout<<"file name :: "<<dir_str + buffer<<std::endl;
    this->log_file.open(dir_str + buffer,std::ofstream::app);
}

void Supervisor::write_log_file(std::string text)
{
    if (this->log_file.is_open())
    {
        this->log_file << text;
    }
}

void Supervisor::close_log_file()
{
    std::cout<<"Log file is being closed"<<std::endl;
    this->log_file.close();
}


void Supervisor::stop()
{
    std::cout << "shutting down the supervisor node." << std::endl;
    // rclcpp::Duration(0.1).sleep();
    rclcpp::shutdown();
}


// void Supervisor::srv_callback(const std::shared_ptr<std_srvs::srv::Empty::Request> request, const std::shared_ptr<std_srvs::srv::Empty::Response> response)
// {
//     std::cout << "Service called." << std::endl;
//     theta_arr = {3.14, 0.0, 0.0, 0.0};
//     std::cout << "Orientation modified." << std::endl;
// }


void Supervisor::emulate_vision()
{
    for (int i = 0; i < ROBOTS_NUM; i++)
    {
        geometry_msgs::msg::PoseStamped r;
        r.header.stamp = this->now();
        r.header.frame_id = "map";
        r.pose.position.x = x_arr[i];
        r.pose.position.y = y_arr[i];
        r.pose.position.z = 0.0;
        r.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), theta_arr[i]));

        this->fakeposePub_[i]->publish(r);

        std::cout << "------------------------------------\n";
    }
        
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Supervisor>());
    rclcpp::shutdown();
    return 0;
}
