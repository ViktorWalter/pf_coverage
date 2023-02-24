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
    Supervisor() : Node("vision_supervisor")
    {
        //------------------------------------------------- ROS parameters ---------------------------------------------------------
        this->declare_parameter<int>("ROBOTS_NUM", 4);
        this->get_parameter("ROBOTS_NUM", ROBOTS_NUM);

        //Range di percezione singolo robot (= metà lato box locale)
        this->declare_parameter<double>("ROBOT_RANGE", 30.0);
        this->get_parameter("ROBOT_RANGE", ROBOT_RANGE);
        
        // Field of view (degrees)
        this->declare_parameter<double>("ROBOT_FOV", 150.0);
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
            odomSub_.push_back(this->create_subscription<geometry_msgs::msg::PoseStamped>("/vrpn_client_node/turtle" + std::to_string(i) + "/pose", 1,[this, i](geometry_msgs::msg::PoseStamped::SharedPtr msg) {this->poseCallback(msg,i);}));
            // poseSub_.push_back(this->create_subscription<nav_msgs::msg::Odometry>("/turtlebot" + std::to_string(i) + "/odom", 1,[this, i](nav_msgs::msg::Odometry::SharedPtr msg) {this->poseCallback(msg,i);}));
            neighposePub_.push_back(this->create_publisher<geometry_msgs::msg::PoseArray>("supervisor/robot" + std::to_string(i) + "/pose", 1));
        }

        timer_ = this->create_wall_timer(200ms, std::bind(&Supervisor::emulate_vision, this));

        //----------------------------------------------------------- init Variables ---------------------------------------------------------
        pose_x = Eigen::VectorXd::Zero(ROBOTS_NUM);
        pose_y = Eigen::VectorXd::Zero(ROBOTS_NUM);
        pose_theta = Eigen::VectorXd::Zero(ROBOTS_NUM);
        time(&this->timer_init_count);
        time(&this->timer_final_count);

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
    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg, int id);
    void emulate_vision();

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

    //------------------------- Publishers and subscribers ------------------------------
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> poseSub_;
    std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> odomSub_;
    std::vector<rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr> neighposePub_;
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

void Supervisor::poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg, int id)
{
    pose_x(id) = msg->pose.position.x;
    pose_y(id) = msg->pose.position.y;
    tf2::Quaternion q(
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z,
        msg->pose.orientation.w);

    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    this->pose_theta(id) = yaw;

    // std::cout << "Robot " << id << " position: " << pose_x(id) << " " << pose_y(id) << " " << pose_theta(id) << std::endl;
}


void Supervisor::emulate_vision()
{
    // Fake position for non detected robots
    geometry_msgs::msg::Pose fake_pose;
    fake_pose.position.x = 100.0;
    fake_pose.position.y = 100.0;
    fake_pose.position.z = 0.0;

    for (int i = 0; i < ROBOTS_NUM; i++)
    {
        // Write position to log file
        if (SAVE_POS)
        {
            std::string text = std::to_string(i) + " " + std::to_string(this->pose_x(i)) + " " + std::to_string(this->pose_y(i)) + " " + std::to_string(this->pose_theta(i)) + "\n";
            write_log_file(text);
        }

        int count = 0;
        geometry_msgs::msg::PoseArray neighbors;
        neighbors.header.stamp = this->get_clock()->now();
        neighbors.header.frame_id = "turtlebot" + std::to_string(i) + "/base_link";

        std::cout << "------------------------------------\n";

        for (int j = 0; j < ROBOTS_NUM; j++)
        {
            if (i != j)
            {
                double dist_x = this->pose_x(j) - this->pose_x(i);
                double dist_y = this->pose_y(j) - this->pose_y(i);
                double dist = sqrt(pow(dist_x, 2) + pow(dist_y, 2));

                if (dist <= ROBOT_RANGE)
                {
                    geometry_msgs::msg::Pose msg;

                    // Distance vector is already the neighbor position expressed in local coordinates.
                    msg.position.x = dist_x * cos(this->pose_theta(i)) + dist_y * sin(this->pose_theta(i));
                    msg.position.y = -dist_x * sin(this->pose_theta(i)) + dist_y * cos(this->pose_theta(i));
                    msg.position.z = 0.0;
                    msg.orientation.w = 1.0;

                    // Filter robots outside FOV
                    double angle = abs(atan2(msg.position.y, msg.position.x));
                    if (angle <= 0.5*ROBOT_FOV_rad && msg.position.x >= 0.0)
                        {
                            std::cout << "Robot " << i << " sees robot " << j << " in " << msg.position.x << ", " << msg.position.y << std::endl;
                            neighbors.poses.push_back(msg);
                            count++;
                        }
                    else
                        {neighbors.poses.push_back(fake_pose);}
                } else {
                    // Filter robots outside range
                    neighbors.poses.push_back(fake_pose);
                }
            }
            else
            {   
                // Fake pose in self position
                neighbors.poses.push_back(fake_pose);
            }
        }

        std::cout << "Robot " << i << " sees " << count << " robots" << std::endl;
        this->neighposePub_[i]->publish(neighbors);

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
