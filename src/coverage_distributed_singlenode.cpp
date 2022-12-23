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
// SFML
#include <SFML/Graphics.hpp>
#include <SFML/OpenGL.hpp>
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
const float CONVERGENCE_TOLERANCE = 0.05;
//------------------------------------------------------------------------
const int shutdown_timer = 5;           //count how many seconds to let the robots stopped before shutting down the node


class Controller : public rclcpp::Node
{

public:
    Controller() : Node("coverage_distributed_singlenode")
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
        this->declare_parameter<double>("PT_X", -10.0);
        this->get_parameter("PT_X", PT_X);
        this->declare_parameter<double>("PT_Y", 10.0);
        this->get_parameter("PT_Y", PT_Y);
        this->declare_parameter<double>("VAR", 2.0);
        this->get_parameter("VAR", VAR);

        //view graphical voronoi rapresentation - bool
        this->declare_parameter<bool>("GRAPHICS_ON", false);
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
        velPub_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtlebot" + std::to_string(Robot_ID) + "/cmd_vel", 1);
        neighposeSub_ = this->create_subscription<turtlebot3_msgs::msg::Num>("/supervisor/turtlebot" + std::to_string(Robot_ID) + "/pose", 1, std::bind(&Controller::neighposeCallback, this, _1));

        joySub_ = this->create_subscription<geometry_msgs::msg::Twist>("/joy_vel", 1, std::bind(&Controller::joy_callback, this, _1));
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
        if (GRAPHICS_ON)
        {
            app_gui.reset(new Graphics{AREA_SIZE_x, AREA_SIZE_y, AREA_LEFT, AREA_BOTTOM, VAR});
        }
        //---------------------------------------------------------------------------------------------------------------
    }
    ~Controller()
    {
        std::cout<<"DESTROYER HAS BEEN CALLED"<<std::endl;
    }

    //void stop(int signum);
    void stop();
    void test_print();
    void neighposeCallback(const turtlebot3_msgs::msg::Num::SharedPtr msg);
    void joy_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    Eigen::VectorXd Matrix_row_sum(Eigen::MatrixXd x);
    Eigen::MatrixXd Diag_Matrix(Eigen::VectorXd V);
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
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velPub_;
    rclcpp::Subscription<turtlebot3_msgs::msg::Num>::SharedPtr neighposeSub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr joySub_;
    rclcpp::TimerBase::SharedPtr timer_;
    //-----------------------------------------------------------------------------------

    //Rendering with SFML
    //------------------------------ graphics window -------------------------------------
    std::unique_ptr<Graphics> app_gui;
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

void Controller::test_print()
{
    std::cout<<"ENTERED"<<std::endl;
}

void Controller::stop()
{
    //if (signum == SIGINT || signum == SIGKILL || signum ==  SIGQUIT || signum == SIGTERM)
    RCLCPP_INFO_STREAM(this->get_logger(), "shutting down the controller, stopping the robots, closing the graphics window");

    this->timer_->cancel();
    rclcpp::sleep_for(100000000ns);

    geometry_msgs::msg::Twist vel_msg;
    for (int i = 0; i < 100; ++i)
    {
        this->velPub_->publish(vel_msg);
    }

    RCLCPP_INFO_STREAM(this->get_logger(), "controller has been closed and robots have been stopped");
    rclcpp::sleep_for(100000000ns);
}

void Controller::neighposeCallback(const turtlebot3_msgs::msg::Num::SharedPtr msg)
{
    pose_x = Eigen::VectorXd::Zero(ROBOTS_NUM);
    pose_y = Eigen::VectorXd::Zero(ROBOTS_NUM);
    pose_theta = Eigen::VectorXd::Zero(ROBOTS_NUM);
    for (unsigned int i = 0; i < msg->locations.size(); ++i)
    {
        auto x = msg->locations[i].pose.position.x;
        auto y = msg->locations[i].pose.position.y;

        auto theta = msg->locations[i].pose.orientation.z;

        std::cout<<"robot -"<<i<<": "<<x<<", "<<y<<", "<<theta<<std::endl;

        this->pose_x(i) = msg->locations[i].pose.position.x;
        this->pose_y(i) = msg->locations[i].pose.position.y;
        this->pose_theta(i) = msg->locations[i].pose.orientation.z;
    }
}

void Controller::joy_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    this->vel_linear_x = msg->linear.x;
    this->vel_angular_z = msg->angular.z;
}

Eigen::VectorXd Controller::Matrix_row_sum(Eigen::MatrixXd X)
{
    Eigen::VectorXd vect(X.rows());
    auto temp = X.rowwise().sum();
    vect << temp;
    
    return vect;
}

Eigen::MatrixXd Controller::Diag_Matrix(Eigen::VectorXd V)
{
    Eigen::MatrixXd M(V.size(), V.size());
    for (int i = 0; i < V.size(); ++i)
    {
        for (int j = 0; j < V.size(); ++j)
        {
            if (i == j)
            {
                M(i,j) = V(i);
            }
            else{

                M(i,j) = 0;
            }
        }
    }
    return M;
}

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//-------------------------------Rendering functions - for SFML graphical visualization-----------------------------------------
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void Controller::check_window_event(){
    sf::Event event;
    if ((GRAPHICS_ON) && (this->app_gui->isOpen() && this->app_gui->window->pollEvent(event)))
    {
        if (event.type == sf::Event::Closed){
            this->app_gui->close();
        }
    }
}

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

geometry_msgs::msg::Twist Controller::Diff_drive_compute_vel(double vel_x, double vel_y, double alfa){
    //-------------------------------------------------------------------------------------------------------
    //Compute velocities commands for the robot: differential drive control, for UAVs this is not necessary
    //-------------------------------------------------------------------------------------------------------

    geometry_msgs::msg::Twist vel_msg;
    //double alfa = (this->pose_theta(i));
    double v=0, w=0;

    v = cos(alfa) * vel_x + sin(alfa) * vel_y;
    w = -(1 / b) * sin(alfa) * vel_x + (1 / b) * cos(alfa) * vel_y;

    if (abs(v) <= MAX_LIN_VEL)
    {
        vel_msg.linear.x = v;
    }
    else {
        if (v >= 0)
        {
            vel_msg.linear.x = MAX_LIN_VEL;
        } else {
            vel_msg.linear.x = -MAX_LIN_VEL;
        }
    }

    if (abs(w) <= MAX_ANG_VEL)
    {
        vel_msg.angular.z = w;
    }
    else{
        if (w >= 0)
        {
            vel_msg.angular.z = MAX_ANG_VEL;
        } else {
            vel_msg.angular.z = -MAX_ANG_VEL;
        }
    }
    return vel_msg;
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
        if ((this->pose_x[i] != 0.0) && (this->pose_y[i] != 0.0))
        {
            seeds.push_back({this->pose_x[i], this->pose_y[i]});
        }
    }

    //------------------------------------------------ CENTRALIZED COMPUTATION ---------------------------------------
    //only for visualization purposes

    //-----------------Voronoi--------------------
    if ((GRAPHICS_ON) && (this->app_gui->isOpen())){
        check_window_event();

        if (seeds.size() >= 1)
        {
            this->app_gui->clear();
            auto diagram = generateCentralizedDiagram(seeds, AreaBox);

            this->app_gui->drawDiagram(diagram);
            this->app_gui->drawPoints(diagram);
            this->app_gui->drawGlobalReference(sf::Color(255,255,0), sf::Color(255,255,255));
            //Display window
            this->app_gui->display();
        }
    }
    //--------------------------------------------------------------------------------------------------------------

    bool all_robots_stopped = true;
    //-------------------------------------------- DISTRIBUTED COMPUTATION -------------------------------------
    //----------------------------------------------------------------
    //COVERAGE and VORONOI DIAGRAM COMPUTATION
    //local (distributed) coverage control of the robot
    //the robot position is known in a global reference system
    //the neighbors relative positions are known
    //
    //Single integrator dynamics:
    //xdot = K*(xc - x)
    //where: xc : x centroid, x : current position
    //----------------------------------------------------------------

    if (seeds.size() >= 1)
    {
        //-----------------Voronoi--------------------
        //Rielaborazione vettore "points" globale in coordinate locali
        auto local_seeds_i = reworkPointsVector(seeds, seeds[0]);
        //Filtraggio siti esterni alla box (simula azione del sensore)
        auto flt_seeds = filterPointsVector(local_seeds_i, RangeBox);
        auto diagram = generateDecentralizedDiagram(flt_seeds, RangeBox, seeds[0], ROBOT_RANGE, AreaBox);

        if (GAUSSIAN_DISTRIBUTION)
        {
            //compute centroid -- GAUSSIAN DISTRIBUTION
            //centroid = computeGaussianCentroid(diagram, Vector2<double>{PT_X,PT_Y}, VAR);
            std::vector<double> VARs = {VAR};
            std::vector<Vector2<double>> MEANs = {{PT_X, PT_Y}};
            centroid = computePolygonCentroid(diagram, MEANs, VARs);
        } else {
            //compute centroid -- UNIFORM DISTRIBUTION
            centroid = computeCentroid(diagram);
        }
    }

    //---------------------------------------------------------------------------------------
    // from centroids compute velocity vectors according to Lloyd law
    //---------------------------------------------------------------------------------------

    if (centroid.getNorm() > CONVERGENCE_TOLERANCE)
    {
        vel_x = K_gain*(centroid.x);
        vel_y = K_gain*(centroid.y);
        all_robots_stopped = false;
    } else {
        vel_x = 0;
        vel_y = 0;
    }

    //-------------------------------------------------------------------------------------------------------
    //Compute velocities commands for the robot: differential drive control, for UAVs this is not necessary
    //-------------------------------------------------------------------------------------------------------
    auto vel_msg = this->Diff_drive_compute_vel(vel_x, vel_y, this->pose_theta[0]);
    //-------------------------------------------------------------------------------------------------------

    RCLCPP_INFO_STREAM(get_logger(), "sending cmd_vel to " << Robot_ID << ":: " << vel_msg.angular.z << ", "<<vel_msg.linear.x);

    this->velPub_->publish(vel_msg);
    //-------------------------------------------------------------------------------------------------------

    all_robots_stopped = false;
    if (all_robots_stopped == true)
    {
        time(&this->timer_final_count);
        if (this->timer_final_count - this->timer_init_count >= shutdown_timer)
        {
            //shutdown node
            std::cout<<"SHUTTING DOWN THE NODE"<<std::endl;
            this->stop();   //stop the controller
            rclcpp::shutdown();
        }
    } else {
        time(&this->timer_init_count);
    }
    //--------------------------------------------------------------------------------------------------------------
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
