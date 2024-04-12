// STL
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
// SFML
// #include <SFML/Graphics.hpp>
// #include <SFML/OpenGL.hpp>
// My includes
#include "FortuneAlgorithm.h"
#include "Voronoi.h"
#include "Diagram.h"
#include "Graphics.h"
#include <SFML/Window/Mouse.hpp>
// ROS includes
#include "ros/ros.h"
// #include "std_msgs/msg/string.hpp"
// #include "geometry_msgs/msg/point.hpp"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/Range.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

// MRS imports
#include <mrs_msgs/VelocityReferenceStamped.h>
#include <mrs_msgs/PoseWithCovarianceArrayStamped.h>

// Custom libraries
#include "particle_filter/particle_filter.h"
#include <vision_control/VisionController.h>
#include <hqp/Hqp.h>

#define M_PI 3.14159265358979323846 /*pi*/

using namespace std::chrono_literals;
using std::placeholders::_1;

// Robots parameters ------------------------------------------------------
// double SAFETY_DIST = 4.0;
const double MAX_LIN_VEL = 0.5; // set to turtlebot max velocities
// const double MAX_ANG_VEL = 2*M_PI*MAX_LIN_VEL/SAFETY_DIST;
const double MAX_ANG_VEL = 0.1;
const double b = 0.025; // for differential drive control (only if we are moving a differential drive robot (e.g. turtlebot))
//------------------------------------------------------------------------
const bool centralized_centroids = false; // compute centroids using centralized computed voronoi diagram
const float CONVERGENCE_TOLERANCE = 0.1;
//------------------------------------------------------------------------
const int shutdown_timer = 15; // count how many seconds to let the robots stopped before shutting down the node
//------------------------------------------------------------------------
sig_atomic_t volatile node_shutdown_request = 0; // signal manually generated when ctrl+c is pressed
//------------------------------------------------------------------------

bool IsPathExist(const std::string &s)
{
    struct stat buffer;
    return (stat(s.c_str(), &buffer) == 0);
}

class Controller
{
    // 2.0944
public:
    Controller() : nh_priv_("~"), vision_controller(), hqp_solver()
    {
        //------------------------------------------------- ROS parameters ---------------------------------------------------------
        this->nh_priv_.getParam("ROBOTS_NUM", ROBOTS_NUM);

        // ID of the controlled robot
        this->nh_priv_.getParam("ROBOT_ID", ROBOT_ID);

        // Operating mode: 0 = coverage, 1 = milling
        this->nh_priv_.getParam("MODE", MODE);

        // Range di percezione singolo robot (= metà lato box locale)
        this->nh_priv_.getParam("ROBOT_RANGE", ROBOT_RANGE);
        this->nh_priv_.getParam("ROBOT_FOV", ROBOT_FOV);
        this->nh_priv_.getParam("SAFETY_DIST", SAFETY_DIST);

        // view graphical voronoi rapresentation - bool
        this->nh_priv_.getParam("GRAPHICS_ON", GRAPHICS_ON);

        // Area parameter
        this->nh_priv_.getParam("AREA_SIZE_x", AREA_SIZE_x);
        this->nh_priv_.getParam("AREA_SIZE_y", AREA_SIZE_y);
        this->nh_priv_.getParam("AREA_LEFT", AREA_LEFT);
        this->nh_priv_.getParam("AREA_BOTTOM", AREA_BOTTOM);

        this->nh_priv_.getParam("GOAL_X", GOAL_X);
        this->nh_priv_.getParam("GOAL_Y", GOAL_Y);

        this->nh_priv_.getParam("GAUSS_X", GAUSS_X);
        this->nh_priv_.getParam("GAUSS_Y", GAUSS_Y);

        this->nh_priv_.getParam("SAVE_LOGS", SAVE_LOGS);
        this->nh_priv_.getParam("SAVE_CPU_TIME", SAVE_CPU_TIME);

        //--------------------------------------------------- Subscribers and Publishers ----------------------------------------------------
        for (int i = 0; i < ROBOTS_NUM; i++)
        {
            realposeSub_.push_back(nh_.subscribe<nav_msgs::Odometry>("/uav" + std::to_string(i) + "/estimation_manager/odom_main", 1, std::bind(&Controller::realposeCallback, this, std::placeholders::_1, i)));
        }

        odomSub_ = nh_.subscribe<nav_msgs::Odometry>("/uav" + std::to_string(ROBOT_ID) + "/estimation_manager/gps_garmin/odom/local", 1, std::bind(&Controller::odomCallback, this, std::placeholders::_1));
        globalOdomSub_ = nh_.subscribe<nav_msgs::Odometry>("/uav" + std::to_string(ROBOT_ID) + "/estimation_manager/gps_garmin/odom", 1, std::bind(&Controller::globalOdomCallback, this, std::placeholders::_1));
        neighSub_ = nh_.subscribe<mrs_msgs::PoseWithCovarianceArrayStamped>("/uav" + std::to_string(ROBOT_ID) + "/fake_vision", 1, std::bind(&Controller::neighCallback, this, std::placeholders::_1));
        joySub_ = nh_.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, std::bind(&Controller::vel_callback, this, std::placeholders::_1));
        velPub_.push_back(nh_.advertise<mrs_msgs::VelocityReferenceStamped>("/uav" + std::to_string(ROBOT_ID) + "/control_manager/velocity_reference", 1));
        timer_ = nh_.createTimer(ros::Duration(0.1), std::bind(&Controller::loop, this));

        // RViz visualization
        rangePub_ = nh_.advertise<sensor_msgs::Range>("/uav" + std::to_string(ROBOT_ID) + "/fov", 1);
        covPub_ = nh_.advertise<visualization_msgs::MarkerArray>("/uav" + std::to_string(ROBOT_ID) + "/covariances", 1);

        // rclcpp::on_shutdown(std::bind(&Controller::stop,this));

        //----------------------------------------------------------- init Variables ---------------------------------------------------------
        pose_x = Eigen::VectorXd::Zero(ROBOTS_NUM);
        pose_y = Eigen::VectorXd::Zero(ROBOTS_NUM);
        pose_theta = Eigen::VectorXd::Zero(ROBOTS_NUM);
        p_j.resize(3, ROBOTS_NUM); // matrix with global position of neighbors on each column
        p_j_i.resize(3, ROBOTS_NUM - 1);
        p_j_est.resize(3, ROBOTS_NUM - 1);
        slack.resize(4, ROBOTS_NUM - 1);
        slack_neg.resize(4, ROBOTS_NUM - 1);
        realpose_x = Eigen::VectorXd::Zero(ROBOTS_NUM);
        realpose_y = Eigen::VectorXd::Zero(ROBOTS_NUM);
        realpose_theta = Eigen::VectorXd::Zero(ROBOTS_NUM);
        GAUSSIAN_MEAN_PT.resize(2);
        GAUSSIAN_MEAN_PT << GAUSS_X, GAUSS_Y; // Gaussian mean point
        time(&this->timer_init_count);
        time(&this->timer_final_count);

        this->got_gmm = false;

        slack_max.resize(4);
        double x1m, x2m, y1m, y2m;
        double fov = ROBOT_FOV * M_PI / 180;
        x1m = AREA_SIZE_x * cos(-fov / 2 + 0.5 * M_PI);
        x2m = AREA_SIZE_x * cos(fov / 2 + 0.5 * M_PI);
        y1m = AREA_SIZE_y * sin(-fov / 2 + 0.5 * M_PI);
        y2m = AREA_SIZE_y * sin(fov / 2 + 0.5 * M_PI);
        slack_max(0) = tan(fov / 2) * x1m + y1m;
        slack_max(1) = tan(fov / 2) * x2m - y2m;
        // slack_max(2) = -pow(SAFETY_DIST,2);
        slack_max(2) = 0.0; // safety constraint is always hard
        slack_max(3) = -(pow(AREA_SIZE_x, 2) + pow(AREA_SIZE_y, 2)) + pow(ROBOT_RANGE, 2);
        slack_max = slack_max.cwiseAbs();
        std::cout << "Number of robots : " << ROBOTS_NUM << std::endl;
        std::cout << "============ SLACK SATURATION VALUES =================\n"
                  << slack_max.transpose() << "\n==========================\n";

        vision_controller.init(0.75*fov, SAFETY_DIST, ROBOT_RANGE, ROBOTS_NUM - 1);
        vision_controller.setVerbose(false);
        vision_controller.setVelBounds(-MAX_LIN_VEL, MAX_LIN_VEL, -MAX_ANG_VEL, MAX_ANG_VEL);
        vision_controller.setGamma(1.0, 1.0);
        // safety_controller.setVerbose(false);

        hqp_solver.init(0.75*fov, SAFETY_DIST, ROBOT_RANGE, ROBOTS_NUM - 1);
        hqp_solver.setVerbose(false);

        parts = std::round(PARTICLES_NUM / (ROBOTS_NUM - 1));
        for (int i = 0; i < ROBOTS_NUM - 1; i++)
        {
            // if (i == 9)
            // {
            //     ParticleFilter *filter = new ParticleFilter(parts, -10*Eigen::Vector3d::Ones(), 10*Eigen::Vector3d::Ones());
            //     filters.push_back(filter);
            // } else if (i == 6)
            // {
            //     ParticleFilter *filter = new ParticleFilter(parts, 10*Eigen::Vector3d::Ones(), 10*Eigen::Vector3d::Ones());
            //     filters.push_back(filter);
            // } else
            // {
            //     ParticleFilter *filter = new ParticleFilter(parts, Eigen::Vector3d::Zero(), 10*Eigen::Vector3d::Ones());
            //     filters.push_back(filter);
            // }

            ParticleFilter *filter = new ParticleFilter(parts, Eigen::Vector3d::Zero(), 10 * Eigen::Vector3d::Ones());
            filters.push_back(filter);
        }

        for (int i = 0; i < ROBOTS_NUM; i++)
        {
            covariances.push_back(Eigen::Matrix2d::Zero());
        }

        // Initialize visualization msgs
        fov_msg.header.frame_id = "uav" + std::to_string(ROBOT_ID) + "/fcu";
        fov_msg.field_of_view = ROBOT_FOV * M_PI / 180.0;
        fov_msg.min_range = 0.0;
        fov_msg.max_range = 2*ROBOT_RANGE;
        fov_msg.range = ROBOT_RANGE;

        // this->got_gmm = false;
        std::cout << "Hi! I'm robot number " << ROBOT_ID << std::endl;

        justStarted.resize(ROBOTS_NUM, true);

        // std::cout << "Initial set of mean points for GMM: " << std::endl;
        // for (int i = 0; i < gmm_.getMeans().size(); i++)
        // {
        //     std::cout << gmm_.getMeans()[i].transpose() << std::endl;
        // }

        //------------------------------------------------------------------------------------------------------------------------------------

        // ----------------------------------------------- Graphics window -------------------------------------------------
        if (GRAPHICS_ON)
        {
            app_gui.reset(new Graphics{AREA_SIZE_x, AREA_SIZE_y, AREA_LEFT, AREA_BOTTOM, 2.0});
        }

        if (SAVE_LOGS || SAVE_CPU_TIME)
        {
            // for (int i = 0; i < ROBOTS_NUM - 1; i++)
            // {
            //     std::ofstream log_f;
            //     this->log_files.push_back(log_f);
            // }

            // fill vector with log files
            // this->log_files.resize(ROBOTS_NUM-1);
            // std::ofstream log_f;
            // for (int i = 0; i < ROBOTS_NUM-1; i++)
            // {
            //     this->log_files[i] = std::move(log_f);
            // }

            // for (int i = 0; i < ROBOTS_NUM-1; i++)
            // {
            //     open_log_file();
            // }
            this->open_log_file();
        }

        // std::cout << "Input : " << filter->getInput().transpose() << std::endl;
    }
    ~Controller()
    {
        if ((GRAPHICS_ON || SAVE_CPU_TIME) && (this->app_gui->isOpen()))
        {
            this->app_gui->close();
        }
        std::cout << "DESTROYER HAS BEEN CALLED" << std::endl;

        if (SAVE_LOGS || SAVE_CPU_TIME)
        {
            // for (int i = 0; i < ROBOTS_NUM-1; i++)
            // {
            //     close_log_file();
            //     std::cout << "LOG FILE HAS BEEN CLOSED" << std::endl;
            // }
            this->close_log_file();
        }
    }

    // void stop(int signum);
    void stop();
    void test_print();
    void realposeCallback(const nav_msgs::Odometry::ConstPtr msg, int i);
    void odomCallback(const nav_msgs::Odometry::ConstPtr msg);
    void globalOdomCallback(const nav_msgs::Odometry::ConstPtr msg);
    void neighCallback(const mrs_msgs::PoseWithCovarianceArrayStamped::ConstPtr msg);
    void joy_callback(const geometry_msgs::Twist::ConstPtr msg);
    void vel_callback(const geometry_msgs::Twist::ConstPtr &msg);
    Eigen::VectorXd Matrix_row_sum(Eigen::MatrixXd x);
    Eigen::MatrixXd Diag_Matrix(Eigen::VectorXd V);
    void cbf_coverage();
    void loop();
    void pf_coverage();
    bool insideFOV(Eigen::VectorXd q, Eigen::VectorXd q_obs, double fov, double r_sens);
    bool is_outlier(Eigen::VectorXd q, Eigen::VectorXd mean, Eigen::MatrixXd cov_matrix, double threshold);
    bool isOut(Eigen::VectorXd sample, Eigen::VectorXd mean, Eigen::MatrixXd cov_matrix);
    Eigen::VectorXd predictVelocity(Eigen::VectorXd q, Eigen::VectorXd mean_pt);
    Eigen::VectorXd predictCentrVel(Eigen::VectorXd q, Vector2<double> mean_pt);
    Eigen::VectorXd getWheelVelocity(Eigen::VectorXd u, double alpha);
    geometry_msgs::Twist Diff_drive_compute_vel(double vel_x, double vel_y, double alfa);
    double mahalanobis_distance(Eigen::VectorXd x, Eigen::VectorXd mean, Eigen::MatrixXd cov_matrix);
    Eigen::VectorXd gradV(Eigen::VectorXd q, Eigen::VectorXd x2, Eigen::VectorXd x_goal, Eigen::MatrixXd cov, double alpha);
    Eigen::VectorXd boundVel(Eigen::VectorXd u);
    Eigen::VectorXd boundVelprop(Eigen::VectorXd u);
    Eigen::VectorXd gradient_descent(Eigen::VectorXd q, std::vector<Eigen::VectorXd> means, Eigen::VectorXd x_goal, std::vector<Eigen::MatrixXd> cov, double alpha, double alpha_grad, int iters);
    Eigen::VectorXd gradV2(Eigen::VectorXd q, std::vector<Eigen::VectorXd> means, Eigen::VectorXd x_goal, std::vector<Eigen::MatrixXd> cov, double alpha);
    double sigmoid(double x);

    int PARTICLES_NUM = 1000;

    // open write and close LOG file
    void open_log_file();
    void write_log_file(std::string text);
    void close_log_file();

private:
    int ROBOTS_NUM = 2;
    double ROBOT_RANGE = 10.0;
    int ROBOT_ID = 0;
    double ROBOT_FOV = 160.0;
    double SAFETY_DIST = 4.0;
    int MODE = 0;
    double GOAL_X = 10.0;
    double GOAL_Y = 10.0;
    double GAUSS_X = 7.5;
    double GAUSS_Y = 7.5;
    double dist_lim = 1.0; // mahalanobis distance limit (in terms of standard deviations)
    // int PARTICLES_NUM;
    bool got_gmm;
    double vel_linear_x, vel_angular_z;
    Eigen::VectorXd pose_x;
    Eigen::VectorXd pose_y;
    Eigen::VectorXd pose_theta;
    Eigen::MatrixXd p_j, p_j_i, p_j_est, slack, slack_neg;
    Eigen::VectorXd slack_max;
    Eigen::VectorXd realpose_x;
    Eigen::VectorXd realpose_y;
    Eigen::VectorXd realpose_theta;
    std::vector<Eigen::Matrix2d> covariances;
    std::vector<Vector2<double>> seeds_xy;
    int seeds_counter = 0;
    std::vector<std::vector<double>> corners;
    std::vector<double> position;
    // Eigen::VectorXd start;
    // Eigen::VectorXd initCovariance;
    double dt = 0.2;
    std::vector<bool> justLost;
    int parts;
    std::vector<bool> justStarted;

    //------------------------------- Ros NodeHandle ------------------------------------
    ros::NodeHandle nh_;
    ros::NodeHandle nh_priv_;
    //------------------------- Publishers and subscribers ------------------------------
    std::vector<ros::Publisher> velPub_;
    ros::Subscriber odomSub_;
    ros::Subscriber globalOdomSub_;
    ros::Subscriber neighSub_;
    ros::Subscriber joySub_;
    std::vector<ros::Subscriber> realposeSub_;
    ros::Publisher rangePub_;
    ros::Publisher covPub_; 
    ros::Timer timer_;

    sensor_msgs::Range fov_msg;

    // ------------------------------- Particle Filter ----------------------------------
    std::vector<ParticleFilter *> filters;
    // ParticleFilter filter;
    // GaussianMixtureModel gmm_;

    // ------------------------------- Safety Controller ---------------------------------
    // safety_control::SafetyController safety_controller;
    vision_control::VisionController vision_controller;
    Eigen::VectorXd h, h_tmp;
    Eigen::Vector3d ustar;
    Eigen::Vector3d target;

    // ---- HQP solver -------
    Hqp hqp_solver;

    // std::vector<Eigen::VectorXd> total_samples;

    //-----------------------------------------------------------------------------------

    // Rendering with SFML
    //------------------------------ graphics window -------------------------------------
    std::unique_ptr<Graphics> app_gui;
    //------------------------------------------------------------------------------------

    //---------------------------- Environment definition --------------------------------
    double AREA_SIZE_x = 40.0;
    double AREA_SIZE_y = 40.0;
    double AREA_LEFT = -20.0;
    double AREA_BOTTOM = -20.0;
    //------------------------------------------------------------------------------------

    //---------------------- Gaussian Density Function parameters ------------------------
    bool GAUSSIAN_DISTRIBUTION;
    double PT_X;
    double PT_Y;
    double VAR;
    Eigen::VectorXd GAUSSIAN_MEAN_PT;

    //------------------------------------------------------------------------------------

    // graphical view - ON/OFF
    bool GRAPHICS_ON = true;

    bool SAVE_LOGS = false;
    bool SAVE_CPU_TIME = false;

    // timer - check how long robots are being stopped
    time_t timer_init_count;
    time_t timer_final_count;

    std::ofstream log_file;
    std::vector<std::ofstream> log_files;

    int counter = 0;

    bool received = false;
};

void Controller::open_log_file()
{
    std::time_t t = time(0);
    struct tm *now = localtime(&t);
    char buffer[80];

    char *dir = get_current_dir_name();
    std::string dir_str(dir);

    std::cout << "Directory: " << dir_str << std::endl;
    std::string dir_path = dir_str + "/pf_logs-" + std::to_string(ROBOT_ID); //+"-"+std::to_string(id);
    if (IsPathExist(dir_path))                                               // check if the folder exists
    {
        strftime(buffer, 80, "/%Y_%m_%d_%H-%M_logfile.txt", now);
    }
    else
    {
        system(("mkdir " + dir_path).c_str()); //+"-"+std::to_string(id)).c_str());
        strftime(buffer, 80, "/%Y_%m_%d_%H-%M_logfile.txt", now);
    }

    std::cout << "file name :: " << dir_path + buffer << std::endl;
    // this->log_file.open(dir_path + buffer,std::ofstream::app);
    this->log_file.open(dir_path + buffer, std::ofstream::app);
    // this->log_file << "Robot " << ROBOT_ID << " log file\n";
}

void Controller::write_log_file(std::string text)
{
    if (this->log_file.is_open())
    {
        this->log_file << text;
        this->log_file.flush();
    }
}

void Controller::close_log_file()
{
    std::cout << "Log file is being closed" << std::endl;
    this->log_file.close();
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

    double dist = sqrt(pow(q_obs(0) - q(0), 2) + pow(q_obs(1) - q(1), 2));
    double th = q(2);
    double th_obs = atan2(q_obs(1) - q(1), q_obs(0) - q(0));
    double th_rel = th_obs - th;

    double fov_rad = fov * M_PI / 180;

    if (th_rel > M_PI)
        th_rel -= 2 * M_PI;
    else if (th_rel < -M_PI)
        th_rel += 2 * M_PI;

    if (th_rel > 0.5 * fov_rad || th_rel < -0.5 * fov_rad || dist > r_sens)
        return false;
    else
        return true;
}

// function to get wheel velocities of differential drive robot
Eigen::VectorXd Controller::getWheelVelocity(Eigen::VectorXd u, double alpha)
{
    double r = 0.033; // turtlebot3 burger wheel radius
    double d = 0.16;  // turtlebot3 burger distance between wheels

    double vx = u(0);
    double vy = u(1);

    geometry_msgs::Twist vel_msg = Diff_drive_compute_vel(vx, vy, alpha);
    double v_lin = vel_msg.linear.x;
    double v_ang = vel_msg.angular.z;

    // std::cout << "v_lin: " << v_lin << ", v_ang: " << v_ang << std::endl;

    double wl = v_lin / r - 0.5 * d * v_ang / r;
    double wr = v_lin / r + 0.5 * d * v_ang / r;

    Eigen::VectorXd u_final(2);
    u_final << wl, wr;

    return u_final;
}



void Controller::loop()
{
    auto timerstart = std::chrono::high_resolution_clock::now();
    Eigen::Vector3d processCovariance = 0.25 * Eigen::Vector3d::Ones();
    Eigen::Vector3d robot; // controlled robot's global position
    robot << this->pose_x(ROBOT_ID), this->pose_y(ROBOT_ID), this->pose_theta(ROBOT_ID);
    Eigen::MatrixXd samples(3, parts); // particles' global position
    std::vector<Eigen::VectorXd> samples_vec;
    std::vector<bool> detected(ROBOTS_NUM - 1, false); // vector of detected robots
    int detected_counter = 0;                          // number of detected robots
    std::vector<Eigen::VectorXd> detections;
    std::vector<Eigen::VectorXd> obs;
    std::vector<Eigen::MatrixXd> cov;
    std::vector<double> ws;
    slack.setOnes();
    slack = 100000 * slack;
    slack.row(2).setZero();
    Eigen::Matrix2d cov_rel;

    slack_neg.setZero();
    // slack.setZero();

    // Define rotation matrix
    Eigen::Matrix<double, 3, 3> R_w_i;
    R_w_i << cos(this->pose_theta(ROBOT_ID)), -sin(this->pose_theta(ROBOT_ID)), 0,
        sin(this->pose_theta(ROBOT_ID)), cos(this->pose_theta(ROBOT_ID)), 0,
        0, 0, 1;

    // std::cout << "p_j: " << p_j << std::endl;
    /* 
    if (!received)
    {
        return;
    }
    */

    std::cout << "I'm robot " << ROBOT_ID << " in position : " << this->pose_x(ROBOT_ID) << ", " << this->pose_y(ROBOT_ID) << std::endl;

    if (SAVE_LOGS)
    {
        std::string txt;
        // txt = txt + std::to_string(this->pose_x(ROBOT_ID)) + " " + std::to_string(this->pose_y(ROBOT_ID)) + "\n";
        for (int i = 0; i < ROBOTS_NUM; i++)
        {
            txt = txt + std::to_string(this->realpose_x(i) - GAUSS_X) + " " + std::to_string(this->realpose_y(i) - GAUSS_Y) + "\n";
        }
        this->write_log_file(txt);
    }

    for (int j = 0; j < ROBOTS_NUM; j++)
    {
        double c = j; // robot's id variable
        if (j > ROBOT_ID)
        {
            c = j - 1;
        }
        if (j != ROBOT_ID)
        {
            // Check if the robot is detected
            if (this->pose_x(j) < 100.0 && this->pose_y(j) < 100.0)
            {
                std::cout << "Robot " << j << " detected in " << this->pose_x(j) << ", " << this->pose_y(j) << std::endl;
                detected[c] = true;
                detected_counter++;
                if (this->pose_x(j) == 0.0 && this->pose_y(j) == 0.0)
                {
                    // std::cout << "Error in robot " << j << " initialization. Skipping..." << std::endl;
                    return;
                }

                if (!this->got_gmm)
                {
                    obs.push_back(p_j.col(j).head(2));
                    cov.push_back(Eigen::Matrix2d::Identity());
                    ws.push_back(1.0 / (ROBOTS_NUM - 1));
                }

                // Case 1: robot detected --- define particles with small covariance around the detected position
                // Estimate velocity of the lost robot for coverage
                Eigen::VectorXd q_est = filters[c]->getMean();

                // std::cout << "Estimated state: " << q_est.transpose() << std::endl;
                Eigen::Vector2d u_ax = predictVelocity(q_est, robot); // get linear velocity [vx, vy] moving towards me
                // std::cout << "Linear velocity: " << u_ax.transpose() << std::endl;
                // Eigen::VectorXd u_est(2);
                // u_est = getWheelVelocity(u_ax, q_est(2));                               // get wheel velocity [v_l, v_r] to reach the mean point
                // std::cout << "wheels velocity: " << u_est.transpose() << std::endl;
                // Eigen::Vector3d processCovariance = 0.5*Eigen::Vector3d::Ones();
                filters[c]->setProcessCovariance(processCovariance);
                filters[c]->predictUAV(0.1 * u_ax, dt);
                // std::cout << "Prediction completed" << std::endl;

                // std::cout << "sigma x: " << covariances[j](0,0) << ", sigma y: " << covariances[j](1,1) << std::endl;
                // filters[c]->updateWeights2d(p_j.col(j), covariances[j](0,0), covariances[j](1,1));
                // filters[c]->updateWeightsWithCovariance(p_j.col(j), covariances[j]);
                filters[c]->updateWeights(p_j.col(j), 0.5);
            }
            else
            {
                // Case 2: robot not detected
                std::cout << "Robot " << j << " not detected. Relative position: " << p_j.col(j).transpose()  << std::endl;
                // Estimate velocity of the lost robot for coverage
                Eigen::VectorXd q_est = filters[c]->getMean();
                // std::cout << "Estimated state: " << q_est.transpose() << std::endl;
                Eigen::Vector2d u_ax = predictVelocity(q_est, robot); // get linear velocity [vx, vy] moving towards me
                // std::cout << "Linear velocity: " << u_ax.transpose() << std::endl;
                // Eigen::VectorXd u_est(2);
                // u_est = getWheelVelocity(u_ax, q_est(2));                               // get wheel velocity [v_l, v_r] to reach the mean point
                // std::cout << "wheels velocity: " << u_est.transpose() << std::endl;
                filters[c]->setProcessCovariance(processCovariance);
                filters[c]->predictUAV(0.5 * u_ax, dt);
                // std::cout << "Prediction completed" << std::endl;

                // Get particles in required format
                Eigen::MatrixXd particles = filters[c]->getParticles();
                // std::vector<Eigen::VectorXd> samples;
                /*---- PARTICLES DELETION --------- */
                Eigen::VectorXd weights = filters[c]->getWeights();
                double w_min =  weights.minCoeff();
                std::cout << "*********\nMinimum weight: " << w_min << "\n************\n";
                for (int i = 0; i < particles.cols(); i++)
                {
                    Eigen::VectorXd sample = particles.col(i);
                    if (insideFOV(robot, sample, ROBOT_FOV, ROBOT_RANGE))
                    {
                        weights(i) = w_min/10;
                    }
                    // samples.push_back(sample);
                }
                // std::cout << "Particles converted to required format" << std::endl;
                filters[c]->setWeights(weights); // update weights
                /*-------------------- PARTICLES DELETION ---------------*/
            }

            filters[c]->resample();
        }
    }

    if (!this->got_gmm)
    {
        // gmm_.setMeans(obs);
        // gmm_.setCovariances(cov);
        // gmm_.setWeights(ws);
        // gmm_.check();
        // filter.setParticles(samples_vec);

        this->got_gmm = true;
    }

    if ((GRAPHICS_ON) && (this->app_gui->isOpen()))
    {
        this->app_gui->clear();
        this->app_gui->drawGlobalReference(sf::Color(255, 255, 0), sf::Color(255, 255, 255));
        this->app_gui->drawFOV(robot, ROBOT_FOV, ROBOT_RANGE);
        // Vector2<double> goal = {GOAL_X, GOAL_Y};
        // this->app_gui->drawPoint(goal, sf::Color(255,128,0));
        for (int i = 0; i < ROBOTS_NUM; i++)
        {
            auto color = sf::Color(0, 255, 0); // default color for other robots: green
            if (i == ROBOT_ID)
            {
                color = sf::Color(255, 0, 0);
            } // controlled robot color: red
            Vector2<double> n;
            n.x = this->realpose_x(i);
            n.y = this->realpose_y(i);
            // this->app_gui->drawPoint(n, color);
            Vector2<double> nn;
            auto c2 = sf::Color(255, 102, 255);
            nn.x = p_j(0, i);
            nn.y = p_j(1, i);
            // this->app_gui->drawPoint(nn, c2);
            // this->app_gui->drawID(n, i, color);
        }

        Vector2<double> me = {p_j(0, ROBOT_ID), p_j(1, ROBOT_ID)};
        this->app_gui->drawPoint(me, sf::Color(255, 0, 0));
        // this->app_gui->drawParticles(samples_mat);

        for (int i = 0; i < filters.size(); i++)
        {
            this->app_gui->drawParticles(filters[i]->getParticles());
        }
    }

    std::vector<double> distances(ROBOTS_NUM - 1);
    visualization_msgs::MarkerArray markers_msg;
    for (int i = 0; i < ROBOTS_NUM - 1; i++)
    {
        Eigen::VectorXd mean = filters[i]->getMean();
        // std::cout << "Neighbor " << i << " estimated position: " << mean.transpose() << std::endl;
        double dist_x = mean(0) - this->pose_x(ROBOT_ID);
        double dist_y = mean(1) - this->pose_y(ROBOT_ID);

        p_j_est(0, i) = dist_x * cos(this->pose_theta(ROBOT_ID)) + dist_y * sin(this->pose_theta(ROBOT_ID));
        p_j_est(1, i) = -dist_x * sin(this->pose_theta(ROBOT_ID)) + dist_y * cos(this->pose_theta(ROBOT_ID));
        p_j_est(2, i) = 0.0;
        if (this->got_gmm)
        {
            Eigen::MatrixXd cov_matrix = filters[i]->getCovariance();
            if (!isinf(cov_matrix(0, 0)))
            {
                Eigen::EigenSolver<Eigen::MatrixXd> es(cov_matrix.block<2, 2>(0, 0));
                Eigen::VectorXd eigenvalues = es.eigenvalues().real();
                // std::cout << "Eigenvalues: \n" << eigenvalues.transpose() << "\n";
                Eigen::MatrixXd eigenvectors = es.eigenvectors().real();
                // std::cout << "Eigenvectors: \n" << eigenvectors.transpose() << "\n";

                // Convert covariance matrix to local frame
                cov_rel = R_w_i.block<2, 2>(0, 0).transpose() * cov_matrix.block<2, 2>(0, 0) * R_w_i.block<2, 2>(0, 0);

                // Write mean point (relative) and covariance matrix in log file
                // if (SAVE_LOGS)
                // {
                //     std::string pf_txt;
                //     pf_txt = pf_txt + std::to_string(p_j_est(0,i)) + " " + std::to_string(p_j_est(1,i)) + " " + std::to_string(cov_rel(0,0)) + " " + std::to_string(cov_rel(0,1)) + " " + std::to_string(cov_rel(1,0)) + " " + std::to_string(cov_rel(1,1)) + "\n";
                //     this->write_log_file(pf_txt, i);
                // }

                // s = 4.605 for 90% confidence interval
                // s = 5.991 for 95% confidence interval
                // s = 9.210 for 99% confidence interval
                double s = 5.991;
                double a = sqrt(s * eigenvalues(0)); // major axis
                double b = sqrt(s * eigenvalues(1)); // minor axis

                // a could be smaller than b, so swap them
                if (a < b)
                {
                    double temp = a;
                    a = b;
                    b = temp;
                }

                int m = 0; // higher eigenvalue index
                int l = 1; // lower eigenvalue index
                if (eigenvalues(1) > eigenvalues(0))
                {
                    m = 1;
                    l = 0;
                }

                double theta = atan2(eigenvectors(1, m), eigenvectors(0, m)); // angle of the major axis wrt positive x-asis (ccw rotation)
                if (theta < 0.0)
                {
                    theta += M_PI;
                } // angle in [0, 2pi
                if ((GRAPHICS_ON) && (this->app_gui->isOpen()))
                {
                    this->app_gui->drawEllipse(mean, a, b, theta);
                }

                // -------- RViz visaulization
                visualization_msgs::Marker mrk;
                mrk.id = i;
                // mrk.header.frame_id = "map";
                mrk.header.frame_id = "uav" + std::to_string(ROBOT_ID) + "/local_origin";
                mrk.type = visualization_msgs::Marker::SPHERE;
                mrk.action = visualization_msgs::Marker::ADD;
                mrk.pose.position.x = mean(0);
                mrk.pose.position.y = mean(1);
                mrk.pose.position.z = 3.0;
                mrk.scale.x = a;
                mrk.scale.y = b;
                mrk.scale.z = 0.01;
                tf2::Quaternion q;
                q.setRPY(0, 0, theta);
                mrk.pose.orientation.x = q.x();
                mrk.pose.orientation.y = q.y();
                mrk.pose.orientation.z = q.z();
                mrk.pose.orientation.w = q.w();
                mrk.color.r = 255;
                mrk.color.g = 178;
                mrk.color.b = 102;
                mrk.color.a = 1.0;
                markers_msg.markers.push_back(mrk);
                // ---------------------


                double slope = atan2(-mean(1) + this->pose_y(ROBOT_ID), -mean(0) + this->pose_x(ROBOT_ID));
                double x_n = mean(0) + a * cos(slope - theta) * cos(theta) - b * sin(slope - theta) * sin(theta);
                double y_n = mean(1) + a * cos(slope - theta) * sin(theta) + b * sin(slope - theta) * cos(theta);
                Vector2<double> p_near = {x_n, y_n};

                double a_lim = 3.0; // width limit for major axis
                double sx = cov_matrix(0, 0);
                double sy = cov_matrix(1, 1);

                double dist = sqrt(pow(p_near.x - this->pose_x(ROBOT_ID), 2) + pow(p_near.y - this->pose_y(ROBOT_ID), 2));
                // double dist = mahalanobis_distance(robot.head(2), mean.head(2), cov_matrix.block<2,2>(0,0));
                // std::cout << "Distance: " << dist << "\n";
                if (isnan(dist))
                {
                    ROS_WARN("NaN distance calculated");
                    dist = 5.0;
                }
                else
                {
                    slack_neg.col(i).head(2) = -0.2 * slack_max.head(2) * sigmoid((a - 3 * a_lim)); // restrict angular fov by half
                }
                // slack_neg(4) = -0.5 * slack_max(4) * sigmoid(0.5 * (sy - 3*a_lim));                // restrict angular fov by half

                // Check if robot is inside ellipse
                double d = sqrt(pow(mean(0) - this->pose_x(ROBOT_ID), 2) + pow(mean(1) - this->pose_y(ROBOT_ID), 2));
                double range = sqrt(pow(mean(0) - p_near.x, 2) + pow(mean(1) - p_near.y, 2));
                if (d < range)
                {
                    distances[i] = -dist;
                }
                else
                {
                    distances[i] = dist;
                }
            }
        }
    }

    covPub_.publish(markers_msg);

    for (int i = 0; i < distances.size(); i++)
    {
        // slack.col(i) =  slack_max.cwiseProduct(sigmoid(2*(distances[i] - 2*SAFETY_DIST)) * Eigen::VectorXd::Ones(4)) + slack_neg.col(i);
        slack.col(i) = slack_max.cwiseProduct(sigmoid(2*(distances[i] - 3*dist_lim)) * Eigen::VectorXd::Ones(4)); // + slack_neg.col(i);
    }

    // slack.row(3) = 100000 * Eigen::VectorXd::Ones(ROBOTS_NUM-1);

    // Sort neighbors based on distance and update number of particles
    /*
    std::vector<std::pair<double, int>> dist_index;
    for (int i = 0; i < distances.size(); i++)
    {
        dist_index.push_back(std::make_pair(distances[i], i));
    }
    std::sort(dist_index.begin(), dist_index.end(), [](const auto& a, const auto& b){
        return a.first < b.first;
    });

    int elems = std::round((ROBOTS_NUM-1)/3);            // number of elements in each partition (1/3 of total)
    for (int i = 0; i < elems; i++)
    {
        int id = dist_index[i].second;
        int p_num = std::round(0.5 * PARTICLES_NUM / elems);
        if (p_num != filters[id]->getParticlesNumber())
        {
            filters[id]->updateParticlesNumber(p_num);
        }
    }
    for (int i = elems; i < 2*elems; i++)
    {
        int id = dist_index[i].second;
        int p_num = std::round(0.3 * PARTICLES_NUM / elems);
        if (p_num != filters[id]->getParticlesNumber())
        {
            filters[id]->updateParticlesNumber(p_num);
        }

    }
    for (int i = 2*elems; i < ROBOTS_NUM-1; i++)
    {
        int id = dist_index[i].second;
        int p_num = std::round(0.2 * PARTICLES_NUM / elems);
        if (p_num != filters[id]->getParticlesNumber())
        {
            filters[id]->updateParticlesNumber(p_num);
        }
    }
    */

    /*
    std::vector<Vector2<double>> mean_points_vec2;
    for (int j = 0; j < ROBOTS_NUM; j++)
    {
        if (j != ROBOT_ID)
        {
            Vector2<double> mp = {filters[j]->getMean()(0), filters[j]->getMean()(1)};
            mean_points_vec2.push_back(mp);
            this->app_gui->drawPoint(mp, sf::Color(0,0,255));
        }
    }
    this->app_gui->display();
    return;
    */

    // std::cout << "Slack variables: \n" << slack << "\n";

    // --------------------- HQP SLACK VARIABLES CALCULATION ----------------------------------------
    std::vector<Eigen::VectorXd> p_j_ordered;
    // p_j_ordered.resize(2, ROBOTS_NUM-1);

    std::vector<std::pair<Eigen::Vector3d, double>> pos_pairs;
    for (int i = 0; i < distances.size(); i++)
    {
        // std::cout << p_j_est.col(i).transpose() << "\n";
        pos_pairs.push_back(std::make_pair(p_j_est.col(i), distances[i]));
    }

    // sort the vector based on distances
    std::sort(pos_pairs.begin(), pos_pairs.end(), [](const auto &a, const auto &b)
              { return a.second < b.second; });

    std::vector<Eigen::VectorXd> slack_ordered;
    // slack_ordered.resize(2, ROBOTS_NUM-1);

    std::vector<std::pair<Eigen::VectorXd, double>> slack_pairs;
    for (int i = 0; i < distances.size(); i++)
    {
        slack_pairs.push_back(std::make_pair(slack.col(i), distances[i]));
    }

    // sort the vector based on distances
    std::sort(slack_pairs.begin(), slack_pairs.end(), [](const auto &a, const auto &b)
              { return a.second < b.second; });

    for (int i = 0; i < distances.size(); i++)
    {
        p_j_ordered.push_back(pos_pairs[i].first);
        slack_ordered.push_back(slack_pairs[i].first);
    }

    // calculate hqp slack variable
    Eigen::VectorXd hqp_slack = hqp_solver.solve(p_j_ordered);
    // std::cout << "HQP slack: \n" << hqp_slack.transpose() << "\n";

    // backwards conversion to Eigen::MatrixXd
    Eigen::MatrixXd p_j_mat, slack_mat;
    p_j_mat.resize(3, ROBOTS_NUM - 1);
    slack_mat.resize(4, ROBOTS_NUM - 1);
    for (int i = 0; i < distances.size(); i++)
    {
        p_j_mat.col(i) = p_j_ordered[i];
        slack_mat.col(i) = slack_ordered[i] + hqp_slack.block<4, 1>(4 * i, 0);
    }
    //---------------------------------------------------------------------------


    // Move to the goal
    Eigen::Vector3d udes;
    double head_des = atan2(GOAL_Y-p_j(1, ROBOT_ID), GOAL_X-p_j(0, ROBOT_ID));
    double head_err = head_des - p_j(2, ROBOT_ID);
    udes(0) = 0.8 * (GOAL_X - p_j(0, ROBOT_ID));
    udes(1) = 0.8 * (GOAL_Y - p_j(1, ROBOT_ID));
    udes(2) = 0.8 * head_err;

    udes = boundVel(udes);

    // Eigen::Vector3d udes;
    // double head_des = atan2(centroid_eigen(1), centroid_eigen(0));
    // double head_err = head_des - this->pose_theta(ROBOT_ID);
    // double vx = std::max(0.0, 0.8 * centroid_local(0));
    // double vy = std::max(0.0, 0.8 * centroid_local(1));
    // double w = 0.8 * head_err;
    // Eigen::Vector3d udes_loc = {vx, 0.0, w};

    Eigen::Vector3d uopt, uopt_loc, utemp, utemp_loc;
    Eigen::Vector3d udes_loc = R_w_i.transpose() * udes;
    // std::cout << "Local position of centroid: " << centroid_local.transpose() << "\n";
    // std::cout << "Desired local velocity for robot " << this->ROBOT_ID <<": " << udes_loc.transpose() << "\n";
    std::cout  << "Slack variables matrix: \n--------------------------\n" << slack.transpose() << "\n--------------------------------\n";

    // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! p_j_est !!!!!!!!!!!!!!!!!!!
    if (!vision_controller.applyCbf(utemp_loc, udes_loc, p_j_mat, slack_mat)) // slack variables equal to 0 in dangerous conditions -> hard constraint
    {
        // utemp = R_w_i.transpose() * utemp_loc;
        utemp = R_w_i * utemp_loc;
        std::cout << "Desired control input: " << udes.transpose() << "\n";

        // uopt = utemp;
        uopt = boundVel(utemp);
        std::cout << "optimal local control input: " << uopt.transpose() << std::endl;
    }
    else
    {
        ROS_WARN("SAFETY CBF FAILED");
        uopt = udes;
    }

    // std::cout << "Optimal control input: " << uopt.transpose() << "\n";
    // // utemp = boundVel(utemp);

    if (uopt.head(2).norm() < CONVERGENCE_TOLERANCE)
    {
        std::cout << "Converged\n";
        uopt.head(2).setZero();
    }

    // std::vector<double> margins(ROBOTS_NUM-1);
    // std::fill(margins.begin(), margins.end(), 0.0);
    // safety_controller.applyCbfLocal(uopt, h, udes_loc, p_j_i, margins);

    // Publish control input
    mrs_msgs::VelocityReferenceStamped vel_msg;
    // vel_msg.header.frame_id = "uav" + std::to_string(ROBOT_ID) + "/local_origin";
    // vel_msg.header.frame_id = "/hummingbird" + std::to_string(ROBOT_ID) + "/base_link";
    vel_msg.reference.velocity.x = uopt(0);
    vel_msg.reference.velocity.y = uopt(1);
    vel_msg.reference.altitude = 3.0;
    vel_msg.reference.use_altitude = true;
    vel_msg.reference.heading_rate = uopt(2);
    vel_msg.reference.use_heading_rate = true;
    this->velPub_[0].publish(vel_msg);

    // fov_msg.header.stamp = ros::Time::now();
    rangePub_.publish(fov_msg);

    // if (SAVE_LOGS)
    // {
    //     // write velocity of robot(ID)
    //     std::string txt = std::to_string(uopt(0)) + " " + std::to_string(uopt(1)) + "\n";
    //     this->write_log_file(txt);
    // }

    if ((GRAPHICS_ON) && (this->app_gui->isOpen()))
    {
        // Draw Voronoi diagram (centralized)
        std::vector<Vector2<double>> mean_points_vec2;
        Vector2<double> n = {this->pose_x(ROBOT_ID), this->pose_y(ROBOT_ID)};
        mean_points_vec2.push_back(n);
        for (int j = 0; j < ROBOTS_NUM - 1; j++)
        {
            Vector2<double> mp = {filters[j]->getMean()(0), filters[j]->getMean()(1)};
            mean_points_vec2.push_back(mp);
            this->app_gui->drawPoint(mp, sf::Color(0, 0, 255));
        }
        Vector2<double> g = {GOAL_X, GOAL_Y};
        this->app_gui->drawPoint(g, sf::Color(255, 128, 0));
        this->app_gui->display();
    }

    // this->velPub_[0].publish(vel_msg);

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - timerstart).count();
    if (SAVE_CPU_TIME)
    {
        std::string txt = std::to_string(duration) + "\n";
        this->write_log_file(txt);
    }
    std::cout << "Computation time cost: -----------------: " << duration << " ms\n";
}

double Controller::mahalanobis_distance(Eigen::VectorXd x, Eigen::VectorXd mean, Eigen::MatrixXd cov)
{
    Eigen::VectorXd diff = x - mean;
    double dist = diff.transpose() * cov.inverse() * diff;
    return sqrt(dist);
}

Eigen::VectorXd Controller::gradV(Eigen::VectorXd q, Eigen::VectorXd x2, Eigen::VectorXd x_goal, Eigen::MatrixXd cov, double alpha)
{
    Eigen::Vector2d gradV_goal = 2 * (q.head(2) - x_goal.head(2));
    std::cout << "gradV_goal: " << gradV_goal.transpose() << "\n";
    // Eigen::Vector2d den = ((q.head(2) - x2.head(2)).transpose() * cov.block<2,2>(0,0).inverse());     // denominator of gradV_info
    // den = den.array().pow(3);
    // Eigen::Vector2d gradV_info = 2 * den.cwiseInverse();

    // V_info(x) = 1/(x-x2)^2  --> gradV_info(x) = -2*(x-x2)^(-3)
    // Eigen::Vector2d gradV_info = -2 * (q.head(2) - x2.head(2)).array().pow(-3);
    // double den = pow((pow((q(0) - x2(0)),2) + pow((q(1) - x2(1)),2)),2);
    // Eigen::Vector2d gradV_info = -2 * (q.head(2) - x2.head(2)) / den;
    Eigen::Vector2d gradV_info = (q.head(2) - x2.head(2)).cwiseInverse();
    std::cout << "gradV_info: " << gradV_info.transpose() << "\n";
    double th_des = 0.0;
    if (alpha > 0.5)
    {
        th_des = atan2(x_goal(1) - q(1), x_goal(0) - q(0));
    }
    else
    {
        th_des = atan2(x2(1) - q(1), x2(0) - q(0));
    }

    Eigen::Vector2d g = alpha * gradV_goal + (1 - alpha) * gradV_info;
    Eigen::Vector3d gradV;
    gradV << g(0), g(1), -th_des + q(2);
    return gradV;
}

// Vinfo(x) is the probability density function
Eigen::VectorXd Controller::gradV2(Eigen::VectorXd q, std::vector<Eigen::VectorXd> means, Eigen::VectorXd x_goal, std::vector<Eigen::MatrixXd> cov, double alpha)
{
    Eigen::Vector2d gradV_goal = 2 * (q.head(2) - x_goal.head(2));
    std::cout << "gradV_goal: " << gradV_goal.transpose() << "\n";

    double dj_dx = 0.0;
    double dj_dy = 0.0;
    std::vector<double> distances(means.size());
    for (int i = 0; i < means.size(); i++)
    {
        Eigen::VectorXd diff = q.head(2) - means[i].head(2);
        double dist = sqrt(pow((q(0) - means[i](0)), 2) + pow((q(1) - means[i](1)), 2));
        distances[i] = dist;
        double coeff = 1 / sqrt(cov[i].block<2, 2>(0, 0).determinant() * pow((2 * M_PI), 2));
        double dj_dx_i, dj_dy_i;
        double expon = diff.transpose() * cov[i].inverse() * diff;
        dj_dx_i = exp((-0.5 * expon)) * (-q(0) + means[i](0));
        dj_dy_i = exp((-0.5 * expon)) * (-q(1) + means[i](1));
        dj_dx += coeff * dj_dx_i * dist;
        dj_dy += coeff * dj_dy_i * dist;
    }
    Eigen::Vector2d gradV_info;
    gradV_info << -dj_dx, -dj_dy;
    std::cout << "gradV_info: " << gradV_info.transpose() << "\n";

    double min_dist = *std::min_element(distances.begin(), distances.end());
    int index = find(distances.begin(), distances.end(), min_dist) - distances.begin();
    double th_des = 0.0;
    if (alpha > 0.5)
    {
        th_des = atan2(x_goal(1) - q(1), x_goal(0) - q(0));
    }
    else
    {
        th_des = atan2(means[index](1) - q(1), means[index](0) - q(0));
    }

    Eigen::Vector2d g = alpha * gradV_goal + (1 - alpha) * gradV_info;
    Eigen::Vector3d gradV;
    gradV << g(0), g(1), q(2) - th_des;
    return gradV;
}

Eigen::VectorXd Controller::gradient_descent(Eigen::VectorXd q, std::vector<Eigen::VectorXd> means, Eigen::VectorXd x_goal, std::vector<Eigen::MatrixXd> cov, double alpha, double alpha_grad, int iters)
{
    Eigen::Vector2d q_n = q;
    for (int i = 0; i < iters; i++)
    {
        Eigen::VectorXd gradV = gradV2(q_n, means, x_goal, cov, alpha);
        q_n = q_n - alpha_grad * gradV;
    }
    double th_des = atan2(q_n(1) - q(1), q_n(0) - q(0));
    Eigen::Vector3d q_next;
    q_next << q_n(0), q_n(1), 2 * th_des;

    return q_next;
}

double Controller::sigmoid(double x)
{
    double v = 1 / (1 + exp(-x));
    return v;
}


void Controller::test_print()
{
    std::cout << "ENTERED" << std::endl;
}

void Controller::stop()
{
    // if (signum == SIGINT || signum == SIGKILL || signum ==  SIGQUIT || signum == SIGTERM)
    ROS_INFO("shutting down the controller, stopping the robots, closing the graphics window");
    // if ((GRAPHICS_ON) && (this->app_gui->isOpen())){
    //     this->app_gui->close();
    // }
    // this->timer_->cancel();
    ros::Duration(0.1).sleep();

    geometry_msgs::TwistStamped vel_msg;
    for (int i = 0; i < 100; ++i)
    {
        this->velPub_[0].publish(vel_msg);
    }

    ROS_INFO("controller has been closed and robots have been stopped");
    ros::Duration(0.1).sleep();

    ros::shutdown();
}

void Controller::realposeCallback(const nav_msgs::Odometry::ConstPtr msg, int i)
{
    this->realpose_x(i) = msg->pose.pose.position.x;
    this->realpose_y(i) = msg->pose.pose.position.y;

    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    this->realpose_theta(i) = yaw;

    if (this->justStarted[i])
    {
        this->pose_x(i) = this->realpose_x(i);
        this->pose_y(i) = this->realpose_y(i);
        this->pose_theta(i) = this->realpose_theta(i);
        this->justStarted[i] = false;
    }
}

void Controller::odomCallback(const nav_msgs::Odometry::ConstPtr msg)
{
    this->pose_x(ROBOT_ID) = msg->pose.pose.position.x;
    this->pose_y(ROBOT_ID) = msg->pose.pose.position.y;

    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    this->pose_theta(ROBOT_ID) = yaw;

    /*
    p_j(0, ROBOT_ID) = this->pose_x(ROBOT_ID);
    p_j(1, ROBOT_ID) = this->pose_y(ROBOT_ID);
    p_j(2, ROBOT_ID) = this->pose_theta(ROBOT_ID);
    */

    // std::cout << "I'm in odomCallback" << "\n";
    // std::cout << "Robot position: " << this->pose_x(ROBOT_ID) << ", " << this->pose_y(ROBOT_ID) << ", " << this->pose_theta(ROBOT_ID) << "\n";
}

void Controller::globalOdomCallback(const nav_msgs::Odometry::ConstPtr msg)
{
    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);


    p_j(0, ROBOT_ID) = msg->pose.pose.position.x;
    p_j(1, ROBOT_ID) = msg->pose.pose.position.y;
    p_j(2, ROBOT_ID) = yaw;

    // std::cout << "I'm in odomCallback" << "\n";
    // std::cout << "Robot position: " << this->pose_x(ROBOT_ID) << ", " << this->pose_y(ROBOT_ID) << ", " << this->pose_theta(ROBOT_ID) << "\n";
}

void Controller::neighCallback(const mrs_msgs::PoseWithCovarianceArrayStamped::ConstPtr msg)
{
    for (int j = 0; j < msg->poses.size(); j++)
    {
        int id = msg->poses[j].id;
        if (id != ROBOT_ID)
        {
            this->pose_x(id) = msg->poses[j].pose.position.x;
            this->pose_y(id) = msg->poses[j].pose.position.y;
            tf2::Quaternion q(
                msg->poses[j].pose.orientation.x,
                msg->poses[j].pose.orientation.y,
                msg->poses[j].pose.orientation.z,
                msg->poses[j].pose.orientation.w);
            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);

            if (!isnan(yaw))
            {
                this->pose_theta(id) = yaw;
            }
            else
            {
                this->pose_theta(id) = M_PI;
            }

            double sx = msg->poses[j].covariance[0];
            double sxy = msg->poses[j].covariance[1];
            double sy = msg->poses[j].covariance[7];
            covariances[id](0, 0) = sx;
            covariances[id](1, 0) = sxy;
            covariances[id](0, 1) = sxy;
            covariances[id](1, 1) = sy;
            std::cout << "Covariance of robot " << id << ": \n" << covariances[id] << std::endl;

            // Conversion to global position
            Eigen::MatrixXd R_w_i; // rotation matrix from local to global
            // R_w_i.resize(3,3);
            // R_w_i << cos(this->pose_theta(ROBOT_ID)), -sin(this->pose_theta(ROBOT_ID)), 0,
            //         sin(this->pose_theta(ROBOT_ID)), cos(this->pose_theta(ROBOT_ID)), 0,
            //         0, 0, 1;
            p_j(0, id) = this->pose_x(ROBOT_ID) + this->pose_x(id) * cos(this->pose_theta(ROBOT_ID)) - this->pose_y(id) * sin(this->pose_theta(ROBOT_ID));
            p_j(1, id) = this->pose_y(ROBOT_ID) + this->pose_x(id) * sin(this->pose_theta(ROBOT_ID)) + this->pose_y(id) * cos(this->pose_theta(ROBOT_ID));
            p_j(2, id) = this->pose_theta(id);

            std::cout << "Local position of robot " << id << ": " << this->pose_x(id) << ", " << this->pose_y(id) << std::endl;
            // std::cout << "Global position of robot " << id << ": " << p_j.col(id).transpose() << std::endl;

            received = true;
        }
    }

    // std::cout << "Global position of robots: \n" << p_j << std::endl;
}

Eigen::VectorXd Controller::boundVel(Eigen::VectorXd u)
{
    if (u(0) > MAX_LIN_VEL)
    {
        u(0) = MAX_LIN_VEL;
    }
    else if (u(0) < -MAX_LIN_VEL)
    {
        u(0) = -MAX_LIN_VEL;
    }

    if (u(1) > MAX_LIN_VEL)
    {
        u(1) = MAX_LIN_VEL;
    }
    else if (u(1) < -MAX_LIN_VEL)
    {
        u(1) = -MAX_LIN_VEL;
    }

    if (u(2) > MAX_ANG_VEL)
    {
        u(2) = MAX_ANG_VEL;
    }
    else if (u(2) < -MAX_ANG_VEL)
    {
        u(2) = -MAX_ANG_VEL;
    }

    return u;
}

Eigen::VectorXd Controller::boundVelprop(Eigen::VectorXd u)
{
    int id;
    if (u(0) > u(1))
    {
        id = 0;
    }
    else
    {
        id = 1;
    }

    if (u(id) > MAX_LIN_VEL)
    {
        double m = u(id) / MAX_LIN_VEL;
        u = u / m;
    }

    return u;
}

Eigen::VectorXd Controller::predictVelocity(Eigen::VectorXd q, Eigen::VectorXd goal)
{
    Eigen::VectorXd u(2);
    double K_gain = 1.0;
    u(0) = K_gain * (goal(0) - q(0));
    u(1) = K_gain * (goal(1) - q(1));

    if (sqrt(pow(goal(0) - q(0), 2) + pow(goal(1) - q(1), 2)) < CONVERGENCE_TOLERANCE)
    {
        u(0) = 0;
        u(1) = 0;
    }

    if (u(0) > MAX_LIN_VEL)
    {
        u(0) = MAX_LIN_VEL;
    }
    else if (u(0) < -MAX_LIN_VEL)
    {
        u(0) = -MAX_LIN_VEL;
    }
    if (u(1) > MAX_LIN_VEL)
    {
        u(1) = MAX_LIN_VEL;
    }
    else if (u(1) < -MAX_LIN_VEL)
    {
        u(1) = -MAX_LIN_VEL;
    }

    return u;
}

Eigen::VectorXd Controller::predictCentrVel(Eigen::VectorXd q, Vector2<double> goal)
{
    Eigen::VectorXd u(2);
    double K_gain = 1.0;
    u(0) = K_gain * (goal.x - q(0));
    u(1) = K_gain * (goal.y - q(1));

    if (sqrt(pow(goal.x - q(0), 2) + pow(goal.y - q(1), 2)) < CONVERGENCE_TOLERANCE)
    {
        u(0) = 0;
        u(1) = 0;
    }

    if (u(0) > MAX_LIN_VEL)
    {
        u(0) = MAX_LIN_VEL;
    }
    else if (u(0) < -MAX_LIN_VEL)
    {
        u(0) = -MAX_LIN_VEL;
    }
    if (u(1) > MAX_LIN_VEL)
    {
        u(1) = MAX_LIN_VEL;
    }
    else if (u(1) < -MAX_LIN_VEL)
    {
        u(1) = -MAX_LIN_VEL;
    }

    return u;
}

void Controller::vel_callback(const geometry_msgs::Twist::ConstPtr &msg)
{
    Eigen::Vector3d u_global;
    u_global(0) = msg->linear.x;
    u_global(1) = msg->linear.y;
    u_global(2) = msg->angular.z;

    Eigen::MatrixXd R_w_i; // rotation matrix from global to local
    R_w_i.resize(3, 3);
    R_w_i << cos(this->pose_theta(ROBOT_ID)), -sin(this->pose_theta(ROBOT_ID)), 0,
        sin(this->pose_theta(ROBOT_ID)), cos(this->pose_theta(ROBOT_ID)), 0,
        0, 0, 1;
    ustar = R_w_i * u_global;
}

void Controller::joy_callback(const geometry_msgs::Twist::ConstPtr msg)
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
                M(i, j) = V(i);
            }
            else
            {

                M(i, j) = 0;
            }
        }
    }
    return M;
}

geometry_msgs::Twist Controller::Diff_drive_compute_vel(double vel_x, double vel_y, double alfa)
{
    //-------------------------------------------------------------------------------------------------------
    // Compute velocities commands for the robot: differential drive control, for UAVs this is not necessary
    //-------------------------------------------------------------------------------------------------------

    geometry_msgs::Twist vel_msg;
    // double alfa = (this->pose_theta(i));
    double v = 0, w = 0;

    v = cos(alfa) * vel_x + sin(alfa) * vel_y;
    w = -(1 / b) * sin(alfa) * vel_x + (1 / b) * cos(alfa) * vel_y;

    if (abs(v) <= MAX_LIN_VEL)
    {
        vel_msg.linear.x = v;
    }
    else
    {
        if (v >= 0)
        {
            vel_msg.linear.x = MAX_LIN_VEL;
        }
        else
        {
            vel_msg.linear.x = -MAX_LIN_VEL;
        }
    }

    if (abs(w) <= MAX_ANG_VEL)
    {
        vel_msg.angular.z = w;
    }
    else
    {
        if (w >= 0)
        {
            vel_msg.angular.z = MAX_ANG_VEL;
        }
        else
        {
            vel_msg.angular.z = -MAX_ANG_VEL;
        }
    }
    return vel_msg;
}

bool Controller::is_outlier(Eigen::VectorXd q, Eigen::VectorXd mean, Eigen::MatrixXd cov_matrix, double threshold = 1e-3)
{
    Eigen::VectorXd diff = q - mean;
    double exponent = -0.5 * diff.transpose() * cov_matrix.inverse() * diff;
    double det = sqrt(pow(2 * M_PI, 2) * cov_matrix.determinant());
    double w = 1 / det * exp(exponent);
    if (w > threshold)
    {
        return false;
    }
    else
    {
        // std::cout << "===================================\n";
        // std::cout << "Outlier detected!! Score: " << w << "\n";
        // std::cout << "===================================\n";
        return true;
    }
}

bool Controller::isOut(Eigen::VectorXd sample, Eigen::VectorXd mean, Eigen::MatrixXd cov_matrix)
{
    if (!isinf(cov_matrix(0, 0)))
    {
        Eigen::EigenSolver<Eigen::MatrixXd> es(cov_matrix.block<2, 2>(0, 0));
        Eigen::VectorXd eigenvalues = es.eigenvalues().real();
        // std::cout << "Eigenvalues: \n" << eigenvalues.transpose() << "\n";
        Eigen::MatrixXd eigenvectors = es.eigenvectors().real();
        // std::cout << "Eigenvectors: \n" << eigenvectors.transpose() << "\n";

        // s = 4.605 for 90% confidence interval
        // s = 5.991 for 95% confidence interval
        // s = 9.210 for 99% confidence interval
        double s = 10.597;
        double a = sqrt(s * eigenvalues(0)); // major axis
        double b = sqrt(s * eigenvalues(1)); // minor axis

        // a could be smaller than b, so swap them
        if (a < b)
        {
            double temp = a;
            a = b;
            b = temp;
        }

        int m = 0; // higher eigenvalue index
        int l = 1; // lower eigenvalue index
        if (eigenvalues(1) > eigenvalues(0))
        {
            m = 1;
            l = 0;
        }

        double theta = atan2(eigenvectors(1, m), eigenvectors(0, m)); // angle of the major axis wrt positive x-asis (ccw rotation)
        if (theta < 0.0)
        {
            theta += M_PI;
        }                                                                                     // angle in [0, 2pi
        double slope = atan2(sample(1) - mean(1), sample(0) - mean(0));                       // angle of the line connecting the robot and the sample
        double dx_edge = mean(0) + a * cos(theta) * cos(slope) - b * sin(theta) * sin(slope); // x coordinate of the edge of the ellipse
        double dy_edge = mean(1) + a * sin(theta) * cos(slope) + b * cos(theta) * sin(slope); // y coordinate of the edge of the ellipse
        double d_edge = sqrt(pow(dx_edge - mean(0), 2) + pow(dy_edge - mean(1), 2));          // distance between the edge of the ellipse and the mean
        double dist = sqrt(pow(sample(0) - mean(0), 2) + pow(sample(1) - mean(1), 2));        // distance between the sample and the mean
        if (dist > d_edge)
        {
            // std::cout << "===================================\n";
            // std::cout << "Outlier detected!!\n";
            // std::cout << "===================================\n";
            return true;
        }
        else
        {
            return false;
        }
    }
}

// alternatively to a global variable to have access to the method you can make STATIC the class method interested,
// but some class function may not be accessed: "this->" method cannot be used

std::shared_ptr<Controller> globalobj_signal_handler; // the signal function requires only one argument {int}, so the class and its methods has to be global to be used inside the signal function.
void nodeobj_wrapper_function(int)
{
    std::cout << "signal handler function CALLED" << std::endl;
    node_shutdown_request = 1;
}

int main(int argc, char **argv)
{
    signal(SIGINT, nodeobj_wrapper_function);

    ros::init(argc, argv, "cbf_coverage", ros::init_options::NoSigintHandler);
    auto node = std::make_shared<Controller>();

    // globalobj_signal_handler = node;    //to use the ros function publisher, ecc the global pointer has to point to the same node object.

    // rclcpp::spin(node);

    // rclcpp::sleep_for(100000000ns);
    // rclcpp::shutdown();

    ros::spin();

    return 0;
}
