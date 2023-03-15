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
// #include "kalman_filter/kalman_filter.h"
// #include "kalman_filter/extended_kalman_filter.h"
#include "particle_filter/particle_filter.h"
#include <GaussianMixtureModel/GaussianMixtureModel.h>
#include <GaussianMixtureModel/ExpectationMaximization.h>
#include <GaussianMixtureModel/TrainSet.h>
#include <GaussianMixtureModel/GaussianMixtureModelFactory.h>

#include <gaussian_mixture_model/gaussian_mixture_model.h>



#define M_PI   3.14159265358979323846  /*pi*/

using namespace std::chrono_literals;
using std::placeholders::_1;

//Robots parameters ------------------------------------------------------
double SAFETY_DIST = 1.0;
const double MAX_LIN_VEL = 1.5;         //set to turtlebot max velocities
// const double MAX_ANG_VEL = 2*M_PI*MAX_LIN_VEL/SAFETY_DIST;
const double MAX_ANG_VEL = 3.0;
const double b = 0.025;                 //for differential drive control (only if we are moving a differential drive robot (e.g. turtlebot))
//------------------------------------------------------------------------
const bool centralized_centroids = false;   //compute centroids using centralized computed voronoi diagram
const float CONVERGENCE_TOLERANCE = 0.1;
//------------------------------------------------------------------------
const int shutdown_timer = 15;           //count how many seconds to let the robots stopped before shutting down the node
//------------------------------------------------------------------------
sig_atomic_t volatile node_shutdown_request = 0;    //signal manually generated when ctrl+c is pressed
//------------------------------------------------------------------------

bool IsPathExist(const std::string &s)
{
  struct stat buffer;
  return (stat (s.c_str(), &buffer) == 0);
}


class Controller
{

public:
    Controller() : nh_priv_("~"), gmm_()
    {
        //------------------------------------------------- ROS parameters ---------------------------------------------------------
        this->nh_priv_.getParam("ROBOTS_NUM", ROBOTS_NUM);

        // ID of the controlled robot
        this->nh_priv_.getParam("ROBOT_ID", ROBOT_ID);

        // Operating mode: 0 = coverage, 1 = milling
        this->nh_priv_.getParam("MODE", MODE);
        
        //Range di percezione singolo robot (= metà lato box locale)
        this->nh_priv_.getParam("ROBOT_RANGE", ROBOT_RANGE);
        this->nh_priv_.getParam("ROBOT_FOV", ROBOT_FOV);

        //view graphical voronoi rapresentation - bool
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


    //--------------------------------------------------- Subscribers and Publishers ----------------------------------------------------
    for (int i = 0; i < ROBOTS_NUM; i++)
    {   
        realposeSub_.push_back(nh_.subscribe<nav_msgs::Odometry>("/hummingbird" + std::to_string(i) + "/ground_truth/odometry", 1, std::bind(&Controller::realposeCallback, this, std::placeholders::_1, i)));
    }
    
    odomSub_ = nh_.subscribe<nav_msgs::Odometry>("/hummingbird" + std::to_string(ROBOT_ID) + "/ground_truth/odometry", 1, std::bind(&Controller::odomCallback, this, std::placeholders::_1));
    neighSub_ = nh_.subscribe<geometry_msgs::PoseArray>("/supervisor/robot" + std::to_string(ROBOT_ID) + "/pose", 1, std::bind(&Controller::neighCallback, this, std::placeholders::_1));
    joySub_ = nh_.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, std::bind(&Controller::joy_callback, this, std::placeholders::_1));
    velPub_.push_back(nh_.advertise<geometry_msgs::TwistStamped>("/hummingbird" + std::to_string(ROBOT_ID) + "/autopilot/velocity_command", 1));
    timer_ = nh_.createTimer(ros::Duration(0.2), std::bind(&Controller::phd_coverage, this));
    
    //rclcpp::on_shutdown(std::bind(&Controller::stop,this));

    //----------------------------------------------------------- init Variables ---------------------------------------------------------
    pose_x = Eigen::VectorXd::Zero(ROBOTS_NUM);
    pose_y = Eigen::VectorXd::Zero(ROBOTS_NUM);
    pose_theta = Eigen::VectorXd::Zero(ROBOTS_NUM);
    p_j.resize(3,ROBOTS_NUM);                           // matrix with global position of neighbors on each column
    realpose_x = Eigen::VectorXd::Zero(ROBOTS_NUM);
    realpose_y = Eigen::VectorXd::Zero(ROBOTS_NUM);
    realpose_theta = Eigen::VectorXd::Zero(ROBOTS_NUM);
    GAUSSIAN_MEAN_PT.resize(2);
    GAUSSIAN_MEAN_PT << GAUSS_X, GAUSS_Y;                       // Gaussian mean point
    time(&this->timer_init_count);
    time(&this->timer_final_count);

    this->got_gmm = false;


    // Eigen::VectorXd fake_vec = Eigen::VectorXd::Zero(3);
    // Eigen::MatrixXd fake_matrix = Eigen::MatrixXd::Ones(3,3);
    // // filter = ParticleFilter::ParticleFilter((ROBOTS_NUM-1) * PARTICLES_NUM, fake_vec, fake_matrix);
    // gauss::GaussianDistribution *distribution_ = new gauss::GaussianDistribution(fake_vec, fake_matrix, true);
    // gauss::gmm::GaussianMixtureModel *gmm_ = new gauss::gmm::GaussianMixtureModel(0.8, *distribution_);
    // std::cout << "SONO QUI 1";
    // ParticleFilter *filter = new ParticleFilter((ROBOTS_NUM-1) * PARTICLES_NUM, fake_vec, fake_matrix);
    // filters.push_back(filter);

    for (int i = 0; i < ROBOTS_NUM - 1; i++)
    {
        ParticleFilter *filter = new ParticleFilter(PARTICLES_NUM, Eigen::Vector3d::Zero(), 0.1*Eigen::Vector3d::Ones());
        filters.push_back(filter);
    }
    

	// this->got_gmm = false;
    std::cout << "Hi! I'm robot number " << ROBOT_ID << std::endl;

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

    // std::cout << "Input : " << filter->getInput().transpose() << std::endl;
    }
    ~Controller()
    {
        if ((GRAPHICS_ON) && (this->app_gui->isOpen())){this->app_gui->close();}
        std::cout<<"DESTROYER HAS BEEN CALLED"<<std::endl;
    }

    //void stop(int signum);
    void stop();
    void test_print();
    void realposeCallback(const nav_msgs::Odometry::ConstPtr msg, int i);
    void odomCallback(const nav_msgs::Odometry::ConstPtr msg);
    void neighCallback(const geometry_msgs::PoseArray::ConstPtr msg);
    void joy_callback(const geometry_msgs::Twist::ConstPtr msg);
    Eigen::VectorXd Matrix_row_sum(Eigen::MatrixXd x);
    Eigen::MatrixXd Diag_Matrix(Eigen::VectorXd V);
    void phd_coverage();
    void pf_coverage();
    void save_distribution(std::vector<gauss::gmm::GaussianMixtureModel*> mix_models);
    bool insideFOV(Eigen::VectorXd q, Eigen::VectorXd q_obs, double fov, double r_sens);
    bool is_outlier(Eigen::VectorXd q, Eigen::VectorXd mean, Eigen::MatrixXd cov_matrix, double threshold);
    bool isOut(Eigen::VectorXd sample, Eigen::VectorXd mean, Eigen::MatrixXd cov_matrix);
    Eigen::VectorXd predictVelocity(Eigen::VectorXd q, Eigen::VectorXd mean_pt);
    Eigen::VectorXd getWheelVelocity(Eigen::VectorXd u, double alpha);
    geometry_msgs::Twist Diff_drive_compute_vel(double vel_x, double vel_y, double alfa);
    
    int PARTICLES_NUM = 40;
    



private:
    int ROBOTS_NUM = 6;
    double ROBOT_RANGE = 15.0;
    int ROBOT_ID = 0;
    double ROBOT_FOV = 150.0;
    int MODE = 0;
    double GOAL_X = 10.0;
    double GOAL_Y = 10.0;
    double GAUSS_X = 10.0;
    double GAUSS_Y = 10.0;
    // int PARTICLES_NUM;
    bool got_gmm;
    double vel_linear_x, vel_angular_z;
    Eigen::VectorXd pose_x;
    Eigen::VectorXd pose_y;
    Eigen::VectorXd pose_theta;
    Eigen::MatrixXd p_j;
    Eigen::VectorXd realpose_x;
    Eigen::VectorXd realpose_y;
    Eigen::VectorXd realpose_theta;
    std::vector<Vector2<double>> seeds_xy;
    int seeds_counter = 0;
    std::vector<std::vector<double>> corners;
    std::vector<double> position;
    // Eigen::VectorXd start;
    // Eigen::VectorXd initCovariance;
    double dt = 0.2;
    std::vector<bool> justLost;

    //------------------------------- Ros NodeHandle ------------------------------------
    ros::NodeHandle nh_;
    ros::NodeHandle nh_priv_;
    //------------------------- Publishers and subscribers ------------------------------
    std::vector<ros::Publisher> velPub_;
    ros::Subscriber odomSub_;
    ros::Subscriber neighSub_;
    ros::Subscriber joySub_;
    std::vector<ros::Subscriber> realposeSub_;
    // rclcpp::Publisher<geometry_msgs::PolygonStamped>::ConstPtr voronoiPub;
    ros::Timer timer_;

    // ------------------------------- Particle Filter ----------------------------------
    std::vector<ParticleFilter *> filters;
    // ParticleFilter filter;
    GaussianMixtureModel gmm_;

    
    // std::vector<Eigen::VectorXd> total_samples;
    
    
    //-----------------------------------------------------------------------------------

    //Rendering with SFML
    //------------------------------ graphics window -------------------------------------
    std::unique_ptr<Graphics> app_gui;
    //------------------------------------------------------------------------------------

    //---------------------------- Environment definition --------------------------------
    double AREA_SIZE_x = 20.0;
    double AREA_SIZE_y = 20.0;
    double AREA_LEFT = -10.0;
    double AREA_BOTTOM = -10.0;
    //------------------------------------------------------------------------------------

    //---------------------- Gaussian Density Function parameters ------------------------
    bool GAUSSIAN_DISTRIBUTION;
    double PT_X;
    double PT_Y;
    double VAR;
    Eigen::VectorXd GAUSSIAN_MEAN_PT;

    //------------------------------------------------------------------------------------

    //graphical view - ON/OFF
    bool GRAPHICS_ON = true;

    //timer - check how long robots are being stopped
    time_t timer_init_count;
    time_t timer_final_count;

    int counter = 0;

    // gauss::gmm::GaussianMixtureModel gmm_;

};

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


// function to get wheel velocities of differential drive robot
Eigen::VectorXd Controller::getWheelVelocity(Eigen::VectorXd u, double alpha)
{
    double r = 0.033;                           // turtlebot3 burger wheel radius
    double d = 0.16;                            // turtlebot3 burger distance between wheels

    double vx = u(0);
    double vy = u(1);

    geometry_msgs::Twist vel_msg = Diff_drive_compute_vel(vx,vy,alpha);
    double v_lin = vel_msg.linear.x;
    double v_ang = vel_msg.angular.z;

    // std::cout << "v_lin: " << v_lin << ", v_ang: " << v_ang << std::endl;

    double wl = v_lin / r - 0.5 * d * v_ang / r;
    double wr = v_lin / r + 0.5 * d * v_ang / r;

    Eigen::VectorXd u_final(2);
    u_final << wl, wr;

    return u_final;
}



void Controller::phd_coverage()
{
    auto timerstart = std::chrono::high_resolution_clock::now();
    Eigen::Vector3d processCovariance = 0.1*Eigen::Vector3d::Ones();
    Eigen::Vector3d robot;                           // controlled robot's global position
    robot << this->pose_x(ROBOT_ID), this->pose_y(ROBOT_ID), this->pose_theta(ROBOT_ID);
    Eigen::MatrixXd samples(3, PARTICLES_NUM);          // particles' global position
    std::vector<Eigen::VectorXd> samples_vec;
    // Eigen::MatrixXd samples_mat(3, PARTICLES_NUM*(ROBOTS_NUM-1));
    // std::vector<Eigen::VectorXd> samples_lost;
    std::vector<bool> detected(ROBOTS_NUM-1, false);      // vector of detected robots
    int detected_counter = 0;                           // number of detected robots
    std::vector<Eigen::VectorXd> detections;
    std::vector<Eigen::VectorXd> obs;
    std::vector<Eigen::MatrixXd> cov;
    std::vector<double> ws;

    // Define rotation matrix
    Eigen::Matrix<double,3,3> R_w_i;
    R_w_i << cos(this->pose_theta(ROBOT_ID)), -sin(this->pose_theta(ROBOT_ID)), 0,
             sin(this->pose_theta(ROBOT_ID)), cos(this->pose_theta(ROBOT_ID)), 0,
             0, 0, 1;

    for (int j = 0; j < ROBOTS_NUM; j++)
    {
        double c = j;                       // robot's id variable
        if (j > ROBOT_ID) {c = j-1;}
        if (j != ROBOT_ID)
        {
            // Check if the robot is detected
            if (this->pose_x(j) != 100.0 && this->pose_y(j) != 100.0)
            {
                std::cout << "Robot " << j << " detected in " << this->pose_x(j) << ", " << this->pose_y(j) << std::endl;
                detected[c] = true;
                detected_counter++;
                if (this->pose_x(j) == 0.0 && this->pose_y(j) == 0.0)
                {
                    std::cout << "Error in robot " << j << " initialization. Skipping..." << std::endl;
                    return;
                }

                if (!this->got_gmm)
                {
                    obs.push_back(p_j.col(j).head(2));
                    cov.push_back(Eigen::Matrix2d::Identity());
                    ws.push_back(1.0 / (ROBOTS_NUM-1));
                }

                // Case 1: robot detected --- define particles with small covariance around the detected position
                filters[c]->matchObservation(p_j.col(j));
            } else 
            {
                // Case 2: robot not detected
                std::cout << "Robot " << j << " not detected." << std::endl;            
                // Estimate velocity of the lost robot for coverage
                Eigen::VectorXd q_est = filters[c]->getMean();
                std::cout << "Estimated state: " << q_est.transpose() << std::endl;
                Eigen::Vector2d u_ax = predictVelocity(q_est, GAUSSIAN_MEAN_PT);                        // get linear velocity [vx, vy] moving towards me
                std::cout << "Linear velocity: " << u_ax.transpose() << std::endl;
                // Eigen::VectorXd u_est(2);
                // u_est = getWheelVelocity(u_ax, q_est(2));                               // get wheel velocity [v_l, v_r] to reach the mean point
                // std::cout << "wheels velocity: " << u_est.transpose() << std::endl;
                filters[c]->setProcessCovariance(processCovariance);
                filters[c]->predictUAV(0.5*Eigen::Vector2d::Ones(),dt);
                std::cout << "Prediction completed" << std::endl;

                // Get particles in required format
                Eigen::MatrixXd particles = filters[c]->getParticles();
                std::vector<Eigen::VectorXd> samples;
                for (int i=0; i<particles.cols(); i++)
                {
                    Eigen::VectorXd sample = particles.col(i);
                    if (!insideFOV(robot, sample, ROBOT_FOV, ROBOT_RANGE))
                    {
                        samples.push_back(sample);
                    }
                    // samples.push_back(sample);
                }
                std::cout << "Particles converted to required format" << std::endl;

                // Generate new particles to replace the lost ones
                // if (this->got_gmm)
                // {
                //     std::cout << "GMM already available. Generating needed particles: " << (PARTICLES_NUM - samples.size()) << std::endl;
                //     std::vector<Eigen::VectorXd> new_samples = gmm_.drawSamples(PARTICLES_NUM - samples.size());
                //     for (int k = 0; k < new_samples.size(); k++)
                //     {
                //         samples.push_back(new_samples[k]);
                //     }
                // } else
                // {
                // std::cout << "GMM not available. Generating needed particles: " << (PARTICLES_NUM - samples.size()) << std::endl;
                Eigen::VectorXd mean = filters[c]->getState();
                double mean_x = mean(0);
                double mean_y = mean(1);
                double mean_theta = mean(2);
                std::default_random_engine gen;
                std::normal_distribution<double> dx(mean_x, 0.5);
                std::normal_distribution<double> dy(mean_y, 0.5);
                std::normal_distribution<double> dtheta(mean_theta, 0.5);
                
                for (int k = samples.size(); k < PARTICLES_NUM; k++)
                {
                    Eigen::VectorXd sample(3);
                    sample << dx(gen), dy(gen), dtheta(gen);
                    samples.push_back(sample);
                }
                // }

                filters[c]->setParticles(samples);
            }
        }
    }

    if (!this->got_gmm)
    {
        gmm_.setMeans(obs);
        gmm_.setCovariances(cov);
        gmm_.setWeights(ws);
        gmm_.check();
        // filter.setParticles(samples_vec);
        
        this->got_gmm = true;
    }

    
    // Get all particles in required format
    std::cout << "Getting all particles in required format...\n";
    for (int i = 0; i < filters.size(); i++)
    {
        Eigen::MatrixXd samples = filters[i]->getParticles();
        for (int j = 0; j < samples.cols(); j++)
        {
            samples_vec.push_back(samples.col(j));
            // samples_mat.col(i*filters.size() + j) = samples.col(j);
        }
    }

    std::cout << "Running Expectation-Maximization...\n";
    gmm_.fitgmm(samples_vec, ROBOTS_NUM - 1, 1000, 1e-2, false);
    std::cout << "EM completed." << std::endl;
    std::cout << "GMM means: " << std::endl;
    for (int i = 0; i < gmm_.getMeans().size(); i++)
    {
        std::cout << gmm_.getMeans()[i].transpose() << std::endl;
    }



    if ((GRAPHICS_ON) && (this->app_gui->isOpen()))
    {
        this->app_gui->clear();
        this->app_gui->drawGlobalReference(sf::Color(255,255,0), sf::Color(255,255,255));
        this->app_gui->drawFOV(robot, ROBOT_FOV, ROBOT_RANGE);
        this->app_gui->drawPoint(GAUSSIAN_MEAN_PT, sf::Color(255,128,0));
        // Vector2<double> goal = {GOAL_X, GOAL_Y};
        // this->app_gui->drawPoint(goal, sf::Color(255,128,0));
        for (int i = 0; i < ROBOTS_NUM; i++)
        {
            auto color = sf::Color(0,255,0);                        // default color for other robots: green
            if (i == ROBOT_ID) {color = sf::Color(255,0,0);}        // controlled robot color: red
            Vector2<double> n;
            n.x = this->realpose_x(i);
            n.y = this->realpose_y(i);
            this->app_gui->drawPoint(n, color);
            // this->app_gui->drawID(n, i, color);
        }

        // this->app_gui->drawPoint(me);
        // this->app_gui->drawParticles(samples_mat);
        
        for (int i = 0; i < filters.size(); i++)
        {
            this->app_gui->drawParticles(filters[i]->getParticles());
        }
        
        this->app_gui->display();
    }






    auto end = std::chrono::high_resolution_clock::now();
    std::cout<<"Computation time cost: -----------------: "<<std::chrono::duration_cast<std::chrono::milliseconds>(end - timerstart).count()<<" ms\n";
}



void Controller::test_print()
{
    std::cout<<"ENTERED"<<std::endl;
}

void Controller::stop()
{
    //if (signum == SIGINT || signum == SIGKILL || signum ==  SIGQUIT || signum == SIGTERM)
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

    // std::cout << "I'm in odomCallback" << "\n";
    // std::cout << "Robot position: " << this->pose_x(ROBOT_ID) << ", " << this->pose_y(ROBOT_ID) << ", " << this->pose_theta(ROBOT_ID) << "\n";
}

void Controller::neighCallback(const geometry_msgs::PoseArray::ConstPtr msg)
{
    for (int j = 0; j < ROBOTS_NUM; j++)
    {
        if (j != ROBOT_ID)
        {
            this->pose_x(j) = msg->poses[j].position.x;
            this->pose_y(j) = msg->poses[j].position.y;

            tf2::Quaternion q(
            msg->poses[j].orientation.x,
            msg->poses[j].orientation.y,
            msg->poses[j].orientation.z,
            msg->poses[j].orientation.w);
            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);

            if (!isnan(yaw))
            {
                this->pose_theta(j) = yaw;
            } else
            {
                this->pose_theta(j) = M_PI;
            }

            // Conversion to global position
            Eigen::MatrixXd R_w_i;                          // rotation matrix from local to global
            // R_w_i.resize(3,3);
            // R_w_i << cos(this->pose_theta(ROBOT_ID)), -sin(this->pose_theta(ROBOT_ID)), 0,
            //         sin(this->pose_theta(ROBOT_ID)), cos(this->pose_theta(ROBOT_ID)), 0,
            //         0, 0, 1;
            p_j(0,j) = this->pose_x(ROBOT_ID) + this->pose_x(j) * cos(this->pose_theta(ROBOT_ID)) - this->pose_y(j) * sin(this->pose_theta(ROBOT_ID));
            p_j(1,j) = this->pose_y(ROBOT_ID) + this->pose_x(j) * sin(this->pose_theta(ROBOT_ID)) + this->pose_y(j) * cos(this->pose_theta(ROBOT_ID));
            p_j(2,j) = this->pose_theta(j);
        } else 
        {
            // column of the controlled robot contains its own global position
            p_j.col(j) << this->pose_x(ROBOT_ID), this->pose_y(ROBOT_ID), this->pose_theta(ROBOT_ID);
        }
    }

    // std::cout << "Global position of robots: \n" << p_j << std::endl;
}


Eigen::VectorXd Controller::predictVelocity(Eigen::VectorXd q, Eigen::VectorXd goal)
{
    Eigen::VectorXd u(2);
    double K_gain = 1.0;
    u(0) = K_gain * (goal(0) - q(0));
    u(1) = K_gain * (goal(1) - q(1));

    if (sqrt(pow(goal(0) - q(0),2) + pow(goal(1) - q(1),2)) < CONVERGENCE_TOLERANCE)
    {
        u(0) = 0;
        u(1) = 0;
    }

    if (u(0) > MAX_LIN_VEL)
    {
        u(0) = MAX_LIN_VEL;
    } else if (u(0) < -MAX_LIN_VEL)
    {
        u(0) = -MAX_LIN_VEL;
    }
    if (u(1) > MAX_LIN_VEL)
    {
        u(1) = MAX_LIN_VEL;
    } else if (u(1) < -MAX_LIN_VEL)
    {
        u(1) = -MAX_LIN_VEL;
    }

    return u;
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
                M(i,j) = V(i);
            }
            else{

                M(i,j) = 0;
            }
        }
    }
    return M;
}


geometry_msgs::Twist Controller::Diff_drive_compute_vel(double vel_x, double vel_y, double alfa){
    //-------------------------------------------------------------------------------------------------------
    //Compute velocities commands for the robot: differential drive control, for UAVs this is not necessary
    //-------------------------------------------------------------------------------------------------------

    geometry_msgs::Twist vel_msg;
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


void Controller::save_distribution(std::vector<gauss::gmm::GaussianMixtureModel*> mix_models)
{
    std::ofstream myfile;
    myfile.open ("/home/mattia/distribution"+std::to_string(ROBOT_ID)+".txt");
    myfile << "Considering Robot number " << ROBOT_ID << " in position " << this->pose_x(ROBOT_ID) << ", " << this->pose_y(ROBOT_ID) << ", " << this->pose_theta(ROBOT_ID) << "\n";
    for (int j = 0; j < mix_models.size(); j++)
    {
        double c = j;
        if (j > ROBOT_ID) {c = j-1;}
        if (j == ROBOT_ID) {continue;}
        if (this->pose_x(j) != 100.0 && this->pose_y(j) != 100.0)
        {
            myfile << "Robot number " << j << " detected.\n";
            myfile << "Position: " << this->pose_x(j) << ", " << this->pose_y(j) << ", " << this->pose_theta(j) << "\n";

        } else
        {
            myfile << "Robot number " << j  << " not detected. "<< "\n";
            myfile << "Mean: " << mix_models[c]->getClusters()[0].distribution->getMean().transpose() << "\n";
            myfile << "Covariance: \n" << mix_models[c]->getClusters()[0].distribution->getCovariance() << "\n";
        }
        
    }
    myfile.close();
}

bool Controller::is_outlier(Eigen::VectorXd q, Eigen::VectorXd mean, Eigen::MatrixXd cov_matrix, double threshold = 1e-3)
{
    Eigen::VectorXd diff = q - mean;
    double exponent = -0.5 * diff.transpose() * cov_matrix.inverse() * diff;
    double det = sqrt(pow(2*M_PI,2)*cov_matrix.determinant());
    double w = 1/det * exp(exponent);
    if (w > threshold)
    {
        return false;
    } else
    {
        // std::cout << "===================================\n";
        // std::cout << "Outlier detected!! Score: " << w << "\n";
        // std::cout << "===================================\n";
        return true;
    }
}

bool Controller::isOut(Eigen::VectorXd sample, Eigen::VectorXd mean, Eigen::MatrixXd cov_matrix)
{
    if(!isinf(cov_matrix(0,0)))
    {
        Eigen::EigenSolver<Eigen::MatrixXd> es(cov_matrix.block<2,2>(0,0));
        Eigen::VectorXd eigenvalues  = es.eigenvalues().real();
        // std::cout << "Eigenvalues: \n" << eigenvalues.transpose() << "\n";
        Eigen::MatrixXd eigenvectors = es.eigenvectors().real();
        // std::cout << "Eigenvectors: \n" << eigenvectors.transpose() << "\n";
        
        // s = 4.605 for 90% confidence interval
        // s = 5.991 for 95% confidence interval
        // s = 9.210 for 99% confidence interval
        double s = 10.597;
        double a = sqrt(s*eigenvalues(0));            // major axis
        double b = sqrt(s*eigenvalues(1));            // minor axis

        // a could be smaller than b, so swap them
        if (a < b)
        {
            double temp = a;
            a = b;
            b = temp;
        }

        int m = 0;                  // higher eigenvalue index
        int l = 1;                  // lower eigenvalue index
        if (eigenvalues(1) > eigenvalues(0)) 
        {
            m = 1;
            l = 0;
        }
        
        double theta = atan2(eigenvectors(1,m), eigenvectors(0,m));             // angle of the major axis wrt positive x-asis (ccw rotation)
        if (theta < 0.0) {theta += M_PI;}                                    // angle in [0, 2pi
        double slope = atan2(sample(1)-mean(1), sample(0)-mean(0));           // angle of the line connecting the robot and the sample
        double dx_edge = mean(0) + a*cos(theta)*cos(slope) - b*sin(theta)*sin(slope); // x coordinate of the edge of the ellipse
        double dy_edge = mean(1) + a*sin(theta)*cos(slope) + b*cos(theta)*sin(slope); // y coordinate of the edge of the ellipse
        double d_edge = sqrt(pow(dx_edge-mean(0),2) + pow(dy_edge-mean(1),2)); // distance between the edge of the ellipse and the mean
        double dist = sqrt(pow(sample(0)-mean(0),2) + pow(sample(1)-mean(1),2)); // distance between the sample and the mean
        if (dist > d_edge)
        {
            // std::cout << "===================================\n";
            // std::cout << "Outlier detected!!\n";
            // std::cout << "===================================\n";
            return true;
        } else
        {
            return false;
        }
    }
}





//alternatively to a global variable to have access to the method you can make STATIC the class method interested, 
//but some class function may not be accessed: "this->" method cannot be used

std::shared_ptr<Controller> globalobj_signal_handler;     //the signal function requires only one argument {int}, so the class and its methods has to be global to be used inside the signal function.
void nodeobj_wrapper_function(int){
    std::cout<<"signal handler function CALLED"<<std::endl;
    node_shutdown_request = 1;
}

int main(int argc, char **argv)
{
    signal(SIGINT, nodeobj_wrapper_function);

    ros::init(argc, argv, "phd_coverage", ros::init_options::NoSigintHandler);
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
