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
    Controller() : nh_priv_("~")
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
    service = nh_.advertiseService("change_goal", &Controller::changeGoal, this);
    if (MODE == 0)
    {
        timer_ = nh_.createTimer(ros::Duration(0.2), std::bind(&Controller::pf_coverage, this));
    } else if (MODE == 1)
    {
        timer_ = nh_.createTimer(ros::Duration(0.2), std::bind(&Controller::pf_milling, this));
    }
    
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

    Eigen::VectorXd fake_vec = Eigen::VectorXd::Zero(2);
    Eigen::MatrixXd fake_matrix = Eigen::MatrixXd::Ones(2,2);
    gauss::GaussianDistribution *distribution_ = new gauss::GaussianDistribution(fake_vec, fake_matrix, true);
    gauss::gmm::GaussianMixtureModel *gmm_ = new gauss::gmm::GaussianMixtureModel(0.8, *distribution_);
    for (int i = 0; i < ROBOTS_NUM-1; i++)
    {
        this->got_gmm.push_back(false);
        mix_models.push_back(gmm_);
    }
	// this->got_gmm = false;
    std::cout << "Hi! I'm robot number " << ROBOT_ID << std::endl;

    for (int i=0; i<ROBOTS_NUM-1; i++)
    {
        justLost.push_back(true);
    }

    

    for (int i = 0; i < ROBOTS_NUM - 1; i++)
    {
        ParticleFilter *filter = new ParticleFilter(PARTICLES_NUM, start, initCovariance);
        filters.push_back(filter);
    }
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
    void pf_coverage();
    void pf_milling();
    void save_distribution(std::vector<gauss::gmm::GaussianMixtureModel*> mix_models);
    bool changeGoal(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    void coverage();
    bool insideFOV(Eigen::VectorXd q, Eigen::VectorXd q_obs, double fov, double r_sens);
    bool is_outlier(Eigen::VectorXd q, Eigen::VectorXd mean, Eigen::MatrixXd cov_matrix, double threshold);
    bool isOut(Eigen::VectorXd sample, Eigen::VectorXd mean, Eigen::MatrixXd cov_matrix);
    Eigen::VectorXd predictVelocity(Eigen::VectorXd q, Eigen::VectorXd mean_pt);
    Eigen::VectorXd getWheelVelocity(Eigen::VectorXd u, double alpha);
    geometry_msgs::Twist Diff_drive_compute_vel(double vel_x, double vel_y, double alfa);
    Eigen::VectorXd start = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd initCovariance = 0.001*Eigen::VectorXd::Ones(3);
    int PARTICLES_NUM = 100;
    const std::size_t num_clusters = 1;
    // ParticleFilter *global_filter = new ParticleFilter(3*PARTICLES_NUM, start, initCovariance);

    
    



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
    std::vector<bool> got_gmm;
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
    ros::ServiceServer service;
    // rclcpp::Publisher<geometry_msgs::PolygonStamped>::ConstPtr voronoiPub;
    ros::Timer timer_;

    // -------------- EKF -------------------
    // kf::ExtendedKalmanFilter *filter = new kf::ExtendedKalmanFilter(input, output, state);
    // Eigen::VectorXd input, output, state;
    std::vector<ParticleFilter *> filters;
    std::vector<gauss::gmm::GaussianMixtureModel *> mix_models;
    
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

bool Controller::changeGoal(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    if ((GOAL_X >= 0.0) && (GOAL_Y >= 0.0))
    {
        GOAL_X = -GOAL_X;
        GOAL_Y = GOAL_Y;
        std::cout << "Goal changed to: " << GOAL_X << ", " << GOAL_Y << std::endl;
    }
    else if ((GOAL_X < 0.0) && (GOAL_Y >= 0.0))
    {
        GOAL_X = GOAL_X;
        GOAL_Y = -GOAL_Y;
        std::cout << "Goal changed to: " << GOAL_X << ", " << GOAL_Y << std::endl;
    }
    else if ((GOAL_X < 0.0) && (GOAL_Y < 0.0))
    {
        GOAL_X = -GOAL_X;
        GOAL_Y = GOAL_Y;
        std::cout << "Goal changed to: " << GOAL_X << ", " << GOAL_Y << std::endl;
    }
    else
    {
        GOAL_X = GOAL_X;
        GOAL_Y = -GOAL_Y;
        std::cout << "Goal changed to: " << GOAL_X << ", " << GOAL_Y << std::endl;
    }

    return true;

}

void Controller::pf_coverage()
{
    auto timerstart = std::chrono::high_resolution_clock::now();
    Eigen::VectorXd u(2);
    u << 0.5, 0.5;
    Eigen::VectorXd processCovariance = 0.1*Eigen::VectorXd::Ones(3);
    Eigen::VectorXd robot(3);                           // controlled robot's global position
    robot << this->pose_x(ROBOT_ID), this->pose_y(ROBOT_ID), this->pose_theta(ROBOT_ID);
    std::vector<Eigen::VectorXd> total_samples;                                       // samples from the filter
    std::vector<Eigen::VectorXd> mean_points;
    std::vector<double> distances;

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
                if (this->pose_x(j) == 0.0 && this->pose_y(j) == 0.0)
                {
                    std::cout << "Error in robot " << j << " initialization. Skipping..." << std::endl;
                    return;
                }
                // Case 1: robot detected: update the filter
                filters[c]->matchObservation(p_j.col(j));
                justLost[c] = true;
                this->got_gmm[c] = false;
                mean_points.push_back(p_j.col(j));
                
            } else
            {
                if (justLost[c])
                {
                    // Case 2: robot not detected - particle filter prediction
                    std::cout << "Robot " << j << " not detected" << std::endl;
                    // if (j > ROBOT_ID) { c = j-1; } 
                    // Estimate velocity of the lost robot for coverage
                    Eigen::VectorXd u_ax(2);
                    Eigen::VectorXd q_est = filters[c]->getMean();
                    mean_points.push_back(q_est);
                    std::cout << "Estimated state: " << q_est.transpose() << std::endl;
                    u_ax = predictVelocity(q_est, GAUSSIAN_MEAN_PT);                        // get linear velocity [vx, vy] moving towards me
                    std::cout << "Linear velocity: " << u_ax.transpose() << std::endl;
                    // Eigen::VectorXd u_est(2);
                    // u_est = getWheelVelocity(u_ax, q_est(2));                               // get wheel velocity [v_l, v_r] to reach the mean point
                    // std::cout << "wheels velocity: " << u_est.transpose() << std::endl;
                    filters[c]->setProcessCovariance(processCovariance);
                    filters[c]->predictUAV(u_ax,dt);
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

                    // Generate new samples to fill the filter 
                    if (this->got_gmm[c])
                    {   
                        // std::cout << "GMM already available. Generating needed particles: " << (PARTICLES_NUM - samples.size()) << std::endl; 
                        std::vector<Eigen::VectorXd> samples_gmm = mix_models[c]->drawSamples(PARTICLES_NUM - samples.size());
                        // std::cout << "Particles generated from GMM" << std::endl;
                        for (int k = 0; k < samples_gmm.size(); k++)
                        {
                            samples.push_back(samples_gmm[k]);
                        }

                    } else
                    {
                        std::cout << "GMM not available. Generating needed particles: " << (PARTICLES_NUM - samples.size()) << std::endl;
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
                        // std::cout << "Particles generated from normal distribution" << std::endl;
                    }
                    

                    filters[c]->setParticles(samples);
                    // std::cout << "Particles set" << std::endl;

                    gauss::TrainSet samples_set(samples);
                    std::cout << "Samples set defined, size: " << samples.size() <<". Applying EM ...\n";
                    std::vector<gauss::gmm::Cluster> clusters = gauss::gmm::ExpectationMaximization(samples_set, num_clusters);
                    std::cout << "EM applied\n";

                    // Create GMM from clusters
                    std::cout << "Creating GMM ...\n";
                    gauss::gmm::GaussianMixtureModel *model = new gauss::gmm::GaussianMixtureModel(clusters);
                    std::cout << "GMM created\n";
                    // *this->gmm_ = gmm;
                    this->got_gmm[c] = true;
                    mix_models[c] = model;

                    Eigen::MatrixXd covm = mix_models[c]->getClusters()[0].distribution->getCovariance();
                    // Remove samples inside FOV
                    // Eigen::VectorXd robot = Eigen::VectorXd::Zero(3);                           // controlled robot's position (always in 0,0,0 because in local coordinates)
                    // std::vector<Eigen::VectorXd> samples_filtered;
                    // std::cout << "Removing particles inside FOV ...\n";
                    // for (int i=0; i<samples.size(); i++)
                    // {
                    //     if (!insideFOV(robot, samples[i], ROBOT_FOV, ROBOT_RANGE))
                    //     {
                    //         total_samples.push_back(samples[i]);
                    //     }
                    // }
                    total_samples.insert(total_samples.end(), samples.begin(), samples.end());
                    std::cout << "Particles inside FOV removed. Total number of particles: " << total_samples.size() << "\n";
                    // justLost[c] = false;
                }
            }
        }
    }

    // Send velocity to the robot
    geometry_msgs::TwistStamped vel;

    std::cout << "Starting Voronoi calculation... \n";
    // ------------------------------- Voronoi -------------------------------
    // Get detected or estimated position of neighbors in local coordinates
    Box<double> AreaBox{AREA_LEFT, AREA_BOTTOM, AREA_SIZE_x + AREA_LEFT, AREA_SIZE_y + AREA_BOTTOM};
    Box<double> RangeBox{-ROBOT_RANGE, -ROBOT_RANGE, ROBOT_RANGE, ROBOT_RANGE};
    std::vector<double> VARs = {2.0};
    std::vector<Vector2<double>> MEANs = {{GAUSSIAN_MEAN_PT(0), GAUSSIAN_MEAN_PT(1)}};
    double vel_x=0, vel_y=0;

    Vector2<double> p = {this->pose_x(ROBOT_ID), this->pose_y(ROBOT_ID)};
    std::vector<Vector2<double>> local_points;
    local_points.push_back(p);
    // Vector2<double> p_local = {0.0, 0.0};
    // local_points.push_back(p_local);
    for (int i = 0; i < mean_points.size(); i++)
    {
        Vector2<double> p_local = {mean_points[i](0) - p.x, mean_points[i](1) - p.y};
        local_points.push_back(p_local);
    }

    std::cout << "Generating decentralized diagram.\n";
    auto diagram = generateDecentralizedDiagram(local_points, RangeBox, p, ROBOT_RANGE, AreaBox);
    std::cout << "Diagram generated. Calculating centroid.\n";
    Vector2<double> centroid = computePolygonCentroid(diagram, MEANs, VARs);
    std::cout << "Centroid: " << centroid.x << ", " << centroid.y << "\n";

    //---------------------------------------------------------------------------------------
    // from centroids compute velocity vectors according to Lloyd law
    //---------------------------------------------------------------------------------------

    if (centroid.getNorm() > CONVERGENCE_TOLERANCE)
    {
        vel_x = 0.8*(centroid.x);
        vel_y = 0.8*(centroid.y);
    } else {
        vel_x = 0;
        vel_y = 0;
    }

    std::cout << "Velocity towards centroid : " << vel_x << ", " << vel_y << "\n";

    //-------------------------------------------------------------------------------------------------------
    //Compute velocities commands for the robot: differential drive control, for UAVs this is not necessary
    //-------------------------------------------------------------------------------------------------------
    // vel.twist = this->Diff_drive_compute_vel(vel_x, vel_y, this->pose_theta[ROBOT_ID]);
    vel.twist.linear.x = vel_x;
    vel.twist.linear.y = vel_y;
    // double th;
    // if (vel_x != 0 && vel_y != 0)
    // {
    //     th = atan2(centroid.y, centroid.x);   
    // } else
    // {
    //     th = atan2((GAUSSIAN_MEAN_PT(1) - this->pose_y(ROBOT_ID)), GAUSSIAN_MEAN_PT(0) - this->pose_x(ROBOT_ID));
    // }
    if (sqrt(pow(centroid.x,2) + pow(centroid.y,2)) > CONVERGENCE_TOLERANCE)
    {
        double th = atan2(centroid.y, centroid.x);
        double ww = 0.8 * (th - this->pose_theta[ROBOT_ID]);
        vel.twist.angular.z = ww;
        std::cout << "Angular velocity: " << ww << std::endl;
    }

    //-------------------------------------------------------------------------------------------------------



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
        for (int j = 0; j < ROBOTS_NUM; j++)
        {
            if (j > ROBOT_ID)
            {
                this->app_gui->drawParticles(filters[j-1]->getParticles());
            }
            else if (j < ROBOT_ID)
            {
                this->app_gui->drawParticles(filters[j]->getParticles());
            }
        }

        // while (sf::Mouse::isButtonPressed(sf::Mouse::Left))
        // {
        //     std::cout << "Saving distrribution.\n";
        //     this->save_distribution(mix_models);
        //     std::cout << "Distribution saved.\n";
        // }   

        // Draw Voronoi diagram (centralized)
        std::vector<Vector2<double>> mean_points_vec2;
        Vector2<double> n = {this->pose_x(ROBOT_ID), this->pose_y(ROBOT_ID)};
        mean_points_vec2.push_back(n);
        for (int j = 0; j < mean_points.size(); j++)
        {
            Vector2<double> mp = {mean_points[j](0), mean_points[j](1)};
            mean_points_vec2.push_back(mp);
        }
        auto diagram_centr = generateCentralizedDiagram(mean_points_vec2, AreaBox);
        Vector2<double> centroid_global = {centroid.x + p.x, centroid.y + p.y};
        this->app_gui->drawDiagram(diagram_centr);
        this->app_gui->drawPoint(centroid_global, sf::Color(0,255,255));


        for (int i = 0; i < mean_points.size(); i++)
        {
            this->app_gui->drawPoint(mean_points[i], sf::Color(0,0,255));

            if (this->got_gmm[i])
            {
                Eigen::MatrixXd cov_matrix = mix_models[i]->getClusters()[0].distribution->getCovariance();
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
                    double s = 4.605;
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
                    this->app_gui->drawEllipse(mean_points[i], a, b, theta);

                    double slope = atan2(-mean_points[i](1) + this->pose_y(ROBOT_ID), -mean_points[i](0) + this->pose_x(ROBOT_ID));
                    // slope += theta;
                    // double slope = 0.0;
                    // double x_n = mean_points[i](0) + a*cos(0.0);
                    // double y_n = mean_points[i](1) + b*sin(0.0);
                    double x_n = mean_points[i](0) + a * cos(slope - theta) * cos(theta) - b * sin(slope - theta) * sin(theta);
                    double y_n = mean_points[i](1) + a * cos(slope - theta) * sin(theta) + b * sin(slope - theta) * cos(theta);
                    // double x_n = mean_points[i](0) + eigenvectors(0,m) * a * cos(slope) + eigenvectors(0,l) * b * sin(slope);
                    // double y_n = mean_points[i](1) + eigenvectors(1,m) * a * cos(slope) + eigenvectors(1,l) * b * sin(slope);
                    Vector2<double> p_near = {x_n, y_n};

                    // std::cout << "Robot position: " << this->pose_x(ROBOT_ID) << ", " << this->pose_y(ROBOT_ID) << "\n";
                    // std::cout << "Neighbor estimate: " << mean_points[i](0) << ", " << mean_points[i](1) << "\n";
                    // std::cout << "Ellipse orientation: " << theta << "\n";
                    // std::cout << "Slope" << slope << "\n";
                    // std::cout << "Eigenvalues: " << eigenvalues(m) << ", " << eigenvalues(l) << "\n";

                    double dist = sqrt(pow(p_near.x - this->pose_x(ROBOT_ID), 2) + pow(p_near.y - this->pose_y(ROBOT_ID), 2));
                    std::cout << "Distance: " << dist << "\n";

                    // Check if robot is inside ellipse
                    double d = sqrt(pow(mean_points[i](0) - this->pose_x(ROBOT_ID), 2) + pow(mean_points[i](1) - this->pose_y(ROBOT_ID), 2));
                    if (d < a)
                    {
                        distances.push_back(0.5*SAFETY_DIST);
                    } else
                    {
                        distances.push_back(dist);
                    }

                    this->app_gui->drawPoint(p_near, sf::Color(255,0,127));
                }
            }
        }
    
        this->app_gui->display();
    }

    // Get mean point of predictions and rotate robot to face it
    // double w = 0;
    // if (mean_points.size() > 0)
    // {
    //     double xm = 0;
    //     double ym = 0;

    //     for (int i = 0; i < mean_points.size(); i++)
    //     {
    //         xm += mean_points[i](0);
    //         ym += mean_points[i](1);
    //     }

    //     xm /= mean_points.size();
    //     ym /= mean_points.size();

    //     double theta_des = atan2((ym-this->realpose_y(ROBOT_ID)), (xm-this->realpose_x(ROBOT_ID)));
    //     w  = 0.5*(theta_des - this->realpose_theta(ROBOT_ID));
    // }
    


    

    double omega = 0.0;
    for (int i = 0; i < distances.size(); i++)
    {
        if (distances[i] < SAFETY_DIST)
        {
            std::cout << "Unsafe condition. Avoiding collision." << "\n";
            
            double theta_des = atan2((mean_points[i](1)-this->pose_y(ROBOT_ID)), (mean_points[i](0)-this->pose_x(ROBOT_ID)));
            double w = -0.5*(theta_des - this->pose_theta(ROBOT_ID));
            if (abs(w) > abs(omega)) {omega = w;}
            vel.twist.linear.x = 0.0;
            vel.twist.linear.y = 0.0;
            vel.twist.angular.z = 0.0;
        } else
        {
            std::cout << "Safe condition. Moving to goal." << "\n";
        }
    }

    // Start rotating in unsafe condition
    if (omega != 0.0)
    {
        vel.twist.angular.z = MAX_ANG_VEL;
    }

    this->velPub_[0].publish(vel);

    auto end = std::chrono::high_resolution_clock::now();
    std::cout<<"Computation time cost: -----------------: "<<std::chrono::duration_cast<std::chrono::nanoseconds>(end - timerstart).count()<<" ns\n";
}


void Controller::pf_milling()
{
    // std::cout << "Starting" << std::endl;
    auto timerstart = std::chrono::high_resolution_clock::now();
    Eigen::VectorXd u(2);
    u << 0.5, 0.5;
    Eigen::VectorXd processCovariance = 1.0*Eigen::VectorXd::Ones(3);
    Eigen::VectorXd robot(3);                           // controlled robot's global position
    robot << this->pose_x(ROBOT_ID), this->pose_y(ROBOT_ID), this->pose_theta(ROBOT_ID);
    std::vector<Eigen::VectorXd> total_samples;                                       // samples from the filter
    std::vector<Eigen::VectorXd> mean_points;
    std::vector<double> distances;

    for (int j = 0; j < ROBOTS_NUM; j++)
    {
        double c = j;                       // robot's id variable
        if (j > ROBOT_ID) {c = j-1;}
        if (j != ROBOT_ID)
        {
            // Check if the robot is detected
            if (this->pose_x(j) != 100.0 && this->pose_y(j) != 100.0)
            {
                // std::cout << "Robot " << j << " detected in " << this->pose_x(j) << ", " << this->pose_y(j) << std::endl;
                // Case 1: robot detected: update the filter
                // Eigen::VectorXd obs(3);
                // obs(0) = this->pose_x(j);
                // obs(1) = this->pose_y(j);
                // obs(2) = this->pose_theta(j);
                filters[c]->matchObservation(p_j.col(j));
                justLost[c] = true;
                this->got_gmm[c] = false;
                mean_points.push_back(p_j.col(j));
                
            } else
            {
                if (justLost[c])
                {
                    // Case 2: robot not detected - particle filter prediction
                    // std::cout << "Robot " << j << " not detected" << std::endl;
                    // if (j > ROBOT_ID) { c = j-1; } 
                    // Estimate velocity of the lost robot for coverage
                    Eigen::VectorXd u_ax(2);
                    Eigen::VectorXd q_est = filters[c]->getMean();
                    if (insideFOV(robot, q_est, ROBOT_FOV, ROBOT_RANGE))
                    {
                        double th = atan2(q_est(1)-robot(1), q_est(0)-robot(0));
                        q_est(0) = robot(0) + ROBOT_RANGE*cos(th);
                        q_est(1) = robot(1) + ROBOT_RANGE*sin(th);
                        filters[c]->setState(q_est);
                        this->got_gmm[c] = false;
                        
                    }
                    mean_points.push_back(q_est);
                    // std::cout << "Estimated state: " << q_est.transpose() << std::endl;
                    u_ax = predictVelocity(q_est, robot);                        // get linear velocity [vx, vy] moving towards me
                    // std::cout << "Linear velocity: " << u_ax.transpose() << std::endl;
                    // Eigen::VectorXd u_est(2);
                    // u_est = getWheelVelocity(u_ax, q_est(2));                               // get wheel velocity [v_l, v_r] to reach the mean point
                    // std::cout << "wheels velocity: " << u_est.transpose() << std::endl;
                    filters[c]->setProcessCovariance(processCovariance);
                    filters[c]->predictUAV(u_ax,dt);
                    // std::cout << "Prediction completed" << std::endl;

                    // Get particles in required format
                    Eigen::MatrixXd particles = filters[c]->getParticles();
                    std::vector<Eigen::VectorXd> samples;
                    int outliers_counter = 0;
                    for (int i=0; i<particles.cols(); i++)
                    {
                        Eigen::VectorXd sample = particles.col(i);
                        if (!insideFOV(robot, sample, ROBOT_FOV, ROBOT_RANGE))
                        {
                            if (this->got_gmm[c])
                            {
                                Eigen::MatrixXd covm = mix_models[c]->getClusters()[0].distribution->getCovariance();
                                Eigen::VectorXd m = mix_models[c]->getClusters()[0].distribution->getMean();
                                if (!isOut(sample, m, covm))
                                {
                                    samples.push_back(sample);
                                } else
                                {
                                    outliers_counter += 1;
                                }
                            } 
                            // else
                            // {
                            //     samples.push_back(sample);
                            // }
                        }
                        // samples.push_back(sample);
                    }
                    // std::cout << "Particles converted to required format" << std::endl;
                    std::cout << "====================================\n";
                    std::cout << "Robot " << j << "'s outliers: " << outliers_counter << std::endl;
                    std::cout << "====================================\n";

                    // Generate new samples to fill the filter 
                    if (this->got_gmm[c])
                    {   
                        std::cout << "GMM already available. Generating needed particles: " << (PARTICLES_NUM - samples.size()) << std::endl; 
                        std::vector<Eigen::VectorXd> samples_gmm = mix_models[c]->drawSamples(PARTICLES_NUM - samples.size());
                        // std::cout << "Particles generated from GMM" << std::endl;
                        for (int k = 0; k < samples_gmm.size(); k++)
                        {
                            samples.push_back(samples_gmm[k]);
                        }

                    } else
                    {
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
                        // std::cout << "Particles generated from normal distribution" << std::endl;
                    }
                    

                    filters[c]->setParticles(samples);
                    // std::cout << "Particles set" << std::endl;

                    gauss::TrainSet samples_set(samples);
                    std::cout << "Samples set defined, size: " << samples.size() <<". Applying EM ...\n";
                    std::vector<gauss::gmm::Cluster> clusters = gauss::gmm::ExpectationMaximization(samples_set, num_clusters);
                    std::cout << "EM applied\n";

                    // Create GMM from clusters
                    std::cout << "Creating GMM ...\n";
                    gauss::gmm::GaussianMixtureModel *model = new gauss::gmm::GaussianMixtureModel(clusters);
                    std::cout << "GMM created\n";
                    // *this->gmm_ = gmm;
                    this->got_gmm[c] = true;
                    mix_models[c] = model;

                    
                    // Remove samples inside FOV
                    // Eigen::VectorXd robot = Eigen::VectorXd::Zero(3);                           // controlled robot's position (always in 0,0,0 because in local coordinates)
                    // std::vector<Eigen::VectorXd> samples_filtered;
                    // std::cout << "Removing particles inside FOV ...\n";
                    // for (int i=0; i<samples.size(); i++)
                    // {
                    //     if (!insideFOV(robot, samples[i], ROBOT_FOV, ROBOT_RANGE))
                    //     {
                    //         total_samples.push_back(samples[i]);
                    //     }
                    // }
                    total_samples.insert(total_samples.end(), samples.begin(), samples.end());
                    // std::cout << "Particles inside FOV removed. Total number of particles: " << total_samples.size() << "\n";
                    // justLost[c] = false;
                }
            }
        }
    }


    if ((GRAPHICS_ON) && (this->app_gui->isOpen()))
    {
        this->app_gui->clear();
        this->app_gui->drawGlobalReference(sf::Color(255,255,0), sf::Color(255,255,255));
        this->app_gui->drawFOV(robot, ROBOT_FOV, ROBOT_RANGE);
        Vector2<double> goal = {GOAL_X, GOAL_Y};
        this->app_gui->drawPoint(goal, sf::Color(255,128,0));
        for (int i = 0; i < ROBOTS_NUM; i++)
        {
            auto color = sf::Color(0,255,0);                        // default color for other robots: green
            if (i == ROBOT_ID) {color = sf::Color(255,0,0);}        // controlled robot color: red
            Vector2<double> n;
            n.x = this->realpose_x(i);
            n.y = this->realpose_y(i);
            this->app_gui->drawPoint(n, color);
            
        }

        // this->app_gui->drawPoint(me);
        for (int j = 0; j < ROBOTS_NUM; j++)
        {
            if (j > ROBOT_ID)
            {
                this->app_gui->drawParticles(filters[j-1]->getParticles());
            }
            else if (j < ROBOT_ID)
            {
                this->app_gui->drawParticles(filters[j]->getParticles());
            }
        }


        for (int i = 0; i < mean_points.size(); i++)
        {
            this->app_gui->drawPoint(mean_points[i], sf::Color(0,0,255));

            if (this->got_gmm[i])
            {
                Eigen::MatrixXd cov_matrix = mix_models[i]->getClusters()[0].distribution->getCovariance();
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
                    double s = 5.991;
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
                    this->app_gui->drawEllipse(mean_points[i], a, b, theta);

                    double slope = atan2(-mean_points[i](1) + this->pose_y(ROBOT_ID), -mean_points[i](0) + this->pose_x(ROBOT_ID));
                    // slope += theta;
                    // double slope = 0.0;
                    // double x_n = mean_points[i](0) + a*cos(0.0);
                    // double y_n = mean_points[i](1) + b*sin(0.0);
                    double x_n = mean_points[i](0) + a * cos(slope - theta) * cos(theta) - b * sin(slope - theta) * sin(theta);
                    double y_n = mean_points[i](1) + a * cos(slope - theta) * sin(theta) + b * sin(slope - theta) * cos(theta);
                    // double x_n = mean_points[i](0) + eigenvectors(0,m) * a * cos(slope) + eigenvectors(0,l) * b * sin(slope);
                    // double y_n = mean_points[i](1) + eigenvectors(1,m) * a * cos(slope) + eigenvectors(1,l) * b * sin(slope);
                    Vector2<double> p_near = {x_n, y_n};

                    // std::cout << "Robot position: " << this->pose_x(ROBOT_ID) << ", " << this->pose_y(ROBOT_ID) << "\n";
                    // std::cout << "Neighbor estimate: " << mean_points[i](0) << ", " << mean_points[i](1) << "\n";
                    // std::cout << "Ellipse orientation: " << theta << "\n";
                    // std::cout << "Slope" << slope << "\n";
                    // std::cout << "Eigenvalues: " << eigenvalues(m) << ", " << eigenvalues(l) << "\n";

                    double dist = sqrt(pow(p_near.x - this->pose_x(ROBOT_ID), 2) + pow(p_near.y - this->pose_y(ROBOT_ID), 2));
                    std::cout << "Distance: " << dist << "\n";

                    // Check if robot is inside ellipse
                    double d = sqrt(pow(mean_points[i](0) - this->pose_x(ROBOT_ID), 2) + pow(mean_points[i](1) - this->pose_y(ROBOT_ID), 2));
                    if (d < a)
                    {
                        distances.push_back(0.5*SAFETY_DIST);
                    } else
                    {
                        distances.push_back(dist);
                    }

                    this->app_gui->drawPoint(p_near, sf::Color(255,0,127));
                }
            }
        }
    
        this->app_gui->display();
    }

    // Get mean point of predictions and rotate robot to face it
    // double w = 0;
    // if (mean_points.size() > 0)
    // {
    //     double xm = 0;
    //     double ym = 0;

    //     for (int i = 0; i < mean_points.size(); i++)
    //     {
    //         xm += mean_points[i](0);
    //         ym += mean_points[i](1);
    //     }

    //     xm /= mean_points.size();
    //     ym /= mean_points.size();

    //     double theta_des = atan2((ym-this->realpose_y(ROBOT_ID)), (xm-this->realpose_x(ROBOT_ID)));
    //     w  = 0.5*(theta_des - this->realpose_theta(ROBOT_ID));
    // }


    // Send velocity to the robot
    geometry_msgs::TwistStamped vel;
    // vel.linear.x = this->vel_linear_x;
    // vel.angular.z = this->vel_angular_z;
    double xg = GOAL_X - this->pose_x(ROBOT_ID);
    double yg = GOAL_Y - this->pose_y(ROBOT_ID);
    if (xg * xg + yg * yg < CONVERGENCE_TOLERANCE)
    {
        vel.twist.linear.x = 0.0;
        vel.twist.linear.y = 0.0;
        vel.twist.angular.z = 0.0;
    } else
    {
        // vel.twist = Diff_drive_compute_vel(0.5*xg, 0.5*yg, this->pose_theta(ROBOT_ID));
        if (xg > MAX_LIN_VEL) {xg = MAX_LIN_VEL;}
        else if (xg < -MAX_LIN_VEL) {xg = -MAX_LIN_VEL;}
        if (yg > MAX_LIN_VEL) {yg = MAX_LIN_VEL;}
        else if (yg < -MAX_LIN_VEL) {yg = -MAX_LIN_VEL;}
        vel.twist.linear.x = xg;
        vel.twist.linear.y = yg;
        double ww = atan2(yg, xg) - this->pose_theta(ROBOT_ID);
        if (ww > M_PI) {ww -= 2*M_PI;}
        else if (ww < -M_PI) {ww += 2*M_PI;}
        if (ww > MAX_ANG_VEL) {ww = MAX_ANG_VEL;}
        else if (ww < -MAX_ANG_VEL) {ww = -MAX_ANG_VEL;}
        vel.twist.angular.z = ww;
    }

    double omega = 0.0;
    for (int i = 0; i < distances.size(); i++)
    {
        if (distances[i] < SAFETY_DIST)
        {
            std::cout << "Unsafe condition. Avoiding collision." << "\n";
            
            double theta_des = atan2((mean_points[i](1)-this->pose_y(ROBOT_ID)), (mean_points[i](0)-this->pose_x(ROBOT_ID)));
            double w = -0.5*(theta_des - this->pose_theta(ROBOT_ID));
            if (abs(w) > abs(omega)) {omega = w;}
            vel.twist.linear.x = 0.0;
            vel.twist.linear.y = 0.0;
            vel.twist.angular.z = 0.0;
        } else
        {
            std::cout << "Safe condition. Moving to goal." << "\n";
        }
    }

    // Change angular velocity only in unsafe condition
    if (omega != 0.0)
    {
        vel.twist.angular.z = MAX_ANG_VEL;
    }

    this->velPub_[0].publish(vel);

    auto end = std::chrono::high_resolution_clock::now();

    std::cout<<"Computation time cost: -----------------: "<<std::chrono::duration_cast<std::chrono::nanoseconds>(end - timerstart).count()<<" ns\n";
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

    ros::init(argc, argv, "pf_coverage", ros::init_options::NoSigintHandler);
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
