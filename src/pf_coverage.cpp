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
#include "pf_coverage/Diagram.h"
#include "pf_coverage/Graphics.h"
// ROS includes
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "sensor_msgs/msg/channel_float32.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/bool.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "turtlebot3_msgs/msg/gaussian.hpp"
#include "turtlebot3_msgs/msg/gmm.hpp"
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
const double MAX_ANG_VEL = 0.5;
const double MAX_LIN_VEL = 0.5;         //set to turtlebot max velocities
const double b = 0.025;                 //for differential drive control (only if we are moving a differential drive robot (e.g. turtlebot))
//------------------------------------------------------------------------
const bool centralized_centroids = false;   //compute centroids using centralized computed voronoi diagram
const float CONVERGENCE_TOLERANCE = 0.1;
//------------------------------------------------------------------------
const int shutdown_timer = 15;           //count how many seconds to let the robots stopped before shutting down the node

// std::vector<std::vector<double>> corners;

// Corners
// std::vector<Vector2<double>> corners{{-10.0,-10.0},{10.0,-10.0},{-10.0,-7.0},{-10.0,-5.0},{10.0,-3.0},{10.0,-1.0},{-10.0,1.0},{-10.0,3.0},{10.0,5.0},{10.0,7.0},{-10.0,10.0},{10.0,10.0}};

bool IsPathExist(const std::string &s)
{
  struct stat buffer;
  return (stat (s.c_str(), &buffer) == 0);
}


class Controller : public rclcpp::Node
{

public:
    Controller() : Node("pf_coverage")
    {
        //------------------------------------------------- ROS parameters ---------------------------------------------------------
        this->declare_parameter<int>("ROBOTS_NUM", 4);
        this->get_parameter("ROBOTS_NUM", ROBOTS_NUM);

        // ID of the controlled robot
        this->declare_parameter<int>("ROBOT_ID", 0);
        this->get_parameter("ROBOT_ID", ROBOT_ID);

        // Operating mode: 0 = coverage, 1 = milling
        this->declare_parameter<int>("MODE", 0);
        this->get_parameter("MODE", MODE);

        //Range di percezione singolo robot (= metà lato box locale)
        this->declare_parameter<double>("ROBOT_RANGE", 15.0);
        this->get_parameter("ROBOT_RANGE", ROBOT_RANGE);
        this->declare_parameter<double>("ROBOT_FOV", 120.0);
        this->get_parameter("ROBOT_FOV", ROBOT_FOV);

        // Numero di particelle del particle filter
        // this->declare_parameter<int>("PARTICLES_NUM", 500);
        // this->get_parameter("PARTICLES_NUM", PARTICLES_NUM);
        
        //view graphical voronoi rapresentation - bool
        this->declare_parameter<bool>("GRAPHICS_ON", true);
        this->get_parameter("GRAPHICS_ON", GRAPHICS_ON);

        // Area parameter
        this->declare_parameter<double>("AREA_SIZE_x", 40);
        this->get_parameter("AREA_SIZE_x", AREA_SIZE_x);
        this->declare_parameter<double>("AREA_SIZE_y", 40);
        this->get_parameter("AREA_SIZE_y", AREA_SIZE_y);
        this->declare_parameter<double>("AREA_LEFT", -20);
        this->get_parameter("AREA_LEFT", AREA_LEFT);
        this->declare_parameter<double>("AREA_BOTTOM", -20);
        this->get_parameter("AREA_BOTTOM", AREA_BOTTOM);

        this->declare_parameter<double>("GOAL_X", -5.0);
        this->get_parameter("GOAL_X", GOAL_X);
        this->declare_parameter<double>("GOAL_Y", 5.0);
        this->get_parameter("GOAL_Y", GOAL_Y);



        

    //--------------------------------------------------- Subscribers and Publishers ----------------------------------------------------
    for (int i = 0; i < ROBOTS_NUM; i++)
    {   
        realposeSub_.push_back(this->create_subscription<nav_msgs::msg::Odometry>("/turtlebot" + std::to_string(i) + "/odom", 100, [this, i](nav_msgs::msg::Odometry::SharedPtr msg) {this->realposeCallback(msg,i);}));
    }
    
    odomSub_ = this->create_subscription<nav_msgs::msg::Odometry>("/turtlebot" + std::to_string(ROBOT_ID) + "/odom", 1, std::bind(&Controller::odomCallback, this, _1));
    neighSub_ = this->create_subscription<geometry_msgs::msg::PoseArray>("/supervisor/robot" + std::to_string(ROBOT_ID) + "/pose", 1, std::bind(&Controller::neighCallback, this, _1));
    joySub_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 1, std::bind(&Controller::joy_callback, this, _1));
    gmmSub_ = this->create_subscription<turtlebot3_msgs::msg::GMM>("/gaussian_mixture_model", 1, std::bind(&Controller::gmm_callback, this, _1));
    velPub_.push_back(this->create_publisher<geometry_msgs::msg::Twist>("/turtlebot" + std::to_string(ROBOT_ID) + "/cmd_vel", 1));
    // voronoiPub = this->create_publisher<geometry_msgs::msg::PolygonStamped>("/voronoi"+std::to_string(ID)+"_diagram", 1);
    if (MODE == 0)
    {
        timer_ = this->create_wall_timer(500ms, std::bind(&Controller::pf_coverage, this));
    } else if (MODE == 1)
    {
        timer_ = this->create_wall_timer(500ms, std::bind(&Controller::pf_milling, this));
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
    GAUSSIAN_MEAN_PT << 10.0, 10.0;                       // Gaussian mean point
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
    void realposeCallback(const nav_msgs::msg::Odometry::SharedPtr msg, int i);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void neighCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg);
    void joy_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void gmm_callback(const turtlebot3_msgs::msg::GMM::SharedPtr msg);
    Eigen::VectorXd Matrix_row_sum(Eigen::MatrixXd x);
    Eigen::MatrixXd Diag_Matrix(Eigen::VectorXd V);
    void pf_coverage();
    void pf_milling();
    void coverage();
    bool insideFOV(Eigen::VectorXd q, Eigen::VectorXd q_obs, double fov, double r_sens);
    Eigen::VectorXd predictVelocity(Eigen::VectorXd q, Eigen::VectorXd mean_pt);
    Eigen::VectorXd getWheelVelocity(Eigen::VectorXd u, double alpha);
    geometry_msgs::msg::Twist Diff_drive_compute_vel(double vel_x, double vel_y, double alfa);
    Eigen::VectorXd start = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd initCovariance = 0.001*Eigen::VectorXd::Ones(3);
    int PARTICLES_NUM = 100;
    const std::size_t num_clusters = 1;
    double SAFETY_DIST = 1.0;
    // ParticleFilter *global_filter = new ParticleFilter(3*PARTICLES_NUM, start, initCovariance);

    
    



private:
    int ROBOTS_NUM;
    double ROBOT_RANGE;
    int ROBOT_ID;
    double ROBOT_FOV;
    int MODE;
    double GOAL_X, GOAL_Y;
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
    double dt = 0.5;
    std::vector<bool> justLost;

    //------------------------- Publishers and subscribers ------------------------------
    std::vector<rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr> velPub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr neighSub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr joySub_;
    rclcpp::Subscription<turtlebot3_msgs::msg::GMM>::SharedPtr gmmSub_;
    std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> realposeSub_;
    // rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr voronoiPub;
    rclcpp::TimerBase::SharedPtr timer_;
    turtlebot3_msgs::msg::GMM gmm_msg;
    geometry_msgs::msg::Polygon polygon_msg;
    geometry_msgs::msg::PolygonStamped polygonStamped_msg;

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
    Eigen::VectorXd GAUSSIAN_MEAN_PT;

    //------------------------------------------------------------------------------------

    //graphical view - ON/OFF
    bool GRAPHICS_ON;

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

    geometry_msgs::msg::Twist vel_msg = Diff_drive_compute_vel(vx,vy,alpha);
    double v_lin = vel_msg.linear.x;
    double v_ang = vel_msg.angular.z;

    // std::cout << "v_lin: " << v_lin << ", v_ang: " << v_ang << std::endl;

    double wl = v_lin / r - 0.5 * d * v_ang / r;
    double wr = v_lin / r + 0.5 * d * v_ang / r;

    Eigen::VectorXd u_final(2);
    u_final << wl, wr;

    return u_final;
}


void Controller::pf_coverage()
{
    auto timerstart = this->get_clock()->now().nanoseconds();
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
                    Eigen::VectorXd u_est(2);
                    u_est = getWheelVelocity(u_ax, q_est(2));                               // get wheel velocity [v_l, v_r] to reach the mean point
                    std::cout << "wheels velocity: " << u_est.transpose() << std::endl;
                    filters[c]->setProcessCovariance(processCovariance);
                    filters[c]->predict(0.5*u_est,dt);
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
    geometry_msgs::msg::Twist vel;

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
    vel = this->Diff_drive_compute_vel(vel_x, vel_y, this->pose_theta[ROBOT_ID]);

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
            vel.linear.x = 0.0;
            vel.angular.z = 0.0;
        } else
        {
            std::cout << "Safe condition. Moving to goal." << "\n";
        }
    }

    // Change angular velocity only in unsafe condition
    if (omega != 0.0)
    {
        vel.angular.z = 0.8;
    }

    this->velPub_[0]->publish(vel);

    auto end = this->get_clock()->now().nanoseconds();
    std::cout<<"Computation time cost: -----------------: "<<end - timerstart<<std::endl;
}


void Controller::pf_milling()
{
    // std::cout << "Starting" << std::endl;
    auto timerstart = this->get_clock()->now().nanoseconds();
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
                    mean_points.push_back(q_est);
                    // std::cout << "Estimated state: " << q_est.transpose() << std::endl;
                    u_ax = predictVelocity(q_est, robot);                        // get linear velocity [vx, vy] moving towards me
                    // std::cout << "Linear velocity: " << u_ax.transpose() << std::endl;
                    Eigen::VectorXd u_est(2);
                    u_est = getWheelVelocity(u_ax, q_est(2));                               // get wheel velocity [v_l, v_r] to reach the mean point
                    // std::cout << "wheels velocity: " << u_est.transpose() << std::endl;
                    filters[c]->setProcessCovariance(processCovariance);
                    filters[c]->predict(u_est,dt);
                    // std::cout << "Prediction completed" << std::endl;

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
                    // std::cout << "Particles converted to required format" << std::endl;

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
    geometry_msgs::msg::Twist vel;
    // vel.linear.x = this->vel_linear_x;
    // vel.angular.z = this->vel_angular_z;
    double xg = GOAL_X - this->pose_x(ROBOT_ID);
    double yg = GOAL_Y - this->pose_y(ROBOT_ID);
    if (xg * xg + yg * yg < CONVERGENCE_TOLERANCE)
    {
        vel.linear.x = 0.0;
        vel.angular.z = 0.0;
    } else
    {
        vel = Diff_drive_compute_vel(0.5*xg, 0.5*yg, this->pose_theta(ROBOT_ID));
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
            vel.linear.x = 0.0;
            vel.angular.z = 0.0;
        } else
        {
            std::cout << "Safe condition. Moving to goal." << "\n";
        }
    }

    // Change angular velocity only in unsafe condition
    if (omega != 0.0)
    {
        vel.angular.z = 0.8;
    }

    this->velPub_[0]->publish(vel);

    auto end = this->get_clock()->now().nanoseconds();
    std::cout<<"Computation time cost: -----------------: "<<end - timerstart<<std::endl;
}

void Controller::test_print()
{
    std::cout<<"ENTERED"<<std::endl;
}

void Controller::stop()
{
    //if (signum == SIGINT || signum == SIGKILL || signum ==  SIGQUIT || signum == SIGTERM)
    RCLCPP_INFO_STREAM(this->get_logger(), "shutting down the controller, stopping the robots, closing the graphics window");
    // if ((GRAPHICS_ON) && (this->app_gui->isOpen())){
    //     this->app_gui->close();
    // }
    this->timer_->cancel();
    rclcpp::sleep_for(100000000ns);

    geometry_msgs::msg::Twist vel_msg;
    for (int i = 0; i < 100; ++i)
    {
        for (int r = 0; r < ROBOTS_NUM; ++r)
        {
            this->velPub_[r]->publish(vel_msg);
        }
    }

    RCLCPP_INFO_STREAM(this->get_logger(), "controller has been closed and robots have been stopped");
    rclcpp::sleep_for(100000000ns);
    // this->close_log_file();
    // this->close_gauss_file();
    // this->close_k_file();
}

void Controller::realposeCallback(const nav_msgs::msg::Odometry::SharedPtr msg, int i)
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

void Controller::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
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
}

void Controller::neighCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
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

            this->pose_theta(j) = yaw;

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

    return u;
}



void Controller::joy_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    this->vel_linear_x = msg->linear.x;
    this->vel_angular_z = msg->angular.z;
}

void Controller::gmm_callback(const turtlebot3_msgs::msg::GMM::SharedPtr msg)
{
    this->gmm_msg.gaussians = msg->gaussians;
    this->gmm_msg.weights = msg->weights;
    // this->got_gmm = true;
    // RCLCPP_INFO_STREAM(this->get_logger(), "Sto ricevendo il GMM");
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


    rclcpp::spin(node);

    rclcpp::sleep_for(100000000ns);
    rclcpp::shutdown();

    return 0;
}
