#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation/trajectory.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/PoseStamped.h"
#include "ros/ros.h"
#include <mav_planning_msgs/PolynomialTrajectory4D.h>
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation/timing.h>
#include <mav_trajectory_generation_ros/feasibility_analytic.h>
#include<mav_trajectory_generation_ros/ros_conversions.h>


int main(int argc,char** argv)
{

    ros::init(argc,argv,"my_node");
    ros::NodeHandle nh;
    ros::Publisher traj_pub = nh.advertise<visualization_msgs::MarkerArray>("trajectory", 10);
    ros::Publisher polynomial_trajectory_pub_ = nh.advertise<mav_planning_msgs::PolynomialTrajectory4D>("path_segments", 1);
    ros::Rate sleep_Rate(1);

    while(ros::ok())
    {
        mav_trajectory_generation::Vertex::Vector vertices;
        const int dimension = 3;
        const int derivative_to_optimize = mav_trajectory_generation::derivative_order::ACCELERATION;
        mav_trajectory_generation::Vertex start(dimension), middle(dimension), end(dimension);

        start.makeStartOrEnd(Eigen::Vector3d(0,0,1), derivative_to_optimize);
        vertices.push_back(start);

        middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(1,0,1));
        vertices.push_back(middle);

        middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(1,0,2));
        vertices.push_back(middle);

        end.makeStartOrEnd(Eigen::Vector3d(1,1,2), derivative_to_optimize);
        vertices.push_back(end);

        std::vector<double> segment_times;
        const double v_max = 2.0;
        const double a_max = 2.0;
        const double magic_fabian_constant = 6.5; // A tuning parameter.
        segment_times = estimateSegmentTimes(vertices, v_max, a_max);

        const int N = 10;
        mav_trajectory_generation::PolynomialOptimization<N> opt(dimension);
        opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
        opt.solveLinear();

        mav_trajectory_generation::Segment::Vector segments;
        opt.getSegments(&segments);

        mav_trajectory_generation::Trajectory trajectory;
        opt.getTrajectory(&trajectory);

        double sampling_time = 2.0;
        int derivative_order = mav_trajectory_generation::derivative_order::POSITION;
        Eigen::VectorXd sample = trajectory.evaluate(sampling_time, derivative_order);

        double t_start = 2.0;
        double t_end = 10.0;
        double dt = 0.01;
        std::vector<Eigen::VectorXd> result;
        std::vector<double> sampling_times; // Optional.
        trajectory.evaluateRange(t_start, t_end, dt, derivative_order, &result, &sampling_times);

        visualization_msgs::MarkerArray markers;
        double distance = 1.0; // Distance by which to seperate additional markers. Set 0.0 to disable.
        std::string frame_id = "world";

        mav_trajectory_generation::drawMavTrajectory(trajectory, distance, frame_id, &markers);
        mav_planning_msgs::PolynomialTrajectory4D msg;
        mav_trajectory_generation::trajectoryToPolynomialTrajectoryMsg(trajectory,&msg);
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "world";
        polynomial_trajectory_pub_.publish(msg);
        
        ros::spinOnce();

        traj_pub.publish(markers);

        vertices.clear();
        sleep_Rate.sleep();

    }
    return 0;
}