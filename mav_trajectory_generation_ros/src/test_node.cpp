#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation/trajectory.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/PoseStamped.h"
#include "ros/ros.h"


geometry_msgs::PoseArray way_;
void waycb(const geometry_msgs::PoseArray::ConstPtr &msg)
{
    way_ = *msg;
}

float x,y,odom_detected_flag = 0;
void odomcb(const nav_msgs::Odometry::ConstPtr &msg)
{
    x = (msg->pose.pose.position.x);
    y = (msg->pose.pose.position.y);

    odom_detected_flag = 1;
}

float object_y,object_x,pick_goal_x,pick_goal_y;
int aruco_detected_flag = 0;
void arucocb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    object_y = -(msg->pose.position.x);
    object_x = -(msg->pose.position.y);

    pick_goal_x = x + object_x;
    pick_goal_y = y + object_y;

    aruco_detected_flag = 1;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_node");
    ros::NodeHandle n;
    ros::Subscriber odo_sub = n.subscribe<geometry_msgs::PoseArray>("/waypoints", 10, waycb);
    ros::Subscriber aruco_sub = n.subscribe<geometry_msgs::PoseStamped>("/aruco_single/pose", 10, arucocb);
    ros::Publisher vis_pub = n.advertise<visualization_msgs::MarkerArray>("trajectory_traject", 10);
    // rate
    ros::Rate sleep_rate(1);
    // int marker_size = markers.markers.size();
    // int trajectory_size = markers.markers[marker_size-1].points.size();
    // for (int i=0 ; i < trajectory_size ; i++)
    // std::cout<<markers.markers[marker_size-1].points[i]<<std::endl;

    while (ros::ok())
    {   
        float x_,y_,z_;
        n.getParam("/test_node/x_", x_);
        n.getParam("/test_node/y_", y_);
        n.getParam("/test_node/z_", z_);

        mav_trajectory_generation::Vertex::Vector vertices;
        const int dimension = 3;
        const int derivative_to_optimize = mav_trajectory_generation::derivative_order::ACCELERATION;
        mav_trajectory_generation::Vertex start(dimension), middle(dimension), end(dimension);

        // Time count
        ros::Time t0 = ros::Time::now();

        // Add constraints to the vertices
        // curve 1 in paper
        if(odom_detected_flag==1 && aruco_detected_flag==1)
        {
            start.makeStartOrEnd(Eigen::Vector3d(x, y,1), derivative_to_optimize);
            vertices.push_back(start);

            end.makeStartOrEnd(Eigen::Vector3d(pick_goal_x,pick_goal_y,1),derivative_to_optimize);
            vertices.push_back(end);
        }

        else
        {

            if(way_.poses.size()>0)
            start.makeStartOrEnd(Eigen::Vector3d(way_.poses[0].position.x, way_.poses[0].position.y,0), derivative_to_optimize);
            else
            start.makeStartOrEnd(Eigen::Vector3d(0, 0,1), derivative_to_optimize);
            vertices.push_back(start);

            if (way_.poses.size() > 0)
            {
            //std::cout << "debug=" << way_.poses[way_.poses.size() - 1].position.y << "," << way_.poses.size() - 1 << std::endl;
            middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(way_.poses[way_.poses.size()/2 - 1].position.x, way_.poses[way_.poses.size()/2 - 1].position.y, 0));
            vertices.push_back(middle);

            middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(way_.poses[way_.poses.size() - 1].position.x, way_.poses[way_.poses.size() - 1].position.y, 0));
            vertices.push_back(middle);
            }

    /*        middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(0., .6, 1));
            vertices.push_back(middle);
    	
    	middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(0, 1, 1));
            vertices.push_back(middle);
    	
    	middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(0., 1.5, 1));
            vertices.push_back(middle);
    	
    	middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(0., 1.7, 1));
            vertices.push_back(middle);
    */
            end.makeStartOrEnd(Eigen::Vector3d(0,5,1),derivative_to_optimize);
            vertices.push_back(end);
            // curve 2 in paper
            // start.makeStartOrEnd(Eigen::Vector3d(0, 0, 0), derivative_to_optimize);
            // vertices.push_back(start);

            // middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(-5, 5, 2.5));
            // vertices.push_back(middle);

            // middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(-7.07, 7.07, 2));
            // vertices.push_back(middle);

            // middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(0, 10, 2.5));
            // vertices.push_back(middle);

            // middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(7.07, 7.07, 1.5));
            // vertices.push_back(middle);

            // middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(10, 0, 2.5));
            // vertices.push_back(middle);

            // middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(7.07, -7.07, 2.5));
            // vertices.push_back(middle);

            // middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(0, -10, 2));
            // vertices.push_back(middle);

            // middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(-7.07, -7.07, 2.5));
            // vertices.push_back(middle);

            // middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(x_, y_, z_));
            // vertices.push_back(middle);

            // end.makeStartOrEnd(Eigen::Vector3d(0, 0, 0), derivative_to_optimize);
            // vertices.push_back(end);
        }

        //compute the segment times
        std::vector<double> segment_times;
        const double v_max = 1.0;
        const double a_max = 3.0;
        const double magic_fabian_constant = 6.5; // A tuning parameter
        segment_times = estimateSegmentTimes(vertices, v_max, a_max);


        //N denotes the number of coefficients of the underlying polynomial
        //N has to be even.
        //If we want the trajectories to be snap-continuous, N needs to be at least 10.

        const int N = 10;
        mav_trajectory_generation::PolynomialOptimization<N> opt(dimension);
        opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
        opt.solveLinear();

        ROS_INFO("Take %f sec to get optimal traject", (ros::Time::now() - t0).toSec());

        //Obtain the polynomial segments
        mav_trajectory_generation::Segment::Vector segments;
        opt.getSegments(&segments);

        //creating Trajectories
        
        mav_trajectory_generation::Trajectory trajectory;
        opt.getTrajectory(&trajectory);
        
        //evaluating the trajectory at particular instances of time
        // Single sample:
        double sampling_time = 2.0;
        int derivative_order = mav_trajectory_generation::derivative_order::POSITION;
        Eigen::VectorXd sample = trajectory.evaluate(sampling_time, derivative_order);

        // Sample range:
        double t_start = 2.0;
        double t_end = 10.0;
        double dt = 0.01;
        std::vector<Eigen::VectorXd> result;
        std::vector<double> sampling_times; // Optional.
        trajectory.evaluateRange(t_start, t_end, dt, derivative_order, &result, &sampling_times);
        //std::cout << result.size() << std::endl;
        //std::cout << sampling_times.size() << std::endl;

        //visualizing Trajectories
        visualization_msgs::MarkerArray markers;
        double distance = 1.6; // Distance by which to seperate additional markers. Set 0.0 to disable.
        std::string frame_id = "map";
        // From Trajectory class:
        mav_trajectory_generation::drawMavTrajectory(trajectory, distance, frame_id, &markers);

        // for (int i = 0; i < segments.size(); i++)
        // {
        //     std::cout << i << "," << segments[i] << "," <<  segments.size() << std::endl;
        // }
        // std::cout << result[1] << std::endl;
        ros::spinOnce();

        vis_pub.publish(markers);
        vertices.clear();
        sleep_rate.sleep();
    }
    return 0;
}