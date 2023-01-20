#include <ros/ros.h>
#include "quadrotor_msgs/TrajectoryPoint.h"
#include "geometry_msgs/PoseStamped.h"
#include <Eigen/Dense>

geometry_msgs::PoseStamped nav_goal_nwu_, uav_pose_nwu_;
bool nav_goal_received_ = false;
double nominal_velocity_;

void NavGoalCallback(const geometry_msgs::PoseStamped::ConstPtr &nav_goal)
{
    nav_goal_received_ = true;
    nav_goal_nwu_ = *nav_goal;
}

void UavPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &uav_pose)
{
    uav_pose_nwu_ = *uav_pose;
}

void EvaluateBasis(Eigen::VectorXd &basis, double time, int order)
{
    double time2 = time * time;
    double time3 = time2 * time;
    double time4 = time3 * time;
    double time5 = time4 * time;

    switch (order)
    {
    case 0:
    {
        basis << 1, time, time2, time3, time4, time5;
        break;
    }

    case 1:
    {
        basis << 0, 1, 2 * time, 3 * time2, 4 * time3, 5 * time4;
        break;
    }

    case 2:
    {
        basis << 0, 0, 2, 6 * time, 12 * time2, 20 * time3;
        break;
    }

    default:
        break;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "traj_gen");
    ros::NodeHandle nh("~");

    nh.param<double>("nomial_velocity", nominal_velocity_, 1.0);

    ros::Subscriber nav_goal_sub = nh.subscribe<geometry_msgs::PoseStamped>(
        "/goal", 1, NavGoalCallback);

    ros::Subscriber uav_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(
        "/drone0/global_nwu", 1, UavPoseCallback);

    ros::Rate rate(20.0);

    while (ros::ok())
    {
        if (nav_goal_received_)
        {
            // generate trajectory and publish
            Eigen::Vector3d current_position, target_position;
            current_position.x() = uav_pose_nwu_.pose.position.x;
            current_position.y() = uav_pose_nwu_.pose.position.y;
            current_position.z() = uav_pose_nwu_.pose.position.z;

            target_position.x() = nav_goal_nwu_.pose.position.x;
            target_position.y() = nav_goal_nwu_.pose.position.y;
            target_position.z() = nav_goal_nwu_.pose.position.z;

            double time_duration = (target_position - current_position).norm() / nominal_velocity_;

            Eigen::MatrixXd equality_A = Eigen::MatrixXd::Zero(6, 6);
            Eigen::MatrixXd equality_b = Eigen::MatrixXd::Zero(6, 3);
            Eigen::MatrixXd coefficient(6, 3);

            // set up equality b matrix
            equality_b.row(0) = current_position.transpose();
            equality_b.row(3) = target_position.transpose();

            // setup equality A matrix
            Eigen::VectorXd beta0(6), beta1(6), beta2(6);

            EvaluateBasis(beta0, 0, 0);
            EvaluateBasis(beta1, 0, 1);
            EvaluateBasis(beta2, 0, 2);

            equality_A.row(0) = beta0;
            equality_A.row(1) = beta1;
            equality_A.row(2) = beta2;

            EvaluateBasis(beta0, time_duration, 0);
            EvaluateBasis(beta1, time_duration, 1);
            EvaluateBasis(beta2, time_duration, 2);

            equality_A.row(3) = beta0;
            equality_A.row(4) = beta1;
            equality_A.row(5) = beta2;

            coefficient = equality_A.inverse() * equality_b;

            // reset the flag
            nav_goal_received_ = false;
        }

        else
        {
            std::cout << "waiting for nav goal\n";
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}