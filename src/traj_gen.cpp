#include <ros/ros.h>
#include "quadrotor_msgs/TrajectoryPoint.h"
#include <traj_utils/PolyTraj.h>
#include "geometry_msgs/PoseStamped.h"
#include <Eigen/Dense>
#include <string>

std::string uav_id_;
geometry_msgs::PoseStamped nav_goal_nwu_, uav_pose_nwu_;
bool nav_goal_received_ = false;
bool traj_ready = false;
double nominal_velocity_;
traj_utils::PolyTraj poly_msg_;
ros::Time start_time_;
Eigen::MatrixXd coefficient_(6, 3);
quadrotor_msgs::TrajectoryPoint traj_sp_nwu_;

ros::Publisher traj_nwu_pub;

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

void cmdCallback(const ros::TimerEvent &)
{
    if (!traj_ready)
    {
        return;
    }

    ros::Time time_now = ros::Time::now();

    double t_curr = (time_now - start_time_).toSec();

    Eigen::Vector3d pos(Eigen::Vector3d::Zero());
    Eigen::Vector3d vel(Eigen::Vector3d::Zero());
    Eigen::Vector3d acc(Eigen::Vector3d::Zero());

    Eigen::VectorXd beta0(6), beta1(6), beta2(6);

    double duration = poly_msg_.duration[0];

    if (t_curr < duration)
    {
        EvaluateBasis(beta0, t_curr, 0);
        EvaluateBasis(beta1, t_curr, 1);
        EvaluateBasis(beta2, t_curr, 2);
        pos = beta0.transpose() * coefficient_;
        vel = beta1.transpose() * coefficient_;
        acc = beta2.transpose() * coefficient_;
    }

    else
    {
        EvaluateBasis(beta0, duration, 0);
        EvaluateBasis(beta1, duration, 1);
        EvaluateBasis(beta2, duration, 2);
        pos = beta0.transpose() * coefficient_;
        vel = beta1.transpose() * coefficient_;
        acc = beta2.transpose() * coefficient_;
    }

    pos = pos.transpose();
    vel = vel.transpose();
    acc = acc.transpose();

    traj_sp_nwu_.position.x = pos.x();
    traj_sp_nwu_.position.y = pos.y();
    traj_sp_nwu_.position.z = pos.z();

    traj_sp_nwu_.velocity.x = vel.x();
    traj_sp_nwu_.velocity.y = vel.y();
    traj_sp_nwu_.velocity.z = vel.z();

    traj_sp_nwu_.acceleration.x = acc.x();
    traj_sp_nwu_.acceleration.y = acc.y();
    traj_sp_nwu_.acceleration.z = acc.z();

    traj_nwu_pub.publish(traj_sp_nwu_);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "traj_gen");
    ros::NodeHandle nh("~");

    nh.param<double>("nominal_velocity", nominal_velocity_, 1.0);
    nh.param<std::string>("uav_id", uav_id_, "drone0");

    ros::Subscriber nav_goal_sub = nh.subscribe<geometry_msgs::PoseStamped>(
        "/goal", 1, NavGoalCallback);

    ros::Subscriber uav_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(
        "/" + uav_id_ + "/global_nwu", 1, UavPoseCallback);

    ros::Publisher poly_traj_pub = nh.advertise<traj_utils::PolyTraj>(
        "/" + uav_id_ + "/poly_traj", 10);

    ros::Rate rate(20.0);

    traj_nwu_pub = nh.advertise<quadrotor_msgs::TrajectoryPoint>(
        "/" + uav_id_ + "/trajectory_point/nwu", 1);

    ros::Timer cmd_timer = nh.createTimer(ros::Duration(0.01), cmdCallback);

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

            coefficient_ = equality_A.inverse() * equality_b;

            std::cout << coefficient_ << std::endl;

            start_time_ = ros::Time::now();

            poly_msg_.drone_id = 0;
            poly_msg_.traj_id = 1;
            poly_msg_.start_time = ros::Time::now();
            poly_msg_.order = 5; // todo, only support order = 5 now.
            poly_msg_.duration.resize(1);
            poly_msg_.coef_x.resize(6 * 1);
            poly_msg_.coef_y.resize(6 * 1);
            poly_msg_.coef_z.resize(6 * 1);
            poly_msg_.duration[0] = time_duration;

            for (int i = 0; i < 6; i++)
            {
                poly_msg_.coef_x[i] = coefficient_(i, 0);
                poly_msg_.coef_y[i] = coefficient_(i, 1);
                poly_msg_.coef_z[i] = coefficient_(i, 2);
            }

            poly_traj_pub.publish(poly_msg_);

            // reset the flag
            nav_goal_received_ = false;
            traj_ready = true;
        }

        else
        {
            // std::cout << "waiting for nav goal\n";
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}