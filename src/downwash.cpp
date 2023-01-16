/**
 * @file downwash.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <string>
#include <iostream>
#include <vector>

mavros_msgs::State current_state;

void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "downwash");
    ros::NodeHandle nh("~");
    std::string m_uav_id_ = "drone";

    std::vector<double> take_off_position;
    std::vector<double> waypoint_1;
    std::vector<double> land_position;

    nh.param<std::string>("uav_id", m_uav_id_, "drone0");
    nh.getParam("take_off_position", take_off_position);
    nh.getParam("waypoint_1", waypoint_1);
    nh.getParam("land_position", land_position);

    std::cout << "/" + m_uav_id_ + "mavros/state" << std::endl;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/" + m_uav_id_ + "/mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/" + m_uav_id_ + "/mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/" + m_uav_id_ + "/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/" + m_uav_id_ + "/mavros/set_mode");

    // the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while (ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = take_off_position[0];
    pose.pose.position.y = take_off_position[1];
    pose.pose.position.z = take_off_position[2];

    // send a few setpoints before starting
    for (int i = 100; ros::ok() && i > 0; --i)
    {
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    bool first_pub = false;
    ros::Time first_pub_time;

    while (ros::ok())
    {
        if (current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(2.0)))
        {
            if (set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        }
        else
        {
            if (!current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(2.0)))
            {
                if (arming_client.call(arm_cmd) &&
                    arm_cmd.response.success)
                {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        if (!first_pub)
        {
            first_pub = true;
            first_pub_time = ros::Time::now();
        }

        double time_lapsed = (ros::Time::now() - first_pub_time).toSec();
        std::cout << "Time lapsed: " << time_lapsed << std::endl;

        if (50 < time_lapsed && time_lapsed < 100)
        {
            pose.pose.position.x = waypoint_1[0];
            pose.pose.position.y = waypoint_1[1];
            pose.pose.position.z = waypoint_1[2];
            // std::cout << "Position Setpoint: (5, 5, 2)\n";
        }

        else if (100 < time_lapsed && time_lapsed < 120)
        {
            pose.pose.position.x = land_position[0];
            pose.pose.position.y = land_position[1];
            pose.pose.position.z = land_position[2];
            // std::cout << "Position Setpoint: (0, 0, 2)\n";
        }

        else if (120 < time_lapsed)
        {
            pose.pose.position.x = land_position[0];
            pose.pose.position.y = land_position[1];
            pose.pose.position.z = 0;
            // std::cout << "Position Setpoint: (3, 3, 2)\n";
        }

        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}