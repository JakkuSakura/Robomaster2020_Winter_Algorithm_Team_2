/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty ofÃ‚Â 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.Ã‚Â  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

#include <Eigen/Eigen>
#include <chrono>
#include <cmath>
#include <iostream>
#include <complex>
#include <cmath>
#include "utility.h"
#include "pid.h"
#include "pid.c"
namespace robomaster
{
inline double limit(double v, double abs_limit)
{
    if (v > 0)
        return std::min(v, abs_limit);
    else
        return std::max(v, -abs_limit);
}
class MyPlanner
{
public:
    MyPlanner(ros::NodeHandle &given_nh) : nh(given_nh), plan_(false), prune_index_(0)
    {
        pid_init(&pid_x_);
        pid_init(&pid_y_);
        pid_init(&pid_z_);
        // TODO 机器人运动最高线速度为 2m/s, 最高角速度为 180°/s, 最高线加速度为 2m/(s^2), 最高角加速度为 180°/(s^2)

        nh.param<double>("max_speed", max_speed_, 2.0);
        nh.param<double>("max_x_speed", max_x_speed_, 2.0);
        nh.param<double>("max_y_speed", max_y_speed_, 2.0);
        double max_angle_diff;
        nh.param<double>("max_angle_diff", max_angle_diff, 180);
        max_angle_diff_ = max_angle_diff * M_PI / 180;

        nh.param<float>("x_p_coeff", pid_x_.kp, 10.0);
        nh.param<float>("x_i_coeff", pid_x_.ki, 0.0);
        nh.param<float>("x_d_coeff", pid_x_.kd, 0.0);
        nh.param<float>("x_i_limit", pid_x_.integrator_limit, 0.0);

        nh.param<float>("y_p_coeff", pid_y_.kp, 10.0);
        nh.param<float>("y_i_coeff", pid_y_.ki, 0.0);
        nh.param<float>("y_d_coeff", pid_y_.kd, 0.0);
        nh.param<float>("y_i_limit", pid_y_.integrator_limit, 0.0);

        nh.param<float>("z_p_coeff", pid_z_.kp, 10.0);
        nh.param<float>("z_i_coeff", pid_z_.ki, 0.0);
        nh.param<float>("z_d_coeff", pid_z_.kd, 0.0);
        nh.param<float>("z_i_limit", pid_z_.integrator_limit, 0.0);

        nh.param<int>("plan_frequency", plan_freq_, 50);
        nh.param<double>("goal_tolerance", goal_tolerance_, 0.1);
        nh.param<double>("prune_ahead_distance", prune_ahead_dist_, 0.3);
        nh.param<double>("goal_angle_tolerance", goal_angle_tolerance_, 0.05);
        goal_angle_tolerance_ = goal_angle_tolerance_ * M_PI / 180;

        nh.param<std::string>("global_frame", global_frame_, "odom");

        tf_listener_ = std::make_shared<tf::TransformListener>();

        cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

        global_path_sub_ = nh.subscribe("/global_planner/path", 5, &MyPlanner::GlobalPathCallback, this);
        plan_timer_ = nh.createTimer(ros::Duration(1.0 / plan_freq_), &MyPlanner::Plan, this);
    }
    ~MyPlanner() = default;
    void GlobalPathCallback(const nav_msgs::PathConstPtr &msg)
    {
        ROS_INFO("Received path");

        if (!msg->poses.empty())
        {
            global_path_ = *msg;
            prune_index_ = 0;
            plan_ = true;
        }
    }

private:
    void Plan(const ros::TimerEvent &event)
    {
        if (plan_)
        {
            auto begin = std::chrono::steady_clock::now();

            // 1. Update the transform from global path frame to local planner frame
            UpdateTransform(tf_listener_, global_frame_,
                            global_path_.header.frame_id, global_path_.header.stamp,
                            path2global_transform_); //source_time needs decided

            // 2. Get current robot pose in global path frame
            geometry_msgs::PoseStamped robot_pose;
            GetGlobalRobotPose(tf_listener_, global_path_.header.frame_id, robot_pose);

            // 3. Check if robot has already arrived with given distance tolerance
            if (GetEuclideanDistance(robot_pose, global_path_.poses.back()) <= goal_tolerance_ && prune_index_ == global_path_.poses.size() - 1)
            {
                plan_ = false;
                geometry_msgs::Twist cmd_vel;
                cmd_vel.linear.x = 0;
                cmd_vel.linear.y = 0;
                cmd_vel.angular.z = 0;
                cmd_vel_pub_.publish(cmd_vel);
                ROS_INFO("Planning Success!");
                return;
            }

            // 4. Get prune index from given global path
            NextPose(robot_pose, global_path_, prune_index_, prune_ahead_dist_);

            GenCmdVel(global_path_.poses[prune_index_]);
        }
    }

private:
    void GenCmdVel(geometry_msgs::PoseStamped &goal)
    {
        geometry_msgs::PoseStamped goal_pose;
        goal.header.stamp = ros::Time::now();
        //1. Get goal pose in the base_link frame
        if (!TransformPose(tf_listener_, "base_link", goal, goal_pose))
        {
            ROS_WARN("Error: cannot get goal pose in base_link frame");
            return;
        }
        //2. Get yaw angle difference
        double diff_yaw = atan2(goal_pose.pose.position.y, goal_pose.pose.position.x);

        geometry_msgs::Twist cmd_vel;

        if (abs(diff_yaw) < goal_angle_tolerance_)
        {
            cmd_vel.linear.x = max_x_speed_;
            cmd_vel.linear.y = limit(pid_process(&pid_y_, -goal_pose.pose.position.y), max_y_speed_);
            double speed = sqrt(pow(cmd_vel.linear.x, 2) + pow(cmd_vel.linear.y, 2));

            cmd_vel.linear.x = cmd_vel.linear.x / speed * max_x_speed_;
            cmd_vel.linear.y = cmd_vel.linear.y / speed * max_y_speed_;

            cmd_vel.angular.z = limit(pid_process(&pid_z_, -diff_yaw), max_angle_diff_);
        }
        else if (abs(diff_yaw) < M_PI_2)
        {
            cmd_vel.linear.x = limit(pid_process(&pid_x_, -goal_pose.pose.position.x), max_x_speed_);
            cmd_vel.linear.y = 0;
            cmd_vel.angular.z = max_angle_diff_ * diff_yaw / abs(diff_yaw);
        }
        else
        {
            cmd_vel.linear.x = -max_speed_;
            cmd_vel.linear.y = 0;
            cmd_vel.angular.z = max_angle_diff_ * diff_yaw / abs(diff_yaw);
        }

        cmd_vel_pub_.publish(cmd_vel);
    }
    // update prune_index when arriving at a certain point
    void NextPose(geometry_msgs::PoseStamped &robot_pose, nav_msgs::Path &path, int &prune_index, double prune_ahead_dist)
    {
        double dist = GetEuclideanDistance(robot_pose, path.poses[prune_index]);
        if (dist <= prune_ahead_dist)
        {
            prune_index += 1;
            pid_reset_integral(&pid_x_);
            pid_reset_integral(&pid_y_);
            pid_reset_integral(&pid_z_);
        }
        prune_index = std::min(prune_index, (int)(path.poses.size() - 1));
    }

    ros::NodeHandle nh;
    std::shared_ptr<tf::TransformListener> tf_listener_;
    tf::StampedTransform path2global_transform_;

    std::string global_frame_;
    ros::Timer plan_timer_;

    ros::Subscriber global_path_sub_;
    ros::Publisher cmd_vel_pub_;

    bool plan_;
    int prune_index_;
    nav_msgs::Path global_path_;

    double max_speed_;
    double max_x_speed_;
    double max_y_speed_;
    double max_angle_diff_;

    double goal_tolerance_;
    double prune_ahead_dist_;
    double goal_angle_tolerance_;
    int plan_freq_;
    pid_ctrl_t pid_x_;
    pid_ctrl_t pid_y_;
    pid_ctrl_t pid_z_;
};
} // namespace robomaster

using namespace robomaster;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "my_planner");
    ros::NodeHandle nh("~");
    MyPlanner local_planner(nh);
    ros::spin();
    return 0;
}
