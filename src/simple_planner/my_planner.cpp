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
#include "utility.h"

namespace robomaster
{

class LocalPlanner
{
public:
    LocalPlanner(ros::NodeHandle &given_nh) : nh(given_nh), plan_(false), prune_index_(0)
    {

        nh.param<double>("max_speed", max_speed_, 2.0);
        double max_angle_diff;
        nh.param<double>("max_angle_diff", max_angle_diff, 60);
        max_angle_diff_ = max_angle_diff * M_PI / 180;
        nh.param<double>("p_coeff", p_coeff_, 10.0);
        nh.param<int>("plan_frequency", plan_freq_, 50);
        nh.param<double>("goal_tolerance", goal_tolerance_, 0.1);
        nh.param<double>("prune_ahead_distance", prune_ahead_dist_, 0.3);
        nh.param<std::string>("global_frame", global_frame_, "odom");

        tf_listener_ = std::make_shared<tf::TransformListener>();

        cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

        global_path_sub_ = nh.subscribe("/global_planner/path", 5, &LocalPlanner::GlobalPathCallback, this);
        plan_timer_ = nh.createTimer(ros::Duration(1.0 / plan_freq_), &LocalPlanner::Plan, this);
        ROS_INFO("Init finished");
    }
    ~LocalPlanner() = default;
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

            geometry_msgs::Twist cmd_vel = GenCmdVel(robot_pose, global_path_.poses[prune_index_]);
            cmd_vel_pub_.publish(cmd_vel);
        }
    }

private:

    geometry_msgs::Twist GenCmdVel(const geometry_msgs::PoseStamped &robot_pose, const geometry_msgs::PoseStamped &goal)
    {
        // std::cout << "robot: " << robot_pose.pose.position.x << " " << robot_pose.pose.position.y << std::endl;
        // std::cout << "goal: " << goal.pose.position.x << " " << goal.pose.position.y << std::endl;
        double dx = goal.pose.position.x - robot_pose.pose.position.x;
        double dy = goal.pose.position.y - robot_pose.pose.position.y;
        double speed = sqrt(pow(dx, 2) + pow(dy, 2));
        // std::cout << "dist: " << dx << " " << dy << std::endl;

        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.y = -dx / speed * max_speed_;
        cmd_vel.linear.x = dy / speed * max_speed_;
        cmd_vel.angular.z = 0;
        return cmd_vel;
    }
    // update prune_index when arriving at a certain point
    void NextPose(geometry_msgs::PoseStamped &robot_pose, nav_msgs::Path &path, int &prune_index, double prune_ahead_dist)
    {
        double dist = GetEuclideanDistance(robot_pose, path.poses[prune_index]);
        if(dist <= prune_ahead_dist)
        {
            prune_index += 1;
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
    double max_angle_diff_;
    double p_coeff_;
    double goal_tolerance_;
    double prune_ahead_dist_;
    int plan_freq_;
};
} // namespace robomaster

using namespace robomaster;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "my_planner");
    ros::NodeHandle nh("~");
    LocalPlanner local_planner(nh);
    ros::spin();
    return 0;
}
