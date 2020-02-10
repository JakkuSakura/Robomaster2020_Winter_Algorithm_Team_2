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
#include <deque>
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
inline double limit(double now, double last, double abs_limit_acc, double delta)
{
    double acc = limit((now - last) / delta, abs_limit_acc);
    return last + acc * delta;
}
inline void show_pose(const geometry_msgs::Pose &pose)
{
    printf("p (%.10lf,%.10lf,%.10lf) o (%.10lf,%.10lf,%.10lf,%.10lf)",
           pose.position.x,
           pose.position.y,
           pose.position.z,
           pose.orientation.x,
           pose.orientation.y,
           pose.orientation.z,
           pose.orientation.w);
}
inline bool same_pose(const geometry_msgs::Pose &p1, const geometry_msgs::Pose &p2, double tolerance)
{
    double px1 = p1.position.x, px2 = p2.position.x;
    double py1 = p1.position.y, py2 = p2.position.y;
    double pz1 = p1.position.z, pz2 = p2.position.z;
    double ox1 = p1.orientation.x, ox2 = p2.orientation.x;
    double oy1 = p1.orientation.y, oy2 = p2.orientation.y;
    double oz1 = p1.orientation.z, oz2 = p2.orientation.z;
    double ow1 = p1.orientation.w, ow2 = p2.orientation.w;

    bool flag = std::abs(px1 - px2) < tolerance &&
                std::abs(py1 - py2) < tolerance &&
                std::abs(pz1 - px2) < tolerance &&
                std::abs(ox1 - ox2) < tolerance &&
                std::abs(oy1 - oy2) < tolerance &&
                std::abs(oz1 - oz2) < tolerance &&
                std::abs(ow1 - ow2) < tolerance;

    return flag;
}

template <typename T>
int sign(T val)
{
    return (T(0) < val) - (val < T(0));
}

class MyPlanner
{
public:
    MyPlanner(ros::NodeHandle &given_nh) : nh(given_nh), plan_(false), prune_index_(0)
    {
        pid_init(&pid_x_);
        pid_init(&pid_y_);
        pid_init(&pid_z_);

        
        nh.param<double>("stuck_vel_error", stuck_vel_error_, 0.1);

        nh.param<double>("max_speed", max_speed_, 2.0);
        nh.param<double>("max_acceleration", max_acceleration_, 2.0);

        nh.param<double>("max_x_speed", max_x_speed_, 2.0);
        nh.param<double>("max_y_speed", max_y_speed_, 0.1);

        nh.param<double>("max_angle_diff", max_angle_diff_, 180);
        max_angle_diff_ = max_angle_diff_ * M_PI / 180;

        nh.param<double>("max_angular_acceleration_", max_angular_acceleration_, 180);
        max_angular_acceleration_ = max_angular_acceleration_ * M_PI / 180;

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

        nh.param<int>("plan_frequency", plan_freq_, 20);
        time_delta_ = 1.0 / plan_freq_;

        nh.param<double>("goal_tolerance", goal_tolerance_, 0.1);
        nh.param<double>("prune_ahead_distance", prune_ahead_dist_, 0.3);
        nh.param<double>("rush_angle_tolerance", rush_angle_tolerance_, 30);
        rush_angle_tolerance_ = rush_angle_tolerance_ * M_PI / 180;

        // nh.param<double>("turn_angle_tolerance", turn_angle_tolerance_, 60);
        // turn_angle_tolerance_ = turn_angle_tolerance_ * M_PI / 180;

        nh.param<std::string>("global_frame", global_frame_, "odom");

        tf_listener_ = std::make_shared<tf::TransformListener>();

        cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

        global_path_sub_ = nh.subscribe("/global_planner/path", 5, &MyPlanner::fetch_global_path, this);
        odom_sub_ = nh.subscribe("/odom", 100, &MyPlanner::fetch_odem, this);
        plan_timer_ = nh.createTimer(ros::Duration(time_delta_), &MyPlanner::plan, this);
    }
    ~MyPlanner() = default;

private:
    void fetch_global_path(const nav_msgs::PathConstPtr &msg)
    {
        ROS_INFO("Received path");

        if (!msg->poses.empty())
        {
            global_path_ = *msg;
            prune_index_ = 0;
            plan_ = true;
        }
    }
    void plan(const ros::TimerEvent &event)
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
                publish_velocity(cmd_vel);
                ROS_INFO("Planning Success!");
                return;
            }

            // 4. Get prune index from given global path
            next_pose(robot_pose, global_path_, prune_index_, prune_ahead_dist_);

            geometry_msgs::PoseStamped global_robot_pose;
            global_robot_pose.header.frame_id = global_frame_;

            TransformPose(path2global_transform_, robot_pose, global_robot_pose);

            generate_velocity_command(global_robot_pose, global_path_.poses[prune_index_]);
        }
    }
    void fetch_odem(const nav_msgs::Odometry::ConstPtr &msg)
    {
        pose_and_twist_.push_back(*msg);
        while(ros::Time::now() - pose_and_twist_.front().header.stamp > ros::Duration(1.0))
            pose_and_twist_.pop_front();
    }

    // the robot pose must be in the same frame as pose_and_twist, aka global_frame or odom frame
    void generate_velocity_command(const geometry_msgs::PoseStamped &robot, geometry_msgs::PoseStamped &goal)
    {
        double last_cmd_speed = sqrt(pow(last_cmd_vel_.linear.x, 2) + pow(last_cmd_vel_.linear.y, 2));

        bool is_stuck = last_cmd_speed > stuck_vel_error_ && same_pose(pose_and_twist_.front().pose.pose, pose_and_twist_.back().pose.pose, 1e-5);

        if(is_stuck)
        {
            ROS_WARN("Got stuck! cmd_speed.linear.x=%lf", last_cmd_vel_.linear.x);
            geometry_msgs::Twist cmd_vel;
            cmd_vel.linear.x = -max_speed_ * sign(last_cmd_vel_.linear.x);
            cmd_vel.linear.y = 0;
            cmd_vel.angular.z = 0;
            publish_velocity(cmd_vel);
            ros::Duration(0.5).sleep();
            return;
        }

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
        if (std::isnan(diff_yaw))
        {
            ROS_WARN("Too close, not adjusting");
            return;
        }

        geometry_msgs::Twist cmd_vel;

        if (std::abs(diff_yaw) < rush_angle_tolerance_)
        {
            cmd_vel.linear.x = max_x_speed_;
            cmd_vel.linear.y = limit(pid_process(&pid_y_, -goal_pose.pose.position.y), max_y_speed_);

            double speed = sqrt(pow(cmd_vel.linear.x, 2) + pow(cmd_vel.linear.y, 2));

            cmd_vel.linear.x = cmd_vel.linear.x / speed * max_x_speed_;
            cmd_vel.linear.y = cmd_vel.linear.y / speed * max_y_speed_;
            cmd_vel.angular.z = limit(pid_process(&pid_z_, -diff_yaw), max_angle_diff_);
        }
        else if (std::abs(diff_yaw) < M_PI_2)
        {

            cmd_vel.linear.x = limit(pid_process(&pid_x_, -goal_pose.pose.position.x), max_x_speed_);
            cmd_vel.linear.y = 0;
            cmd_vel.angular.z = max_angle_diff_ * sign(diff_yaw);
        }
        else
        {
            cmd_vel.linear.x = -max_speed_;
            cmd_vel.linear.y = 0;
            cmd_vel.angular.z = max_angle_diff_ * sign(diff_yaw);
        }

        limit_speed(cmd_vel);

        publish_velocity(cmd_vel);
    }
    void limit_speed(geometry_msgs::Twist &cmd_vel)
    {

        cmd_vel.linear.x = limit(cmd_vel.linear.x, last_cmd_vel_.linear.x, max_acceleration_, time_delta_);
        cmd_vel.linear.y = limit(cmd_vel.linear.y, last_cmd_vel_.linear.y, max_acceleration_, time_delta_);

        // cmd_vel.angular.z = limit(cmd_vel.angular.z, last_cmd_vel_.angular.z, max_angular_acceleration_, time_delta_);
    }
    void publish_velocity(const geometry_msgs::Twist &vel)
    {
        last_cmd_vel_ = vel;
        cmd_vel_pub_.publish(vel);
    }

    // update prune_index when arriving at a certain point
    void next_pose(geometry_msgs::PoseStamped &robot_pose, nav_msgs::Path &path, int &prune_index, double prune_ahead_dist)
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

    ros::Subscriber odom_sub_;
    ros::Subscriber global_path_sub_;
    ros::Publisher cmd_vel_pub_;

    bool plan_;
    int prune_index_;
    nav_msgs::Path global_path_;

    geometry_msgs::Twist last_cmd_vel_;
    std::deque<nav_msgs::Odometry> pose_and_twist_;

    double max_speed_;
    double max_acceleration_;
    double max_x_speed_;
    double max_y_speed_;
    double max_angle_diff_;
    double max_angular_acceleration_;

    double goal_tolerance_;
    double prune_ahead_dist_;
    double rush_angle_tolerance_;
    // double turn_angle_tolerance_;
    int plan_freq_;
    double time_delta_;
    pid_ctrl_t pid_x_;
    pid_ctrl_t pid_y_;
    pid_ctrl_t pid_z_;
    double stuck_vel_error_;
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
