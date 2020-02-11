#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>

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
#include "graph.h"
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
    printf(__FILE__" p (%.2lf,%.2lf,%.2lf) o (%.2lf,%.2lf,%.2lf,%.2lf)\n",
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
                std::abs(pz1 - pz2) < tolerance &&
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

        nh.param<double>("stuck_vel_threshold", stuck_vel_threshold_, 0.5);

        nh.param<double>("stuck_detection_time", stuck_detection_time_, 0.2);
        nh.param<double>("stuck_back_up_time", stuck_back_up_time_, 0.5);

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
        nh.param<double>("rush_dist_tolerance", rush_dist_tolerance_, 1);

        nh.param<double>("soft_turn_angle_tolerance", soft_turn_angle_tolerance_, 60);
        soft_turn_angle_tolerance_ = soft_turn_angle_tolerance_ * M_PI / 180;

        nh.param<std::string>("global_frame", global_frame_, "odom");

        tf_listener_ = std::make_shared<tf::TransformListener>();

        cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

        status_pub_ = nh.advertise<std_msgs::String>("/local_planner/status", 1);

        global_path_sub_ = nh.subscribe("/global_planner/path", 5, &MyPlanner::fetch_global_path, this);

        odom_sub_ = nh.subscribe("/odom", 100, &MyPlanner::fetch_odem, this);
        plan_timer_ = nh.createTimer(ros::Duration(time_delta_), &MyPlanner::plan, this);
        reset_cmd_vel_ = false;
    }
    ~MyPlanner() = default;

private:
    void plan(const ros::TimerEvent &event)
    {
        if (plan_)
        {
            if (ros::Time::now() < last_cmd_vel_expiring_time_)
                return;
            if (reset_cmd_vel_)
            {
                publish_velocity(geometry_msgs::Twist());
                reset_cmd_vel_ = false;
            }
            // 1. Update the transform from global path frame to local planner frame
            UpdateTransform(tf_listener_, global_frame_,
                            global_path_.header.frame_id, global_path_.header.stamp,
                            path2global_transform_); //source_time needs decided

            geometry_msgs::PoseStamped global_robot_pose;
            global_robot_pose.header.stamp = ros::Time::now();
            global_robot_pose.header.frame_id = global_frame_;
            global_robot_pose.pose = pose_and_twist_.front().pose.pose;

            // 3. Check if robot has already arrived with given distance tolerance
            if (GetEuclideanDistance(global_robot_pose, global_path_.poses.back()) <= goal_tolerance_ && prune_index_ == global_path_.poses.size() - 1)
            {
                plan_ = false;
                geometry_msgs::Twist cmd_vel;
                cmd_vel.linear.x = 0;
                cmd_vel.linear.y = 0;
                cmd_vel.angular.z = 0;
                publish_velocity(cmd_vel);
                std_msgs::String msg;
                msg.data = "done";
                status_pub_.publish(msg);

                ROS_INFO("Planning Success!");
                return;
            }

            // Get prune index from given global path
            next_pose(global_robot_pose, global_path_, prune_index_, prune_ahead_dist_);

            generate_velocity_command(global_robot_pose, global_path_.poses[prune_index_]);
        }
    }
    void fetch_global_path(const nav_msgs::PathConstPtr &msg)
    {
        ROS_INFO("Received path");

        if (!msg->poses.empty())
        {
            global_path_ = *msg;
            prune_index_ = 0;
            plan_ = true;
            for (size_t i = 0; i < global_path_.poses.size(); i++)
            {
                geometry_msgs::PoseStamped goal_pose;
                if (!TransformPose(tf_listener_, global_frame_, global_path_.poses[i], goal_pose))
                {
                    ROS_WARN("Error: cannot get goal pose in global_frame_ frame");
                }
                show_pose(global_path_.poses[i].pose);
                global_path_.poses[i] = goal_pose;
            }
        }
    }
    void fetch_odem(const nav_msgs::Odometry::ConstPtr &msg)
    {
        pose_and_twist_.push_front(*msg);
        while (ros::Time::now() - pose_and_twist_.back().header.stamp > ros::Duration(stuck_detection_time_))
            pose_and_twist_.pop_back();
    }

    // the robot pose must be in the same frame as pose_and_twist, aka global_frame or odom frame
    void generate_velocity_command(const geometry_msgs::PoseStamped &robot, geometry_msgs::PoseStamped &goal)
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
        if (std::isnan(diff_yaw))
        {
            ROS_WARN("Too close, not adjusting");
            return;
        }

        double last_cmd_speed = hypot(last_cmd_vel_.linear.x, last_cmd_vel_.linear.y);

        bool is_stuck = last_cmd_speed > stuck_vel_threshold_ && same_pose(pose_and_twist_.front().pose.pose, pose_and_twist_.back().pose.pose, 1e-5);

        if (is_stuck)
        {
            ROS_WARN("Got stuck! cmd_speed.linear.x=%lf", last_cmd_vel_.linear.x);
            geometry_msgs::Twist cmd_vel;
            cmd_vel.linear.x = -max_x_speed_ * sign(last_cmd_vel_.linear.x);
            cmd_vel.linear.y = max_y_speed_ * sign(goal_pose.pose.position.y);
            cmd_vel.angular.z = 0;

            publish_velocity(cmd_vel, stuck_back_up_time_);
            reset_cmd_vel_ = true;
            return;
        }

        double turning_angle = 0;
        if (prune_index_ + 1 < global_path_.poses.size())
        {
            const auto &next_goal = global_path_.poses[prune_index_ + 1];
            // double direction = GetYawFromOrientation(robot.pose.orientation);
            double direction = atan2(goal.pose.position.y - robot.pose.position.y, goal.pose.position.x - robot.pose.position.x);
            double direction_next = atan2(next_goal.pose.position.y - goal.pose.position.y, next_goal.pose.position.x - goal.pose.position.x);
            turning_angle = distance_in_radius(direction, direction_next);
        }

        double dist_goal = hypot(goal_pose.pose.position.x, goal_pose.pose.position.x);
        geometry_msgs::Twist cmd_vel;

        if (std::abs(diff_yaw) < rush_angle_tolerance_)
        {
            if (dist_goal > rush_dist_tolerance_ || turning_angle < soft_turn_angle_tolerance_)
                cmd_vel.linear.x = max_x_speed_;
            else
            {
                cmd_vel.linear.x = limit(pid_process(&pid_x_, -dist_goal), max_x_speed_);
            }
            cmd_vel.linear.y = limit(pid_process(&pid_y_, -goal_pose.pose.position.y), max_y_speed_);
            cmd_vel.angular.z = limit(pid_process(&pid_z_, -diff_yaw), max_angle_diff_);
        }
        else if (std::abs(diff_yaw) < M_PI_2)
        {
            if (dist_goal > rush_dist_tolerance_ || turning_angle < soft_turn_angle_tolerance_)
                cmd_vel.linear.x = max_x_speed_;
            else
            {
                cmd_vel.linear.x = limit(pid_process(&pid_x_, -dist_goal), max_x_speed_);
            }
            cmd_vel.linear.y = 0;
            cmd_vel.angular.z = max_angle_diff_ * sign(diff_yaw);
        }
        else
        {
            if (dist_goal > rush_dist_tolerance_ || turning_angle < soft_turn_angle_tolerance_)
                cmd_vel.linear.x = -max_x_speed_;
            else
            {
                cmd_vel.linear.x = -limit(pid_process(&pid_x_, -dist_goal), max_x_speed_);
            }
            cmd_vel.linear.y = 0;
            cmd_vel.angular.z = max_angle_diff_ * sign(diff_yaw);
        }

        limit_speed(cmd_vel);

        publish_velocity(cmd_vel);
    }
    void limit_speed(geometry_msgs::Twist &cmd_vel)
    {

        double speed = hypot(cmd_vel.linear.x, cmd_vel.linear.y);

        cmd_vel.linear.x = limit(cmd_vel.linear.x, last_cmd_vel_.linear.x, max_acceleration_, time_delta_);
        cmd_vel.linear.y = limit(cmd_vel.linear.y, last_cmd_vel_.linear.y, max_acceleration_, time_delta_);

        // cmd_vel.angular.z = limit(cmd_vel.angular.z, last_cmd_vel_.angular.z, max_angular_acceleration_, time_delta_);
    }

    void publish_velocity(const geometry_msgs::Twist &vel, double keep_time = 0)
    {
        last_cmd_vel_ = vel;
        last_cmd_vel_expiring_time_ = ros::Time::now() + ros::Duration(keep_time);
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
    ros::Publisher status_pub_;

    bool plan_;
    int prune_index_;
    nav_msgs::Path global_path_;

    geometry_msgs::Twist last_cmd_vel_;
    ros::Time last_cmd_vel_expiring_time_;
    bool reset_cmd_vel_;

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
    double rush_dist_tolerance_;
    double soft_turn_angle_tolerance_;
    int plan_freq_;
    double time_delta_;
    pid_ctrl_t pid_x_;
    pid_ctrl_t pid_y_;
    pid_ctrl_t pid_z_;
    double stuck_vel_threshold_;
    double stuck_detection_time_;
    double stuck_back_up_time_;
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
