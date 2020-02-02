/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>

#include <Eigen/Eigen>
#include <chrono>

#include "utility.h"


namespace robomaster{


class PIDPlanner{
 public:
  PIDPlanner(ros::NodeHandle& given_nh):nh(given_nh),plan_(false){

    nh.param<double>("max_x_speed", max_x_speed_, 1.0);
    nh.param<double>("max_y_speed", max_y_speed_, 1.0);
    nh.param<double>("max_yaw_speed", max_yaw_speed_, 2.0);
    nh.param<double>("p_x_coeff", p_x_coeff_, 1);
    nh.param<double>("p_y_coeff", p_y_coeff_, 1);
    nh.param<double>("p_yaw_coeff", p_yaw_coeff_, 1);
    nh.param<int>("plan_frequency", plan_freq_, 30);
    nh.param<double>("goal_dist_tolerance", goal_dist_tolerance_, 0.1);
    nh.param<double>("goal_angle_tolerance", goal_angle_tolerance_, 0.05);

    tf_listener_ = std::make_shared<tf::TransformListener>();

    cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    goal_sub_ = nh.subscribe("/move_base_simple/goal", 5, &PIDPlanner::GoalCallback,this);

    plan_timer_ = nh.createTimer(ros::Duration(1.0/plan_freq_),&PIDPlanner::Plan,this);

  }
  ~PIDPlanner()= default;
  void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr & msg){
   goal_=*msg;
   plan_=true;
  }
 private:

  void Plan(const ros::TimerEvent& event){

    if (plan_){

      geometry_msgs::PoseStamped goal_pose;
      goal_.header.stamp=ros::Time::now();
      //1. Get goal pose in the base_link frame
      if (!TransformPose(tf_listener_,"base_link", goal_, goal_pose)){
        return;
      }
      //2. Get yaw angle difference
      double diff_yaw = GetYawFromOrientation(goal_pose.pose.orientation);
      geometry_msgs::Twist cmd_vel;

      //3. Check if current robot position is near the goal position within the distance tolerance 
      //   - If so, Check if robot orientation is near the goal orientation:
      //            - If so, planning succeed ,
      //            - If not, use p controller to control robot turn to goal orientation
      //   - If not, use p controller to control robot move to goal position without turning
      if (hypot(goal_pose.pose.position.x,goal_pose.pose.position.y)<= goal_dist_tolerance_){

        if(std::abs(diff_yaw) < goal_angle_tolerance_){
          plan_ = false;
          geometry_msgs::Twist cmd_vel;
          cmd_vel.linear.x = 0;
          cmd_vel.linear.y = 0;
          cmd_vel.angular.z = 0;
          cmd_vel_pub_.publish(cmd_vel);
          ROS_INFO("Planning Success!");
          return;
        } else{
          if( diff_yaw > 0 ){
          cmd_vel.angular.z = std::min(p_yaw_coeff_*diff_yaw, max_yaw_speed_);
          }else{
          cmd_vel.angular.z = std::max(p_yaw_coeff_*diff_yaw, -max_yaw_speed_);
          }
          cmd_vel.linear.x = 0;
          cmd_vel.linear.y = 0;
        }

      } else{
        if( goal_pose.pose.position.x > 0 ){
          cmd_vel.linear.x = std::min(p_x_coeff_*goal_pose.pose.position.x, max_x_speed_);
        }else{
          cmd_vel.linear.x  = std::max(p_x_coeff_*goal_pose.pose.position.x, -max_x_speed_);
        }

        if( goal_pose.pose.position.y > 0 ){
          cmd_vel.linear.y = std::min(p_y_coeff_*goal_pose.pose.position.y, max_y_speed_);
        }else{
          cmd_vel.linear.y  = std::max(p_y_coeff_*goal_pose.pose.position.y, -max_y_speed_);
        }

         cmd_vel.angular.z = 0;
      }
      
      cmd_vel_pub_.publish(cmd_vel);
      

    }

  }

 private:

  ros::NodeHandle nh;
  std::shared_ptr<tf::TransformListener> tf_listener_;

  geometry_msgs::PoseStamped goal_;
  ros::Timer plan_timer_;

  ros::Subscriber goal_sub_;
  ros::Publisher cmd_vel_pub_;

  bool plan_;

  double max_x_speed_, max_y_speed_, max_yaw_speed_;
  double p_x_coeff_,p_y_coeff_,p_yaw_coeff_;
  double goal_dist_tolerance_,goal_angle_tolerance_;

  int plan_freq_;

};
}

using namespace robomaster;
int main(int argc, char **argv)
{
  ros::init(argc, argv, "pid_planner");
  ros::NodeHandle nh("~");
  PIDPlanner pid_planner(nh);
  ros::spin();
  return 0;
}

