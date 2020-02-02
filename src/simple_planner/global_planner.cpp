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

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/Marker.h>

#include <Eigen/Eigen>
#include <chrono>
#include <queue>
#include <cmath>
#include "utility.h"
#include <bitset>

namespace robomaster
{

class GlobalPlanner
{
public:
  GlobalPlanner(ros::NodeHandle &given_nh) : nh(given_nh), record_(false)
  {

    // nh.param<int>("point_num", point_num_, 10);
    nh.param<int>("plan_frequency", plan_freq_, 50);
    nh.param<std::string>("global_frame", global_frame_, "map");
    nh.param<double>("waypoint_distance", waypoint_dist_, 0.4);

    // -------------------visulize endpoints and trajectory---------------------
    tf_listener_ = std::make_shared<tf::TransformListener>();

    setpoint_pub_ = nh.advertise<visualization_msgs::Marker>("set_point", 10);
    global_path_pub_ = nh.advertise<nav_msgs::Path>("path", 10);

    init_map();

    plan_timer_ = nh.createTimer(ros::Duration(1.0 / plan_freq_), &GlobalPlanner::Plan, this);

    path_.header.frame_id = global_frame_;
  }
  ~GlobalPlanner() = default;

private:
  void Plan(const ros::TimerEvent &event)
  {
    Graph graph;
    nav_msgs::Path path_ = Graph.getPath();
    global_path_pub_.publish();
  }

  void init_map()
  {
    // FIXME the first a few markers does not show
    for (size_t i = 0; i < 6; i++)
    {
      for (size_t j = 0; j < 6; j++)
      {
        mark_point(i + 0.25, j + 0.25, 0.3, visualization_msgs::Marker::SPHERE);
        mark_point(i + 0.75, j + 0.75, 0.3, visualization_msgs::Marker::CUBE);
      }
    }
  }

  void mark_point(float x, float y, float z, int shape)
  {
    visualization_msgs::Marker p;
    p.header.frame_id = global_frame_;
    p.header.stamp = ros::Time::now();
    p.id = rand();

    p.type = shape;
    p.action = visualization_msgs::Marker::ADD;

    p.pose.position.x = x;
    p.pose.position.y = y;
    p.pose.position.z = z;
    p.pose.orientation.w = 1;
    p.pose.orientation.x = 0;
    p.pose.orientation.y = 0;
    p.pose.orientation.z = 0;

    p.scale.x = p.scale.y = p.scale.z = 0.2;

    p.color.a = p.color.r = 1.0;
    p.color.g = p.color.b = 0.0;

    p.lifetime = ros::Duration(2000.0);

    setpoint_pub_.publish(p);
    ros::Duration(0.2).sleep();
  }

private:
  ros::NodeHandle nh;
  std::shared_ptr<tf::TransformListener> tf_listener_;
  std::string global_frame_;
  //  ros::Timer plan_timer_;

  ros::Publisher global_path_pub_;
  ros::Publisher setpoint_pub_;

  ros::Subscriber waypoint_sub_;
  ros::Subscriber record_sub_;

  ros::Timer record_timer_;

  int point_num_;
  int plan_freq_;
};
} // namespace robomaster

using namespace robomaster;
int main(int argc, char **argv)
{
  ros::init(argc, argv, "global_planner");
  ros::NodeHandle nh("~");
  // ros::Duration(5).sleep();
  GlobalPlanner global_planner(nh);
  ros::spin();
  return 0;
}
