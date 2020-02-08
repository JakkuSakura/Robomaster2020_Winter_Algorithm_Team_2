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
#include <std_msgs/Int16MultiArray.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/Marker.h>

#include <Eigen/Eigen>
#include <chrono>
#include <queue>
#include <cmath>
#include "utility.h"
#include <bitset>
#include <algorithm>
#include <fstream>
#include "graph.h"
#include "../config.h"

namespace robomaster
{

class GlobalPlanner
{
public:
  GlobalPlanner(ros::NodeHandle &given_nh) : nh(given_nh)
  {

    nh.param<std::string>("global_frame", global_frame_, "map");

    // -------------------visulize endpoints and trajectory---------------------
    tf_listener_ = std::make_shared<tf::TransformListener>();

    // setpoint_pub_ = nh.advertise<visualization_msgs::Marker>("set_point", 100);
    global_path_pub_ = nh.advertise<nav_msgs::Path>("path", 100);
    point_mat_fetcher_ = nh.subscribe("/blue_numbers", 1, &GlobalPlanner::fetch_numbers, this);
  }
  ~GlobalPlanner() = default;

private:
  void fetch_numbers(const std_msgs::Int16MultiArray &points)
  {
    if (planned)
      return;

    int mat1[36], mat2[36];
    for (size_t i = 0; i < 36; i++)
    {
      mat1[i] = i;
      mat2[i] = points.data[i];
    }
    Plan(mat1, mat2);
    planned = true;
    ROS_INFO("Planned global path");
  }
  void Plan(const int *mat1, const int *mat2)
  {
    std::vector<int> best = calculate_path(mat1, mat2);
    nav_msgs::Path path;

    path.header.stamp = ros::Time::now();
    path.header.frame_id = global_frame_;

#ifdef GENERATE_DATA
    std::ofstream output("run.dat", std::ios::app);
    for (size_t i = 0; i < 36; i++)
    {
      output << best[i] << " ";
    }
    output << std::endl;
#endif

    for (size_t i = 0; i < 36; i++)
    {

      {
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = global_frame_;
        float x, y;
        lookup(mat1, mat2, best[i], x, y);
        pose.pose.position.x = y, pose.pose.position.y = x;
        path.poses.push_back(pose);
      }

      {
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = global_frame_;

        float x, y;
        lookup(mat1, mat2, pair(best[i]), x, y);
        pose.pose.position.x = y, pose.pose.position.y = x;
        path.poses.push_back(pose);
      }
    }

    global_path_pub_.publish(path);
  }

private:
  ros::NodeHandle nh;
  std::shared_ptr<tf::TransformListener> tf_listener_;
  std::string global_frame_;
  ros::Publisher global_path_pub_;
  // ros::Publisher setpoint_pub_;

  ros::Subscriber point_mat_fetcher_;
  bool planned = false;
};
} // namespace robomaster

using namespace robomaster;
int main(int argc, char **argv)
{
  ros::init(argc, argv, "global_planner");
  ros::NodeHandle nh("~");
  GlobalPlanner global_planner(nh);
  ros::spin();
  return 0;
}
