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

namespace robomaster
{
float p2(float x)
{
  return x * x;
}
struct State
{
  std::bitset<64> visited;
  float x = 0, y = 0;
  float orientation = 0;
  float g = 0;
  std::vector<int> path;
  friend bool operator<(const State &lhs, const State &rhs)
  {
    // TODO here
    return -lhs.g + p2(lhs.path.size()) * 1000 < -rhs.g + p2(rhs.path.size()) * 1000;
  }
};
class Graph
{
  const int *id1_, *id2_;

public:
  Graph(const int *id1, const int *id2)
  {
    id1_ = id1;
    id2_ = id2;
  }

  float dist(float x1, float y1, float x2, float y2)
  {
    return sqrt(p2(x1 - x2) + p2(y1 - y2));
  }

  float distance_in_degree(float alpha, float beta)
  {
    float phi = abs(beta - alpha);
    while (phi > 360)
      phi -= 360;
    float distance = phi > 180 ? 360 - phi : phi;
    return distance;
  }

  State calc(State start)
  {
    std::priority_queue<State> que;
    que.push(start);
    State best;
    best.g = 1e6;
    int cnt = 0;
    while (que.size())
    {
      const State s = que.top();
      que.pop();

      if (s.path.size() >= 36)
      {
        if (s.g < best.g)
        {
          best = s;
        }
        if (++cnt > 200)
          return best;
      }

      for (size_t i = 0; i < 72; i++)
      {
        if (!s.visited[i % 36])
        {
          State s2;
          s2.visited = s.visited;
          s2.visited[i % 36] = 1;
          float x1, y1;
          lookup(i, x1, y1);           // original location
          lookup(pair(i), s2.x, s2.y); // teleport
          // TODO test this part
          float degree = atan2f(y1 - s.y, x1 - s.x) * 180 / M_PI;
          s2.orientation = atan2f(y1 - s2.y, x1 - s2.y) * 180 / M_PI;
          s2.g = s.g + 10 * p2(dist(x1, y1, s.x, s.y)) + distance_in_degree(s.orientation, degree) / 360.0 * 10 + distance_in_degree(degree, s2.orientation) / 360.0 * 10;
          s2.path = s.path;
          s2.path.push_back(i);
          que.push(s2);
        }
      }
    }
    return best;
  }
  int pair(int x)
  {
    if (x < 36)
    {
      return x + 36;
    }
    else
    {
      return x - 36;
    }
  }
  // get the location from id and type
  void lookup(int id, float &x, float &y)
  {
    int type = id < 36 ? 1 : 2;
    int ans = 0;
    if (type == 1)
    {
      for (size_t i = 0; i < 36; i++)
      {
        if (id1_[i] == id)
        {
          ans = i;
          break;
        }
      }
    }
    else
    {
      for (size_t i = 0; i < 36; i++)
      {
        if (id2_[i] == id - 36)
        {
          ans = i;
          break;
        }
      }
    }

    int row = ans / 6;
    int col = ans % 6;

    y = row * 2 + type;
    x = col * 2 + type;
  }
};
class GlobalPlanner
{
public:
  GlobalPlanner(ros::NodeHandle &given_nh) : nh(given_nh)
  {

    nh.param<std::string>("global_frame", global_frame_, "map");

    // -------------------visulize endpoints and trajectory---------------------
    tf_listener_ = std::make_shared<tf::TransformListener>();

    setpoint_pub_ = nh.advertise<visualization_msgs::Marker>("set_point", 100);
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
    Graph graph(mat1, mat2);
    State s;
    s.path.push_back(0);
    s.visited[0] = 1;
    State best = graph.calc(s);
    nav_msgs::Path path;

    path.header.stamp = ros::Time::now();
    path.header.frame_id = global_frame_;
    std::cout << "Calculated path: ";
    for (size_t i = 0; i < 36; i++)
    {
      std::cout << best.path[i] << " ";
    }
    std::cout << std::endl;

    for (size_t i = 0; i < 36; i++)
    {

      {
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = global_frame_;
        float x, y;
        graph.lookup(best.path[i], x, y);
        pose.pose.position.x = y, pose.pose.position.y = x;
        path.poses.push_back(pose);
      }

      {
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = global_frame_;

        float x, y;
        graph.lookup(graph.pair(best.path[i]), x, y);
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
  ros::Publisher setpoint_pub_;

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
