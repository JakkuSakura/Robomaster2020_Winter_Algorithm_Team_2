
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
#ifdef GENERATE_DATA
    std::ofstream output(data_filename, std::ios::app);
    output << "blue: ";
    for (size_t i = 0; i < 36; i++)
    {
      output << mat2[i] << " ";
    }
    output << std::endl;

    output << "path: ";
    for (size_t i = 0; i < 36; i++)
    {
      output << best[i] << " ";
    }
    output << std::endl;
    output.close();

#endif
    nav_msgs::Path path;

    path.header.stamp = ros::Time::now();
    path.header.frame_id = global_frame_;

    for (size_t i = 0; i < 36; i++)
    {

      {
        float x, y;
        lookup(mat1, mat2, best[i], x, y);
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = global_frame_;
        pose.pose.position.x = y, pose.pose.position.y = x;
        pose.pose.orientation.w = 1;
        
        path.poses.push_back(pose);
      }

      {
        float x, y;
        lookup(mat1, mat2, pair(best[i]), x, y);

        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = global_frame_;
        pose.pose.position.x = y, pose.pose.position.y = x;
        pose.pose.orientation.w = 1;
        
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
