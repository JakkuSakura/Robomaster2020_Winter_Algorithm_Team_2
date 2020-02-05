#if !defined(WITHOUT_SPLINE_H)
#define WITHOUT_SPLINE_H

#include <iostream>
#include <vector>
#include <array>
#include <string>
#include <Eigen/Eigen>

// FIXME Why there's no trace?

void GenTraj(const nav_msgs::Path &path, nav_msgs::Path &smoothed_path, const float interval = 0.1)
{
    smoothed_path = path;
    for (size_t i = 0; i < path.poses.size() - 1; i++)
    {
        float yaw = atan2f(path.poses[i + 1].pose.position.y - path.poses[i].pose.position.y, path.poses[i + 1].pose.position.x - path.poses[i].pose.position.x);
        
        smoothed_path.poses[i].pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    }
}

#endif // WITHOUT_SPLINE_H
