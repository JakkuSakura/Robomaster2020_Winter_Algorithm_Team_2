#include <iostream>
#include <cstdlib>
#include <ctime>
#include <ros/ros.h>
#include <vector>
#include <random>
#include <std_msgs/Int16MultiArray.h>
#include <visualization_msgs/MarkerArray.h>
#include "tf/transform_listener.h"
bool GetGlobalRobotPose(const std::shared_ptr<tf::TransformListener> &tf_listener,
                        const std::string &target_frame,
                        geometry_msgs::PoseStamped &robot_global_pose)
{
    tf::Stamped<tf::Pose> robot_pose_tf;
    robot_pose_tf.setIdentity();
    robot_pose_tf.frame_id_ = "base_link";
    robot_pose_tf.stamp_ = ros::Time();

    tf::Stamped<tf::Pose> robot_global_pose_tf;
    try
    {
        tf_listener->transformPose(target_frame, robot_pose_tf, robot_global_pose_tf);
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("Failed to transform robot pose: %s", ex.what());
        return false;
    }
    tf::poseStampedTFToMsg(robot_global_pose_tf, robot_global_pose);
    return true;
}
double GetEuclideanDistance(const geometry_msgs::Pose &pose_1,
                            const geometry_msgs::Pose &pose_2)
{
    return hypot(pose_1.position.x - pose_2.position.x,
                 pose_1.position.y - pose_2.position.y);
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "random_point");
    ros::NodeHandle nh;
    ros::Publisher random_pub = nh.advertise<std_msgs::Int16MultiArray>("blue_numbers", 5);
    ros::Publisher number_pub = nh.advertise<visualization_msgs::MarkerArray>("number_markers", 10);
    
    int gen_points_model;
    nh.param<int>("gen_points_model", gen_points_model, 1);
    std_msgs::Int16MultiArray points;
    ROS_WARN("param is %d.", gen_points_model);
    //随机数据集
    if (gen_points_model == 0)
    {
        points.data.reserve(36);
        for (int i = 0; i < 36; i++)
        {
            points.data.push_back(i);
        }
        std::random_device rd;
        std::mt19937 gen(rd());
        std::shuffle(points.data.begin(), points.data.end(), gen);
    }
    //特定测试数据集
    else if (gen_points_model == 1)
    {
        points.data = {33,23,9,27,31,1,12,34,32,0,35,30,22,18,29,13,17,16,10,8,26,28,6,19,2,4,7,21,3,14,20,11,15,5,24,25};
    }
    else if (gen_points_model == 2)
    {
        points.data = {32,4,17,21,20,12,27,28,14,9,25,8,5,10,31,29,33,22,34,15,35,0,7,23,6,2,16,18,11,3,1,19,30,24,13,26,};
    }
    else if (gen_points_model == 3)
    {
        points.data = {9,19,16,18,4,15,8,13,7,22,1,17,10,11,24,29,23,12,5,28,0,31,2,34,14,35,3,25,33,27,21,32,26,30,20,6};
    }
    else if (gen_points_model == 4)
    {
        points.data = {31,34,2,28,9,20,6,29,1,16,4,15,13,5,10,22,25,7,27,18,0,21,14,33,30,19,32,17,24,26,12,8,3,35,23,11};
    }
    else if (gen_points_model == 5)
    {
        points.data = {7,30,13,5,21,19,22,20,1,32,8,35,34,6,25,24,18,27,10,16,29,17,28,31,14,15,33,11,26,9,2,4,0,3,23,12};
    }
    else
    {
        ROS_ERROR("param error!");
        return 0;
    }

    for (int i = 0; i < 36; i++)
    {
        std::cout << points.data[i] << std::endl;
    }
    ROS_ERROR("DONE");
    //generate markers
    visualization_msgs::MarkerArray number_array;
    number_array.markers.resize(72);
    for (int i = 0; i < 36; i++)
    {
        number_array.markers[i].header.frame_id = "map";
        number_array.markers[i].header.stamp = ros::Time::now();
        number_array.markers[i].ns = "red";
        number_array.markers[i].id = i;
        number_array.markers[i].action = visualization_msgs::Marker::ADD;
        number_array.markers[i].type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        number_array.markers[i].pose.position.x = 1 + (i / 6) * 2;
        number_array.markers[i].pose.position.y = 1 + (i % 6) * 2;
        number_array.markers[i].pose.position.z = 0.1;
        number_array.markers[i].pose.orientation.x = 0.0;
        number_array.markers[i].pose.orientation.y = 0.0;
        number_array.markers[i].pose.orientation.z = 0.0;
        number_array.markers[i].pose.orientation.w = 1.0;

        number_array.markers[i].text = std::to_string(i);
        number_array.markers[i].scale.z = 0.4;

        number_array.markers[i].color.a = 1.0;
        number_array.markers[i].color.r = 1.0;
        number_array.markers[i].color.g = 0.0;
        number_array.markers[i].color.b = 0.0;
    }

    int marker_index;
    for (int i = 36; i < 72; i++)
    {
        marker_index = 36 + points.data[i - 36];
        number_array.markers[marker_index].header.frame_id = "map";
        number_array.markers[marker_index].header.stamp = ros::Time::now();
        number_array.markers[marker_index].ns = "blue";
        number_array.markers[marker_index].id = marker_index;
        number_array.markers[marker_index].action = visualization_msgs::Marker::ADD;
        number_array.markers[marker_index].type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        number_array.markers[marker_index].pose.position.x = 2 + (i - 36) / 6 * 2;
        number_array.markers[marker_index].pose.position.y = 2 + (i - 36) % 6 * 2;
        number_array.markers[marker_index].pose.position.z = 0.1;
        number_array.markers[marker_index].pose.orientation.x = 0.0;
        number_array.markers[marker_index].pose.orientation.y = 0.0;
        number_array.markers[marker_index].pose.orientation.z = 0.0;
        number_array.markers[marker_index].pose.orientation.w = 1.0;

        number_array.markers[marker_index].text = std::to_string(points.data[i - 36]);
        number_array.markers[marker_index].scale.z = 0.4;

        number_array.markers[marker_index].color.a = 1.0;
        number_array.markers[marker_index].color.r = 0.0;
        number_array.markers[marker_index].color.g = 0.0;
        number_array.markers[marker_index].color.b = 1.0;
    }

    //listen
    std::shared_ptr<tf::TransformListener> tf_listener = std::make_shared<tf::TransformListener>();
    geometry_msgs::PoseStamped robot_pose;
    geometry_msgs::Pose target_pose;
    bool target = false;
    int neighbor_index;
    ros::Time start;
    std::vector<int> pair;
    ros::Rate rate(30);
    while (ros::ok())
    {
        GetGlobalRobotPose(tf_listener, "map", robot_pose);

        bool find = false;
        double dis = GetEuclideanDistance(robot_pose.pose, target_pose);
        if (std::find(pair.begin(), pair.end(), neighbor_index) == pair.end())
            find = true;
        //ROS_INFO("distance=%f,neighbor=%d,find?=%d",dis,neighbor_index,find);

        if (!target)
        {

            neighbor_index = round((robot_pose.pose.position.x - 1) / 2) * 6 +
                             round((robot_pose.pose.position.y - 1) / 2);

            if (std::find(pair.begin(), pair.end(), neighbor_index) == pair.end() &&
                GetEuclideanDistance(robot_pose.pose,
                                     number_array.markers[neighbor_index].pose) < 0.15)
            {
                number_array.markers[neighbor_index].color.r = 0;
                number_array.markers[neighbor_index].color.g = 1;
                number_array.markers[neighbor_index].color.b = 0;
                target = true;
                if (pair.size() == 0)
                {
                    start = ros::Time::now();
                }
                target_pose = number_array.markers[neighbor_index + 36].pose;
            }
        }
        else
        {
            if (GetEuclideanDistance(robot_pose.pose, target_pose) < 0.15)
            {
                number_array.markers[neighbor_index].color.a = 0;
                number_array.markers[neighbor_index + 36].color.a = 0;
                pair.push_back(neighbor_index);
                target = false;
            }
        }
        number_pub.publish(number_array);
        random_pub.publish(points);
        if (pair.size() == 36)
        {
            std::cout << "Finish: " << ros::Time::now() - start << " secs" << std::endl;
            ros::shutdown();
        }
        rate.sleep();
    }
    return 0;
}
