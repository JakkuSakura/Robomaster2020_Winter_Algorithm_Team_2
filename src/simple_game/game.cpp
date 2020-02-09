#include <iostream>
#include <cstdlib>
#include <ctime>
#include <ros/ros.h>
#include <vector>
#include <random>
#include <std_msgs/Int16MultiArray.h>
#include <visualization_msgs/MarkerArray.h>
#include "tf/transform_listener.h"
#include <fstream>
#include "../config.h"
#include "../qt_tool/testdata.h"
bool GetGlobalRobotPose(const std::shared_ptr<tf::TransformListener>& tf_listener,
                        const std::string& target_frame,
                        geometry_msgs::PoseStamped& robot_global_pose){
    tf::Stamped<tf::Pose> robot_pose_tf;
    robot_pose_tf.setIdentity();
    robot_pose_tf.frame_id_ = "base_link";
    robot_pose_tf.stamp_ = ros::Time();

    tf::Stamped<tf::Pose> robot_global_pose_tf;
    try{
        tf_listener->transformPose( target_frame, robot_pose_tf, robot_global_pose_tf);
    }
    catch (tf::TransformException &ex) {
        ROS_ERROR("Failed to transform robot pose: %s", ex.what());
        return false;
    }
    tf::poseStampedTFToMsg(robot_global_pose_tf, robot_global_pose);
    return true;
}
double GetEuclideanDistance(const geometry_msgs::Pose & pose_1,
                            const geometry_msgs::Pose & pose_2){
    return hypot(pose_1.position.x-pose_2.position.x,
                 pose_1.position.y-pose_2.position.y);
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "random_point");
    ros::NodeHandle nh;
    ros::Publisher random_pub = nh.advertise<std_msgs::Int16MultiArray>("blue_numbers",5);
    ros::Publisher number_pub = nh.advertise<visualization_msgs::MarkerArray>("number_markers", 10);
    //random points
    std_msgs::Int16MultiArray points;
    points.data.reserve(36);

    for(int i = 0; i < 36; i++){
        points.data.push_back(get_mat2()[i]);
    }
    
    //generate markers
    visualization_msgs::MarkerArray number_array;
    number_array.markers.resize(72);
    for (int i = 0; i < 36; i++) {
        number_array.markers[i].header.frame_id ="map" ;
        number_array.markers[i].header.stamp = ros::Time::now();
        number_array.markers[i].ns = "red";
        number_array.markers[i].id = i;
        number_array.markers[i].action = visualization_msgs::Marker::ADD;
        number_array.markers[i].type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        number_array.markers[i].pose.position.x = 1+(i/6)*2;
        number_array.markers[i].pose.position.y = 1+(i%6)*2;
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
    for (int i = 36; i < 72; i++) {
        marker_index = 36 + points.data[i-36];
        number_array.markers[marker_index].header.frame_id ="map" ;
        number_array.markers[marker_index].header.stamp = ros::Time::now();
        number_array.markers[marker_index].ns = "blue";
        number_array.markers[marker_index].id = marker_index;
        number_array.markers[marker_index].action = visualization_msgs::Marker::ADD;
        number_array.markers[marker_index].type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        number_array.markers[marker_index].pose.position.x = 2+(i-36)/6*2;
        number_array.markers[marker_index].pose.position.y = 2+(i-36)%6*2;
        number_array.markers[marker_index].pose.position.z = 0.1;
        number_array.markers[marker_index].pose.orientation.x = 0.0;
        number_array.markers[marker_index].pose.orientation.y = 0.0;
        number_array.markers[marker_index].pose.orientation.z = 0.0;
        number_array.markers[marker_index].pose.orientation.w = 1.0;

        number_array.markers[marker_index].text = std::to_string(points.data[i-36]);
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
    ros::Duration(0.1).sleep();
    ros::Rate rate(30);
    while (ros::ok()){
        GetGlobalRobotPose(tf_listener,"map",robot_pose);

        if (!target){

            neighbor_index = round((robot_pose.pose.position.x-1)/2)*6 +
                             round((robot_pose.pose.position.y-1)/2);

            if(std::find(pair.begin(), pair.end(), neighbor_index) == pair.end() &&
               GetEuclideanDistance(robot_pose.pose,
                                    number_array.markers[neighbor_index].pose) < 0.15){
                number_array.markers[neighbor_index].color.r = 0;
                number_array.markers[neighbor_index].color.g = 1;
                number_array.markers[neighbor_index].color.b = 0;
                target = true;
                if (pair.size() == 0){ start = ros::Time::now(); }
                target_pose = number_array.markers[neighbor_index + 36].pose;
            }

        } else{
            if(GetEuclideanDistance(robot_pose.pose, target_pose) < 0.15){
                number_array.markers[neighbor_index].color.a = 0;
                number_array.markers[neighbor_index + 36].color.a = 0;
                pair.push_back(neighbor_index);
                target = false;
            }
        }
        number_pub.publish(number_array);
        random_pub.publish(points);
#ifdef GENERATE_DATA
        if(ros::Time::now() - start > ros::Duration(TIME_LIMIT))
        {
            std::ofstream output(data_filename, std::ios::app);
            output << "OUT" << std::endl;
            output.close();
            system("killall rosmaster");
            ros::Duration(3).sleep();
        }
#endif        
        if (pair.size() == 36){
            auto dur = ros::Time::now() - start;
            std::cout<<"Finish: "<< dur <<" secs"<<std::endl;
#ifdef GENERATE_DATA
            std::ofstream output(data_filename, std::ios::app);
            output << dur << std::endl;
            output.close();

            system("killall rosmaster");
            ros::Duration(3).sleep();
#endif
            ros::shutdown();
        }
        rate.sleep();
    }
    return 0;
}

