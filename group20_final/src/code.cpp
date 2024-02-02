// //// most probably i will use above code

#include "rclcpp/rclcpp.hpp"
#include "mage_msgs/msg/advanced_logical_camera_image.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/msg/point.hpp"
#include <vector>
#include <iostream>
#include "code.hpp"
#include "ros2_aruco_interfaces/msg/aruco_markers.hpp"
#include "mage_msgs/msg/part.hpp"



    std::shared_ptr<rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>> CameraImageSubscriber::createCameraSubscription(const std::string& topic, const std::string& frame_id)
    {
        return this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>(
            topic,
            rclcpp::SensorDataQoS(),
            [this, frame_id](const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg) { 
                this->cameraImageCallback(msg, frame_id); 
            });
    }

    void CameraImageSubscriber::cameraImageCallback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg, const std::string& camera_frame_id)
{
    for (const auto& part_pose : msg->part_poses)
    {
        // Extract the part type and color
        int part_type = part_pose.part.type;
        int part_color = part_pose.part.color;

        geometry_msgs::msg::PoseStamped pose_in_camera_frame;
        pose_in_camera_frame.pose = part_pose.pose;
        pose_in_camera_frame.header.frame_id = camera_frame_id;
        pose_in_camera_frame.header.stamp = this->get_clock()->now();
        
        if (stored_poses_.size() < 5) {

        try {
            // Transform the pose to the map frame
            geometry_msgs::msg::PoseStamped pose_in_map_frame = tf_buffer_->transform(pose_in_camera_frame, "map");

            // Store the transformed pose position in the vector
            stored_poses_.push_back(pose_in_map_frame.pose.position);

            type_color.push_back({part_type, part_color});

            // Log the part type, color, and transformed pose in the map frame
            RCLCPP_INFO(this->get_logger(), "Part Type: %d, Color: %d, Transformed Pose in Map Frame [%s] - Position: [x: %f, y: %f, z: %f]", 
                        part_type, part_color, camera_frame_id.c_str(),
                        pose_in_map_frame.pose.position.x, 
                        pose_in_map_frame.pose.position.y, 
                        pose_in_map_frame.pose.position.z);
            
            
            // Print the contents of the vector
            // printStoredPoses();

        } 
        catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform pose from %s to map frame: %s", camera_frame_id.c_str(), ex.what());
        }
        }
        else if (stored_poses_.size() ==5){
            for (auto &i:camera_image_subscriptions){
                i.reset();
            }
            CameraImageSubscriber::sort_parts(); 
        }
           

        // printStoredPoses();
    }
    // printStoredPoses();
}

void CameraImageSubscriber::arucoMarkerCallback(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg)
    {
        int marker_id;
        for (size_t i = 0; i < msg->marker_ids.size(); ++i)
        {
            marker_id = msg->marker_ids[i];
            // const auto& marker_pose = msg->poses[i];

            RCLCPP_INFO(this->get_logger(), "Marker ID: %d", 
                        marker_id);
        }
        if (marker_id == 0){
            params_list.push_back(this->get_parameter("aruco_0.wp1.color").as_string());
            params_list.push_back(this->get_parameter("aruco_0.wp2.color").as_string());
            params_list.push_back(this->get_parameter("aruco_0.wp3.color").as_string());
            params_list.push_back(this->get_parameter("aruco_0.wp4.color").as_string());
            params_list.push_back(this->get_parameter("aruco_0.wp5.color").as_string());
        }
        if (marker_id == 1){
            params_list.push_back(this->get_parameter("aruco_1.wp1.color").as_string());
            params_list.push_back(this->get_parameter("aruco_1.wp2.color").as_string());
            params_list.push_back(this->get_parameter("aruco_1.wp3.color").as_string());
            params_list.push_back(this->get_parameter("aruco_1.wp4.color").as_string());
            params_list.push_back(this->get_parameter("aruco_1.wp5.color").as_string());
        }


        aruco_marker_subscription_.reset();
    }


   
    void CameraImageSubscriber::printStoredPoses() {
        std::cout << "Stored Poses in Map Frame:" << std::endl;
        for (const auto& pose : stored_poses_) {
            std::cout << "  [x: " << pose.x << ", y: " << pose.y << ", z: " << pose.z << "]" << std::endl;
        }
    }



void CameraImageSubscriber::sort_parts(){
    std::vector<int> params_list_int;
    for (auto &i:params_list){
        if (i == "blue"){params_list_int.push_back(mage_msgs::msg::Part::BLUE);}
        if (i == "green"){params_list_int.push_back(mage_msgs::msg::Part::GREEN);}
        if (i == "red"){params_list_int.push_back(mage_msgs::msg::Part::RED);}
        if (i == "orange"){params_list_int.push_back(mage_msgs::msg::Part::ORANGE);}
        if (i == "purple"){params_list_int.push_back(mage_msgs::msg::Part::PURPLE);}
    }

    std::vector<int> index_list;
    for(auto &i:params_list_int){
        for (size_t j=0; j<type_color.size();j++){
            if(i == type_color[j][1]){index_list.push_back(j);}
        }
    }    

    for(auto &i:index_list){
        sorted_stored_poses_.push_back(stored_poses_[i]);
        RCLCPP_INFO(this->get_logger(), "Sorted Transformed Pose in Map Frame [%d] - Position: [x: %f, y: %f, z: %f]", 
                    type_color[i][1],
                    stored_poses_[i].x, 
                    stored_poses_[i].y, 
                    stored_poses_[i].z);        
    }   
    std::cout<<params_list_int.size()<<'\n';

    CameraImageSubscriber::send_goal();

}

void CameraImageSubscriber::set_initial_pose(){
  auto initial_pose = geometry_msgs::msg::PoseWithCovarianceStamped();
    initial_pose.header.frame_id = "map"; 
    // initial_pose.header.stamp = this->get_clock()->now();

    // Set the position (x, y, z)
    initial_pose.pose.pose.position.x = 1.0; // example values
    initial_pose.pose.pose.position.y = -2.0;
    initial_pose.pose.pose.position.z = 0.0;

    // Set the orientation (represented as a quaternion)
    initial_pose.pose.pose.orientation.x = 0.0;
    initial_pose.pose.pose.orientation.y = 0.0;
    initial_pose.pose.pose.orientation.z = -1.57;
    initial_pose.pose.pose.orientation.w = 1.0;

    // Set the covariance (here we are providing an example; adjust as needed)
    // The covariance matrix is 6x6. Here we set only diagonal elements for simplicity.
    initial_pose.pose.covariance[0] = 0.25; // variance of x
    initial_pose.pose.covariance[7] = 0.25; // variance of y
    initial_pose.pose.covariance[35] = 0.06; // variance of yaw

    // std::this_thread::sleep_for(std::chrono::seconds(5));
    publisher->publish(initial_pose);
    // CameraImageSubscriber::send_goal();

}


void CameraImageSubscriber::send_goal(){
        // Create a goal
        std::this_thread::sleep_for(std::chrono::seconds(5));
        auto goal = nav2_msgs::action::NavigateThroughPoses::Goal();
        goal.poses.reserve(sorted_stored_poses_.size());
        // std::vector<geometry_msgs::msg::PoseStamped> poses;
        for (const auto& point : sorted_stored_poses_) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "map";  
            // pose.header.stamp = this->get_clock()->now();
            pose.pose.position.x = point.x;
            pose.pose.position.y = point.y;
            
            waypoint_pub->publish(pose);

            goal.poses.push_back(pose);
            rclcpp::sleep_for(std::chrono::seconds(1));
        }

        // Create a NavigateThroughPoses goal
        // nav2_msgs::action::NavigateThroughPoses::Goal goal;
        // Send goal and register callbacks
        auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SendGoalOptions();
        send_goal_options.goal_response_callback = std::bind(&CameraImageSubscriber::goal_response_callback, this, std::placeholders::_1);
        // send_goal_options.feedback_callback = std::bind(&CameraImageSubscriber::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback = std::bind(&CameraImageSubscriber::result_callback, this, std::placeholders::_1);

        this->action_client_->async_send_goal(goal, send_goal_options);
}

void CameraImageSubscriber::goal_response_callback(std::shared_future<GoalHandleNavigateThroughPoses::SharedPtr> future) {
    auto goal_handle = future.get();
    if (!goal_handle) {
        RCLCPP_ERROR(rclcpp::get_logger("client"), "Goal was rejected by server");
    } else {
        RCLCPP_INFO(rclcpp::get_logger("client"), "Goal accepted by server, waiting for result");
    }
}

// void CameraImageSubscriber::feedback_callback(
//     GoalHandleNavigateThroughPoses::SharedPtr,
//     const std::shared_ptr<const NavigateThroughPoses::Feedback> feedback) {
//     RCLCPP_INFO(rclcpp::get_logger("client"), "Current waypoint: %d", feedback->current_waypoint);
//     // Additional feedback handling
// }

void CameraImageSubscriber::result_callback(const GoalHandleNavigateThroughPoses::WrappedResult & result) {
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(rclcpp::get_logger("client"), "Goal succeeded");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(rclcpp::get_logger("client"), "Goal was aborted");
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(rclcpp::get_logger("client"), "Goal was canceled");
            return;
        default:
            RCLCPP_ERROR(rclcpp::get_logger("client"), "Unknown result code");
            return;
    }
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraImageSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}



