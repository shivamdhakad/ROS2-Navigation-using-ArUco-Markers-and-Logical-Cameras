#pragma once

#include "rclcpp/rclcpp.hpp"
#include "mage_msgs/msg/advanced_logical_camera_image.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/msg/point.hpp"
#include "ros2_aruco_interfaces/msg/aruco_markers.hpp"
#include <vector>
#include <iostream>

class CameraImageSubscriber : public rclcpp::Node {
public:
    /**
     * Constructor for CameraImageSubscriber.
     * Initializes ROS 2 node, TF2 buffer, and sets up subscriptions.
     */
 CameraImageSubscriber() 
        : Node("camera_image_subscriber"), 
          tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock()))
    {
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Initialize subscriptions for five different cameras
        camera_image_subscriptions[0] = createCameraSubscription("/mage/camera1/image", "camera1_frame");
        camera_image_subscriptions[1] = createCameraSubscription("/mage/camera2/image", "camera2_frame");
        camera_image_subscriptions[2] = createCameraSubscription("/mage/camera3/image", "camera3_frame");
        camera_image_subscriptions[3] = createCameraSubscription("/mage/camera4/image", "camera4_frame");
        camera_image_subscriptions[4] = createCameraSubscription("/mage/camera5/image", "camera5_frame");
        aruco_marker_subscription_ = this->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>(
            "/aruco_markers",
            rclcpp::SensorDataQoS(),
            [this](const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg) {
                this->arucoMarkerCallback(msg);
            });
       
    }  
private:
    /**
     * Callback for processing AdvancedLogicalCameraImage messages.
     * Transforms poses from the camera frame to the map frame and stores them.
     * @param msg Shared pointer to the received message.
     * @param camera_frame_id The frame ID of the camera.
     */
    void cameraImageCallback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg, const std::string& camera_frame_id);

    /**
     * Callback for processing ArucoMarkers messages.
     * Logs information about detected ArUco markers.
     * @param msg Shared pointer to the received ArucoMarkers message.
     */
    void arucoMarkerCallback(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg);

    /**
     * Prints the stored poses in the map frame to the console.
     */
    void printStoredPoses();

    /**
     * Creates a subscription to a camera topic.
     * @param topic The ROS topic to subscribe to.
     * @param frame_id The frame ID associated with the topic.
     * @return A shared pointer to the created subscription.
     */
    std::shared_ptr<rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>> createCameraSubscription(const std::string& topic, const std::string& frame_id);

    // Array of subscriptions for camera images.
    rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr camera_image_subscriptions[5];

    // Subscription for ArUco marker detection.
    rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr aruco_marker_subscription_;

    // TF2 buffer for handling coordinate transformations.
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

    // TF2 listener for coordinate transformations.
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Vector to store transformed pose positions.
    std::vector<geometry_msgs::msg::Point> stored_poses_;
};


