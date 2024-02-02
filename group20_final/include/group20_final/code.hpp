/*! \file navigation_demo.hpp
    \brief Documentation for the CameraImageSubscriber class.

    Describes the purpose and usage of the CameraImageSubscriber class.
*/


#include "rclcpp/rclcpp.hpp"
#include "mage_msgs/msg/advanced_logical_camera_image.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/msg/point.hpp"
#include <vector>
#include <iostream>
#include "ros2_aruco_interfaces/msg/aruco_markers.hpp" 
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp_action/rclcpp_action.hpp"


/*!
 * \class CameraImageSubscriber
 * \brief Subscriber for camera images and ArUco markers, and a client for navigating through poses.
 * 
 * CameraImageSubscriber is a ROS2 node that subscribes to multiple camera image topics and ArUco marker data.
 * It processes this information and interacts with a navigation action server to navigate through specified poses.
 */

class CameraImageSubscriber : public rclcpp::Node
{
public:
        /*!
     * \brief Constructor for CameraImageSubscriber.
     * 
     * Initializes the node, sets up camera image and ArUco marker subscriptions, publishers, and an action client.
     */
  using NavigateToPose = nav2_msgs::action::NavigateThroughPoses;
using GoalHandleNavigateThroughPoses = rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>;
    CameraImageSubscriber() 

        /*!
     * \brief Sets the initial pose of the robot.
     * 
     * This method publishes the initial pose to the appropriate topic.
     */
        : Node("camera_image_subscriber"), 
          tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock()))
    {
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        

        this->declare_parameter("aruco_0.wp1.type","battery");
        this->declare_parameter("aruco_0.wp1.color","green");
        this->declare_parameter("aruco_0.wp2.type","battery");
        this->declare_parameter("aruco_0.wp2.color","red");
        this->declare_parameter("aruco_0.wp3.type","battery");
        this->declare_parameter("aruco_0.wp3.color","orange");
        this->declare_parameter("aruco_0.wp4.type","battery");
        this->declare_parameter("aruco_0.wp4.color","purple");
        this->declare_parameter("aruco_0.wp5.type","battery");
        this->declare_parameter("aruco_0.wp5.color","blue");

        this->declare_parameter("aruco_1.wp1.type","battery");
        this->declare_parameter("aruco_1.wp1.color","blue");
        this->declare_parameter("aruco_1.wp2.type","battery");
        this->declare_parameter("aruco_1.wp2.color","green");
        this->declare_parameter("aruco_1.wp3.type","battery");
        this->declare_parameter("aruco_1.wp3.color","orange");
        this->declare_parameter("aruco_1.wp4.type","battery");
        this->declare_parameter("aruco_1.wp4.color","red");
        this->declare_parameter("aruco_1.wp5.type","battery");
        this->declare_parameter("aruco_1.wp5.color","purple");


        std::this_thread::sleep_for(std::chrono::seconds(5));
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


        publisher = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 10);

        waypoint_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("waypoints",10);

        this->action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateThroughPoses>(this, "navigate_through_poses");

        // Wait for the action server to be available
        if (!this->action_client_->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            rclcpp::shutdown();
        }
       
        set_initial_pose();

    }



private:


    rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr aruco_marker_subscription_;
    rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr camera_image_subscriptions[5];

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_pub;
    
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::vector<geometry_msgs::msg::Point> stored_poses_; // Vector to store transformed pose positions
    std::vector<geometry_msgs::msg::Point> sorted_stored_poses_; // Vector to store transformed pose positions


    std::vector<std::vector<int>> type_color; 

    std::shared_ptr<rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>> createCameraSubscription(const std::string& topic, const std::string& frame_id);

    std::vector<std::string> params_list;


    rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SharedPtr action_client_;

    /*!
     * \brief Callback for camera image data.
     * 
     * \param msg Shared pointer to the received camera image message.
     * \param camera_frame_id The frame ID associated with the camera image.
     */
    
    void cameraImageCallback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg, const std::string& camera_frame_id);

    /*!
     * \brief Callback for ArUco marker data.
     * 
     * \param msg Shared pointer to the received ArUco markers message.
     */
    void arucoMarkerCallback(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg);

    void sort_parts();

    void set_initial_pose();

    void send_goal();
   
    void printStoredPoses();

    void goal_response_callback(
        std::shared_future<GoalHandleNavigateThroughPoses::SharedPtr> future);
    // /**
    //  * @brief Feedback received while the robot is driving towards the goal
    //  *
    //  * @param feedback
    //  */
    // void feedback_callback(
    //     GoalHandleNavigateThroughPoses::SharedPtr,
    //     const std::shared_ptr<const nav2_msgs::action::NavigateThroughPoses::Feedback> feedback);
    // /**
    //  * @brief Result after the action has completed
    //  *
    //  * @param result
    //  */
    void result_callback(const GoalHandleNavigateThroughPoses::WrappedResult& result);

};




