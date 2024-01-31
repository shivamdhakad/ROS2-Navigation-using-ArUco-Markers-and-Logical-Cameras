/*! \file navigation_demo.hpp
    \brief Documentation for the CameraImageSubscriber class.

    Describes the purpose and usage of the CameraImageSubscriber class.
*/

#ifndef NAVIGATION_DEMO_HPP
#define NAVIGATION_DEMO_HPP

// ... [Include statements] ...

namespace navigation {

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
    CameraImageSubscriber();

    /*!
     * \brief Sets the initial pose of the robot.
     * 
     * This method publishes the initial pose to the appropriate topic.
     */
    void set_initial_pose();

    // ... [Other public methods] ...

private:
    // ... [Private member variables] ...

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

    // ... [Other private methods and member variables] ...

    /*!
     * \brief Client for the NavigateThroughPoses action server.
     */
    rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SharedPtr action_client_;

    // ... [Other member variables] ...
};

} // namespace navigation

#endif // NAVIGATION_DEMO_HPP
