#include "map_change_server/map_change_server.hpp"

using namespace std::chrono_literals;

MapChangeServer::MapChangeServer(const rclcpp::NodeOptions &options) : Node("map_change_server", options)
{
    // Publisher to change PCD
    pointcloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "map",
        rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
    
    // Create Server
    server_ = create_service<waypoint_function_msgs::srv::Command>(
        SERVER_NAME,
        std::bind(&MapChangeServer::Callback, this, std::placeholders::_1, std::placeholders::_2)
    );

    // Create Client for Server Apply
    apply_client_ = create_client<waypoint_function_msgs::srv::Command>("server_apply");
    // Apply Tihs Server to Host Server to create connection
    ServerApply();
}

void MapChangeServer::Callback(const std::shared_ptr<waypoint_function_msgs::srv::Command::Request> request, std::shared_ptr<waypoint_function_msgs::srv::Command::Response> response)
{
    RCLCPP_INFO(get_logger(), "Map Change Server Called.");

    // Load PCD file
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(request->data[1], *cloud) == -1) {
        RCLCPP_ERROR(this->get_logger(), "Failed to read PCD file.");
        return;
    }

    // Publish pointcloud
    sensor_msgs::msg::PointCloud2 pointcloud_msg;
    pcl::toROSMsg(*cloud, pointcloud_msg);
    pointcloud_msg.header.frame_id = "map";
    pointcloud_pub_->publish(pointcloud_msg);

    // Send Result Message
    std::string result_msg;
    result_msg = "complete";
    response->message = SERVER_NAME + ":" + result_msg;
}

void MapChangeServer::ServerApply()
{
    // Execute Server Apply
    while (!apply_client_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
          return;
        }
        RCLCPP_INFO(this->get_logger(), "Service is not available. waiting...");
    }
    
    // Send Request to Host Server
    auto request = std::make_shared<waypoint_function_msgs::srv::Command::Request>();
    request->data.push_back(COMMAND_HEADER);
    request->data.push_back(SERVER_NAME);
    auto result = apply_client_->async_send_request(request);

    auto returnCode = rclcpp::spin_until_future_complete(
        this->get_node_base_interface(), result);
    
    // Wait for Recieving Result
    if(returnCode == rclcpp::FutureReturnCode::SUCCESS){
        std::string msg = result.get()->message;
        if(msg[0] == 'S') RCLCPP_INFO(get_logger(), msg.c_str());
        else if (msg[0] == 'F') RCLCPP_ERROR(get_logger(), msg.c_str());
    }
    else RCLCPP_ERROR(get_logger(), "Server Apply Failed.");

}


#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(MapChangeServer)