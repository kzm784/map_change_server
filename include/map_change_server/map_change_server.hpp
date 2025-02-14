#ifndef MAP_CHANGE_SERVER__MAP_CHANGE_SERVER_HPP_
#define MAP_CHANGE_SERVER__MAP_CHANGE_SERVER_HPP_

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/msg/string.hpp>
#include <example_interfaces/msg/empty.hpp>
#include <waypoint_function_msgs/srv/command.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

class MapChangeServer : public rclcpp::Node
{
    public:
        explicit MapChangeServer(const rclcpp::NodeOptions & options);

    private:
        void Callback(const std::shared_ptr<waypoint_function_msgs::srv::Command::Request> request,
                        std::shared_ptr<waypoint_function_msgs::srv::Command::Response> response);
        void ServerApply();
    
        rclcpp::Client<waypoint_function_msgs::srv::Command>::SharedPtr apply_client_;
        rclcpp::Service<waypoint_function_msgs::srv::Command>::SharedPtr server_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;

        std::string COMMAND_HEADER = "map_change";
        std::string SERVER_NAME = "map_change_server";
};

#endif  // MAP_CHANGE_SERVER__MAP_CHANGE_SERVER_HPP_