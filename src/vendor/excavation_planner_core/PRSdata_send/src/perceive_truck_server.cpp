#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>
#include <truck_perceive/srv/perceive_truck.hpp>

using PerceiveTruck = truck_perceive::srv::PerceiveTruck;

class PerceiveTruckServer : public rclcpp::Node
{
public:
    PerceiveTruckServer() : Node("perceive_truck_server")
    {
        declare_parameter<double>("truck_pose.x", 2.862);
        declare_parameter<double>("truck_pose.y", -0.065);
        declare_parameter<double>("truck_pose.z", 0.577);
        declare_parameter<double>("truck_pose.qx", 0.004003);
        declare_parameter<double>("truck_pose.qy", -0.022834);
        declare_parameter<double>("truck_pose.qz", 0.986552);
        declare_parameter<double>("truck_pose.qw", -0.161797);

        declare_parameter<double>("material_volume", 12.5);
        declare_parameter<double>("material_highest_point.x", 0.5);
        declare_parameter<double>("material_highest_point.y", 0.0);
        declare_parameter<double>("material_highest_point.z", 1.2);
        declare_parameter<double>("repose_angle", 0.6);

        declare_parameter<bool>("issuccess", true);
        declare_parameter<std::string>("message", "mock perceive truck ok");

        server_ = create_service<PerceiveTruck>(
            "/perceive_truck",
            std::bind(
                &PerceiveTruckServer::handle_service, this,
                std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(get_logger(), "PerceiveTruck 模拟服务已启动: /perceive_truck");
    }

private:
    rclcpp::Service<PerceiveTruck>::SharedPtr server_;

    void handle_service(
        const std::shared_ptr<PerceiveTruck::Request> request,
        std::shared_ptr<PerceiveTruck::Response> response)
    {
        (void)request;

        response->truck_pose.position.x = get_parameter("truck_pose.x").as_double();
        response->truck_pose.position.y = get_parameter("truck_pose.y").as_double();
        response->truck_pose.position.z = get_parameter("truck_pose.z").as_double();
        response->truck_pose.orientation.x = get_parameter("truck_pose.qx").as_double();
        response->truck_pose.orientation.y = get_parameter("truck_pose.qy").as_double();
        response->truck_pose.orientation.z = get_parameter("truck_pose.qz").as_double();
        response->truck_pose.orientation.w = get_parameter("truck_pose.qw").as_double();

        response->material_volume = get_parameter("material_volume").as_double();
        response->material_highest_point.x = get_parameter("material_highest_point.x").as_double();
        response->material_highest_point.y = get_parameter("material_highest_point.y").as_double();
        response->material_highest_point.z = get_parameter("material_highest_point.z").as_double();
        response->repose_angle = get_parameter("repose_angle").as_double();

        response->issuccess = get_parameter("issuccess").as_bool();
        response->message = get_parameter("message").as_string();

        RCLCPP_INFO(
            get_logger(),
            "发送矿卡信息: pos(%.3f, %.3f, %.3f), quat(%.3f, %.3f, %.3f, %.3f), volume=%.3f, angle=%.3f, ok=%s",
            response->truck_pose.position.x,
            response->truck_pose.position.y,
            response->truck_pose.position.z,
            response->truck_pose.orientation.x,
            response->truck_pose.orientation.y,
            response->truck_pose.orientation.z,
            response->truck_pose.orientation.w,
            response->material_volume,
            response->repose_angle,
            response->issuccess ? "true" : "false");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PerceiveTruckServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
