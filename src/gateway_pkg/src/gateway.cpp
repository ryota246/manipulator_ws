#include "rclcpp/rclcpp.hpp"
#include <serial/serial.h>


double data[4] = {1, 2, 3, 4};

class GatewayNode : public rclcpp::Node
{
public:
    GatewayNode() : Node("gateway_node")
    {
        RCLCPP_INFO(this->get_logger(), "Gateway node has been started.");
    }

	timer = this->create_wall_timer(
		std::chrono::milliseconds(100),
		std::bind(&GatewayNode::timer_callback, this)

	);
	void timer_callback()
	{
		RCLCPP_INFO(this->get_logger(), "Timer callback triggered.");
	}
};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<GatewayNode>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
