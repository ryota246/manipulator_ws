#include "rclcpp/rclcpp.hpp"
#include "serial/serial.h"


class gateway_node : public rclcpp::Node{
	public:

		gateway_node(serial::Serial& my_serial) : Node("gateway_node") , my_serial(my_serial){
			RCLCPP_INFO(this->get_logger(),"created gateway_node");


			timer = this->create_wall_timer(std::chrono::milliseconds(10),std::bind(&gateway_node::timer_callback,this));

			


		}

	private:

		rclcpp::TimerBase::SharedPtr timer;
		serial::Serial& my_serial;

		void timer_callback(){

			float data[8] = {0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8};

			my_serial.write(reinterpret_cast<uint8_t*>(data),sizeof(data));

		}
};

int main(int argc, char **argv){

	if(argc < 2){
		std::cerr << "Usage: gateway_node <serial port address> <baudrate>" << std::endl;
		return 0;
	}

	std::string port(argv[1]);
	unsigned long boud = 0;
	sscanf(argv[2],"%lu",&boud);
	serial::Serial my_serial(port,boud,serial::Timeout::simpleTimeout(1000));


	rclcpp::init(argc,argv);
	auto node = std::make_shared<gateway_node>(my_serial);
	rclcpp::spin(node);
	rclcpp::shutdown();

	return 0;

}