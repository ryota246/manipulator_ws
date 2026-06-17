#include "rclcpp/rclcpp.hpp"
#include "serial/serial.h"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float32.hpp"



class gateway_node : public rclcpp::Node{
	public:

		gateway_node(serial::Serial& my_serial) : Node("gateway_node") , my_serial(my_serial){
			RCLCPP_INFO(this->get_logger(),"created gateway_node");


			timer = this->create_wall_timer(std::chrono::milliseconds(10),std::bind(&gateway_node::timer_callback,this));
			
			auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
			sub_angle_data = this->create_subscription<std_msgs::msg::Float32MultiArray>("angle_data",qos,std::bind(&gateway_node::angles_callback,this,std::placeholders::_1));
			
			sub_rpm_data = this->create_subscription<std_msgs::msg::Float32>("rpm_data",qos,std::bind(&gateway_node::rpm_callback,this,std::placeholders::_1));
			


		}

	private:

		rclcpp::TimerBase::SharedPtr timer;
		serial::Serial& my_serial;
		rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_angle_data;
		rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_rpm_data;

		float angles[6];
		float rpm;

		void rpm_callback(const std_msgs::msg::Float32::SharedPtr msg){
			rpm = msg->data;
		}


		void angles_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg){
			for (int i=0;i<6;i++){
				angles[i] = msg->data[i];
			}
		}

		void timer_callback(){

			angles[0] = 0.0;
			angles[1] = 20;
			angles[2] = 40;
			angles[3] = 60;
			angles[4] = 80;
			angles[5] = 100;
			


			std::string data[7];

			for (int i = 0 ; i < 6 ; i++){
				data[i]  = "S" + std::to_string(i) + " " + std::to_string(angles[i]);
			}

			data[6] = "D" + std::to_string(rpm);



			//float data[8] = {0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8};
			for (int i = 0 ; i < 7 ; i++){
				my_serial.write(data[i]);
			}
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