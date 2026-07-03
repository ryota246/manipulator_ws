#include "rclcpp/rclcpp.hpp"
#include "serial/serial.h"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float32.hpp"



class gateway_node : public rclcpp::Node{
	public:

		gateway_node(serial::Serial& my_serial) : Node("gateway_node") , my_serial(my_serial){
			RCLCPP_INFO(this->get_logger(),"created gateway_node");


			timer = this->create_wall_timer(std::chrono::milliseconds(30),std::bind(&gateway_node::timer_callback,this));
			
			auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
			sub_angle_data = this->create_subscription<std_msgs::msg::Float32MultiArray>("angle_data",qos,std::bind(&gateway_node::angles_callback,this,std::placeholders::_1));
			
			sub_rpm_data = this->create_subscription<std_msgs::msg::Float32>("rpm_data",qos,std::bind(&gateway_node::rpm_callback,this,std::placeholders::_1));

			sensor_pub = this->create_publisher<std_msgs::msg::Float32>("sensor_data",qos);
			


		}

	private:

		rclcpp::TimerBase::SharedPtr timer;
		serial::Serial& my_serial;
		rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_angle_data;
		rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_rpm_data;
		rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr sensor_pub;

		uint16_t angles[6] = {135,90,90,90,90,90};
		
		uint16_t rpm = 0;
		double sensor_data;

		void rpm_callback(const std_msgs::msg::Float32::SharedPtr msg){
			rpm = static_cast<uint16_t>(msg->data);
		}


		void angles_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg){
			double angles_data[6];
			for (int i=0;i<6;i++){
				angles_data[i] = msg->data[i];
			}


			RCLCPP_INFO(this->get_logger(),"angles 2: %f,%f,%f,%f,%f,%f",angles_data[0],angles_data[1],angles_data[2],angles_data[3],angles_data[4],angles_data[5]);

			
			angles_data[0] = angles_data[0] + 135;


			angles_data[1] = angles_data[1] + 7;


			angles_data[1] = 180 - angles_data[1];


			angles_data[4] = angles_data[4] + 90;
			angles_data[4] = 180 - angles_data[4];

			//スケーリング

			angles_data[1] = angles_data[1] - 90;
			if (angles_data[1] < 0){
				angles_data[1] = angles_data[1] * (64.0/90.0) + 90;
			}
			else{
				angles_data[1] = angles_data[1] * (65.0/90.0) + 90;
			}


			angles_data[2] = angles_data[2] + 5;

			angles_data[3] = angles_data[3] + 4;

			angles_data[3] = 180 - angles_data[3];

			

			angles_data[3] = angles_data[3] - 90;
			angles_data[3] = angles_data[3] * (63.0/90.0) + 90;
			

			angles_data[4] = angles_data[4] - 90;
			angles_data[4] = angles_data[4] * (63.0/90.0) + 90;

			angles_data[5] = angles_data[5] - 90;
			angles_data[5] = angles_data[5] * (63.0/90.0) + 90;
			

			for (int i=0;i<6;i++){
				angles[i] = uint16_t(angles_data[i]);
			}
		}

		void timer_callback(){

			


			std::string data;

			data = std::to_string(angles[0]) + "," + std::to_string(angles[1]) + "," + std::to_string(angles[2]) + "," + std::to_string(angles[3]) + "," + std::to_string(angles[4]) + "," + std::to_string(angles[5]) + "," + std::to_string(rpm) + "\n";


			my_serial.write(data);
			//RCLCPP_INFO(this->get_logger(),"send data : %s",data.c_str());

			// //float data[8] = {0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8};
			// for (int i = 0 ; i < 7 ; i++){
			// 	my_serial.write(data[i]);
			// 	//RCLCPP_INFO(this->get_logger(),"send data : %s",data[i].c_str());
			// }

			// sensor_data = std::stoull(my_serial.read(sizeof(sensor_data)));
			// std_msgs::msg::Float32 sensor_msg; 
			// sensor_msg.data = sensor_data;
			// sensor_pub->publish(sensor_msg);
			if (my_serial.available()) {
				std::string recv = my_serial.readline();
				sensor_pub->publish(std_msgs::msg::Float32().set__data(std::stof(recv)));
				RCLCPP_INFO(this->get_logger(), "recv:%s", recv.c_str());
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

	// Arduinoの起動待ち
	rclcpp::sleep_for(std::chrono::seconds(5));


	my_serial.flushInput();
	my_serial.flushOutput();


	rclcpp::init(argc,argv);
	auto node = std::make_shared<gateway_node>(my_serial);
	rclcpp::spin(node);
	rclcpp::shutdown();

	return 0;

}