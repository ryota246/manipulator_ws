#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "custom_msgs/msg/pos.hpp"
#include <cmath>




class IK : public rclcpp::Node{
    public:
        IK() : Node("ik_node"){
            RCLCPP_INFO(this->get_logger(),"created ik_node");

            sub = this->create_subscription<custom_msgs::msg::Pos>("pose_data",rclcpp::QoS(1),std::bind(&IK::pose_callback,this,std::placeholders::_1));
            pub = this->create_publisher<std_msgs::msg::Float32MultiArray>("angle_data",10);



        }
    
    private:
        void pose_callback(const custom_msgs::msg::Pos &msg){
            RCLCPP_INFO(this->get_logger(),"Received pose data");

            pose_data = msg;

            std_msgs::msg::Float32MultiArray angle_msg;
            double xr = pose_data.x;
            double yr = pose_data.y;
            double zr = pose_data.z;

            double theta0 = atan2(yr,xr);
            double x = xr - F*sin(theta0);
            double y = yr - F*cos(theta0);
            double z = zr + H;
            

            if (sqrt(x*x + y*y + z*z) > 2*L){
                RCLCPP_WARN(this->get_logger(),"Target is out of reach");
                double errer = sqrt(x*x + y*y + z*z) / (2*L);
                x = x / errer;
                y = y / errer;
                z = z / errer;
                return;
            }else if (sqrt(x*x + y*y + z*z) < ded_zone){
                RCLCPP_WARN(this->get_logger(),"Target is too close");
                double errer = sqrt(x*x + y*y + z*z) / ded_zone;
                x = x / errer;
                y = y / errer;
                z = z / errer;
                return;
            }
            RCLCPP_INFO(this->get_logger(),"x: %f, y: %f, z: %f", x, y, z);
            double fai = acos(sqrt(x*x + y*y + z*z) / (2*L));
            //RCLCPP_INFO(this->get_logger(),"hikisuu: %f",sqrt(x*x + y*y + z*z) / (2*L)); 
            double fai0 = atan2(z,sqrt(x*x + y*y));
            double theta1 = fai0 + fai;
            double theta2 = acos(1 - ((x*x + y*y + z*z)/(2*L*L)));
            //RCLCPP_INFO(this->get_logger(),"hikisuu: %f",double(1 - ((x*x + y*y + z*z)/(2*L*L))));

            theta0 = theta0*180/M_PI;
            theta1 = theta1*180/M_PI;
            theta2 = theta2*180/M_PI;
            
            double theta4;
            bool mode;
            if (pose_data.mode == 0){
                mode = false;
            }else if (pose_data.mode == 1){
                mode = true;
            }else{
                RCLCPP_WARN(this->get_logger(),"Invalid mode: %d", pose_data.mode);
                mode = false;
            }

            if(mode){
                theta4 = -theta0;

            }else{
                theta4 = pose_data.yaw;
            }
            double theta5;
            std::string grip_state = pose_data.grip;
            if (grip_state == "open"){
                theta5 = OPEN_ANGLE;
            }else if (grip_state == "close"){
                theta5 = CLOSE_ANGLE;
            }else{
                RCLCPP_WARN(this->get_logger(),"Invalid grip state: %s", grip_state.c_str());
                theta5 = OPEN_ANGLE;
            }
            

            double theta3 = 270 - theta1 - theta2;


            
            



            angle_msg.data = {float(theta0), float(theta1), float(theta2), float(theta3), float(theta4), float(theta5)};
            pub->publish(angle_msg);
            RCLCPP_INFO(this->get_logger(),"Published angles: %f, %f, %f, %f, %f, %f", theta0, theta1, theta2, theta3, theta4, theta5);
        }

        rclcpp::Subscription<custom_msgs::msg::Pos>::SharedPtr sub;
        custom_msgs::msg::Pos pose_data;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub;
        double L = 150;
        double F = 24.4;
        double H = 115.4;//115.352
        double ded_zone = 10;
        double OPEN_ANGLE = 160;
        double CLOSE_ANGLE = 5;
};

int main(int argc , char **argv){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<IK>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}