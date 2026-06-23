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

            double theta = atan2(xr,yr);
            double x = xr - F*sin(theta);
            double y = yr - F*cos(theta);
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

            double theta0 = atan2(y,x);
            double theta1 = abs(acos(std::clamp((x*x + y*y + z*z - 2*L*L)/(2*L*L) , -1.0, 1.0)));
            double theta2 = -asin((L + L*cos(theta1))/sqrt(x*x + y*y + z*z)) + atan2(z,sqrt(x*x + y*y)) + M_PI/2;
            double theta3 = 2*M_PI - M_PI/2 - theta1 - (M_PI - theta2);
            double theta4 = pose_data.yaw;
            double theta5 = pose_data.grip;

            theta0 = theta0*180/M_PI;
            theta1 = theta1*180/M_PI;
            theta2 = theta2*180/M_PI;
            theta3 = theta3*180/M_PI;

            theta2 = -(theta2 - 90);

            theta3 = theta3 - 90;


            angle_msg.data = {float(theta0), float(theta1), float(theta2), float(theta3), float(theta4), float(theta5)};
            pub->publish(angle_msg);
        }

        rclcpp::Subscription<custom_msgs::msg::Pos>::SharedPtr sub;
        custom_msgs::msg::Pos pose_data;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub;
        double L = 150;
        double F = 24.4;
        double H = 115.4;//115.352
        double ded_zone = 10;
};

int main(int argc , char **argv){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<IK>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}