#include <memory>
#include <thread>
#include <chrono>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "custom_msgs/action/move.hpp"
#include "std_msgs/msg/float32.hpp"
#include "custom_msgs/msg/pos.hpp"


class move : public rclcpp::Node
{
public:
    using Move = custom_msgs::action::Move;
    using GoalHandleMove = rclcpp_action::ServerGoalHandle<Move>;

    move()
    : Node("move_action_server")
    {
        action_server_ = rclcpp_action::create_server<Move>(
            this,
            "move",
            std::bind(&move::handle_goal, this,
                      std::placeholders::_1,
                      std::placeholders::_2),
            std::bind(&move::handle_cancel, this,
                      std::placeholders::_1),
            std::bind(&move::handle_accepted, this,
                      std::placeholders::_1)
        );

        pub = create_publisher<custom_msgs::msg::Pos>("pose_data", 10);

        RCLCPP_INFO(this->get_logger(), "Move Action Server has been started.");

            
    }

private:
    rclcpp_action::Server<Move>::SharedPtr action_server_;
    rclcpp::Publisher<custom_msgs::msg::Pos>::SharedPtr pub;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub;
    float sensor_data;




    // Goal受信時
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const Move::Goal> goal)
    {
        (void)uuid;
        (void)goal;


        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    // Cancel受信時
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleMove> goal_handle)
    {
        (void)goal_handle;


        return rclcpp_action::CancelResponse::ACCEPT;
    }


    void sensor_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        sensor_data = msg->data;
        RCLCPP_INFO(this->get_logger(), "Received sensor data: %f", msg->data);
    }

    // Goal受理後
    void handle_accepted(
        const std::shared_ptr<GoalHandleMove> goal_handle)
    {
        std::thread(
            std::bind(&move::execute, this, goal_handle)
        ).detach();
    }

    // アクション本体
    void execute(
        const std::shared_ptr<GoalHandleMove> goal_handle)
    {
        auto goal = goal_handle->get_goal();
        (void)goal;

        float s_x = goal->s_x;
        float s_y = goal->s_y;
        float s_z = goal->s_z;
        float s_yaw = goal->s_yaw;
        float g_x = goal->g_x;
        float g_y = goal->g_y;
        float g_z = goal->g_z;
        float g_yaw = goal->g_yaw;
        float speed = goal->speed;
        
        float dx = g_x - s_x;
        float dy = g_y - s_y;
        float dz = g_z - s_z;
        float distance = std::sqrt(dx * dx + dy * dy + dz * dz);


        double total_time = distance / speed;

        rclcpp::Rate rate(10);
        auto start_time = this->now();

        while (rclcpp::ok()) {
            

            double t = (this->now() - start_time).seconds() / total_time;

            if(t > 1.0){
                t = 1.0;
            }

            custom_msgs::msg::Pos msg;

            msg.x = s_x + t * dx;
            msg.y = s_y + t * dy;
            msg.z = s_z + t * dz;
            msg.yaw = s_yaw + t * (g_yaw - s_yaw);
            msg.grip = goal->grip;
            msg.mode = 0;

            pub->publish(msg);

            RCLCPP_INFO(this->get_logger(), "Published position: x=%f, y=%f, z=%f, yaw=%f", msg.x, msg.y, msg.z, msg.yaw);
        
            if(t >= 1.0){
                break;
            }

            rate.sleep();
        }



        auto result = std::make_shared<Move::Result>();

        goal_handle->succeed(result);

    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<move>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}