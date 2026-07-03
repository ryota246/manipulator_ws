#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "custom_msgs/action/conveyor.hpp"
#include "std_msgs/msg/float32.hpp"


class ConveyorActionServer : public rclcpp::Node
{
public:
    using Conveyor = custom_msgs::action::Conveyor;
    using GoalHandleConveyor = rclcpp_action::ServerGoalHandle<Conveyor>;

    ConveyorActionServer()
    : Node("conveyor_action_server")
    {
        action_server_ = rclcpp_action::create_server<Conveyor>(
            this,
            "conveyor",
            std::bind(&ConveyorActionServer::handle_goal, this,
                      std::placeholders::_1,
                      std::placeholders::_2),
            std::bind(&ConveyorActionServer::handle_cancel, this,
                      std::placeholders::_1),
            std::bind(&ConveyorActionServer::handle_accepted, this,
                      std::placeholders::_1)
        );

        pub = create_publisher<std_msgs::msg::Float32>("rpm_data", 10);

        sub = create_subscription<std_msgs::msg::Float32>(
            "sensor_data",
            10,
            std::bind(&ConveyorActionServer::sensor_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Conveyor Action Server has been started.");
            
    }

private:
    rclcpp_action::Server<Conveyor>::SharedPtr action_server_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub;
    float sensor_data;




    // Goal受信時
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const Conveyor::Goal> goal)
    {
        (void)uuid;
        (void)goal;


        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    // Cancel受信時
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleConveyor> goal_handle)
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
        const std::shared_ptr<GoalHandleConveyor> goal_handle)
    {
        std::thread(
            std::bind(&ConveyorActionServer::execute, this, goal_handle)
        ).detach();
    }

    // アクション本体
    void execute(
        const std::shared_ptr<GoalHandleConveyor> goal_handle)
    {
        auto goal = goal_handle->get_goal();
        (void)goal;




        pub->publish(std_msgs::msg::Float32().set__data(1000.0f));



        bool run = true;
        while (rclcpp::ok() && run) {

            RCLCPP_INFO(this->get_logger(), "sensor_data: %f", sensor_data);

            if (sensor_data < 18.0f){
                run = false;
            }

        }
        pub->publish(std_msgs::msg::Float32().set__data(0.0f));

        
        auto result = std::make_shared<Conveyor::Result>();

        goal_handle->succeed(result);

    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ConveyorActionServer>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}