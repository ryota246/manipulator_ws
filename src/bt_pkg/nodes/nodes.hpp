#ifndef INCLUDE_MOVE_PKG_NODES_HPP_
#define INCLUDE_MOVE_PKG_NODES_HPP_

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_ros2/bt_action_node.hpp>
#include <cmath>
#include <custom_msgs/action/approach.hpp>
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "custom_msgs/action/conveyor.hpp"
#include "custom_msgs/msg/pos.hpp"
#include "../../BehaviorTree.ROS2/behaviortree_ros2/src/bt_ros2.cpp"
#include "custom_msgs/action/move.hpp"

using namespace BT;

class approach : public RosActionNode<custom_msgs::action::Approach>{
    public:
    approach(const std::string & name, const NodeConfiguration & config, const RosNodeParams & param) : RosActionNode(name, config, param) {}

    static PortsList providedPorts() {
        return {InputPort<std::string>("action_name"),
                OutputPort<float>("goal_x"),
                OutputPort<float>("goal_y"),
                OutputPort<float>("goal_z"),
                OutputPort<float>("goal_yaw"),
                OutputPort<std::string>("label")};
    }

    bool setGoal(custom_msgs::action::Approach::Goal & goal) override {
        RCLCPP_INFO(rclcpp::get_logger("approach"), "Sending goal to action server");
        return true;
    }

    NodeStatus onResultReceived(const WrappedResult& result) override {
        RCLCPP_INFO(rclcpp::get_logger("approach"), "Received result from action server");
        
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(rclcpp::get_logger("approach"), "Action succeeded");
            setOutput("goal_x", result.result->goal_x);
            setOutput("goal_y", result.result->goal_y);
            setOutput("goal_z", result.result->goal_z);
            setOutput("goal_yaw", result.result->goal_yaw);
            setOutput("label", result.result->label);
            return NodeStatus::SUCCESS;
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("approach"), "Action failed");
            return NodeStatus::FAILURE;
        }
    }

};


class conveyor : public RosActionNode<custom_msgs::action::Conveyor>{
    public:
    conveyor(const std::string & name, const NodeConfiguration & config, const RosNodeParams & param) : RosActionNode(name, config, param) {}

    static PortsList providedPorts() {
        return {InputPort<std::string>("action_name"),
                };
    }

    bool setGoal(custom_msgs::action::Conveyor::Goal & goal) override {
        RCLCPP_INFO(rclcpp::get_logger("conveyor"), "Sending goal to action server");
        return true;
    }

    NodeStatus onResultReceived(const WrappedResult& result) override {
        RCLCPP_INFO(rclcpp::get_logger("conveyor"), "Received result from action server");
        
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(rclcpp::get_logger("conveyor"), "Action succeeded");
            
            return NodeStatus::SUCCESS;
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("conveyor"), "Action failed");
            return NodeStatus::FAILURE;
        }
    }

};


class move : public RosTopicPubNode<custom_msgs::msg::Pos>{
    public:
    move(const std::string & name, const NodeConfiguration & config, const RosNodeParams& params) : RosTopicPubNode<custom_msgs::msg::Pos>(name, config, params) {}

    static PortsList providedPorts() {
        return {InputPort<std::string>("topic_name"),
                InputPort<float>("x"),
                InputPort<float>("y"),
                InputPort<float>("z"),
                InputPort<float>("yaw"),
                InputPort<std::string>("grip"),
                InputPort<int>("mode")};
    }

    bool setMessage(custom_msgs::msg::Pos    & msg) override {
        msg.x = getInput<float>("x").value();
        msg.y = getInput<float>("y").value();
        msg.z = getInput<float>("z").value();
        msg.yaw = getInput<float>("yaw").value();
        msg.grip = getInput<std::string>("grip").value();
        msg.mode = getInput<int>("mode").value();
               return true;
    }
};


class smooth_move : public RosActionNode<custom_msgs::action::Move>{
    public:
    smooth_move(const std::string & name, const NodeConfiguration & config, const RosNodeParams & param) : RosActionNode(name, config, param) {}

    static PortsList providedPorts() {
        return {InputPort<std::string>("action_name"),
                InputPort<float>("s_x"),
                InputPort<float>("s_y"),
                InputPort<float>("s_z"),
                InputPort<float>("s_yaw"),
                InputPort<float>("g_x"),
                InputPort<float>("g_y"),
                InputPort<float>("g_z"),
                InputPort<float>("g_yaw"),
                InputPort<float>("speed"),
                InputPort<std::string>("grip")};
    }

    bool setGoal(custom_msgs::action::Move::Goal & goal) override {
        goal.s_x = getInput<float>("s_x").value();
        goal.s_y = getInput<float>("s_y").value();
        goal.s_z = getInput<float>("s_z").value();
        goal.s_yaw = getInput<float>("s_yaw").value();
        goal.g_x = getInput<float>("g_x").value();
        goal.g_y = getInput<float>("g_y").value();
        goal.g_z = getInput<float>("g_z").value();
        goal.g_yaw = getInput<float>("g_yaw").value();
        goal.speed = getInput<float>("speed").value();
        goal.grip = getInput<std::string>("grip").value();
        RCLCPP_INFO(rclcpp::get_logger("smooth_move"), "Sending goal to action server");

        return true;
    }

    NodeStatus onResultReceived(const WrappedResult& result) override {
        RCLCPP_INFO(rclcpp::get_logger("smooth_move"), "Received result from action server");
        
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(rclcpp::get_logger("smooth_move"), "Action succeeded");
            return NodeStatus::SUCCESS;
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("smooth_move"), "Action failed");
            return NodeStatus::FAILURE;
        }
    }

};
    




class finish : public StatefulActionNode{
    public:
    finish(const std::string & name, const NodeConfiguration & config) : StatefulActionNode(name, config) {}

    static PortsList providedPorts() {
        return {};
    }

    NodeStatus onStart() override {
        RCLCPP_INFO(rclcpp::get_logger("finish"), "Finished all tasks!");
        return NodeStatus::RUNNING;
    }

    NodeStatus onRunning() override {
        return NodeStatus::RUNNING;
    }

    void onHalted() override {}

};


#endif