// Copyright (c) 2023 Davide Faconti, Unmanned Life
//
// このファイルはApache License, Version 2.0（以下「ライセンス」）に基づいてライセンスされています；
// ライセンスに従わない限り、このファイルを使用することはできません。
// ライセンスのコピーは以下から入手できます：
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// 適用される法律または書面による合意がない限り、ライセンスに基づいて配布されるソフトウェアは
// 「現状のまま」提供され、明示または黙示を問わず、いかなる保証もありません。
// 詳細はライセンスを参照してください。

#pragma once

#include <memory>
#include <string>
#include <rclcpp/executors.hpp>
#include <rclcpp/allocator/allocator_common.hpp>
#include "behaviortree_cpp/condition_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_ros2/ros_node_params.hpp"

namespace BT
{

/**
 * @brief ROSパブリッシャをラップする抽象クラス
 *
 */
template <class TopicT>
class RosTopicPubNode : public BT::ConditionNode
{
public:
  // Type definitions
  using Publisher = typename rclcpp::Publisher<TopicT>;

  /** このクラスは直接インスタンス化せず、ファクトリが行います。
   * このクラスをファクトリに登録するには、以下を使います：
   *
   *    RegisterRosAction<DerivedClass>(factory, params)
   *
   * external_action_clientが設定されていない場合、コンストラクタが自動で生成します。
   * */
  explicit RosTopicPubNode(const std::string& instance_name, const BT::NodeConfig& conf,
                           const RosNodeParams& params);

  virtual ~RosTopicPubNode() = default;

  /**
   * @brief RosTopicPubNodeのサブクラスで追加のポートがある場合は
   * providedPortsメソッドを実装し、providedBasicPortsを呼び出してください。
   *
   * @param addition 追加するポート
   * @return 基本ポートとノード固有ポートを含むPortsList
   */
  static PortsList providedBasicPorts(PortsList addition)
  {
    PortsList basic = { InputPort<std::string>("topic_name", "__default__placeholder__",
                                               "Topic name") };
    basic.insert(addition.begin(), addition.end());
    return basic;
  }

  /**
   * @brief BTポートのリストを作成
   * @return 基本ポートとノード固有ポートを含むPortsList
   */
  static PortsList providedPorts()
  {
    return providedBasicPorts({});
  }

  NodeStatus tick() override final;

  /**
   * @brief setMessageはtick内で呼ばれるコールバックで、ユーザーが
   * 送信するメッセージを渡すために使います。
   *
   * @param msg メッセージ
   * @return 問題があればfalseを返し、メッセージは送信されません。
   * ConditionはFAILUREを返します。
   */
  virtual bool setMessage(TopicT& msg) = 0;

protected:
  std::shared_ptr<rclcpp::Node> node_;
  std::string prev_topic_name_;
  bool topic_name_may_change_ = false;

private:
  std::shared_ptr<Publisher> publisher_;

  bool createPublisher(const std::string& topic_name);
};

//----------------------------------------------------------------
//---------------------- DEFINITIONS -----------------------------
//----------------------------------------------------------------

template <class T>
inline RosTopicPubNode<T>::RosTopicPubNode(const std::string& instance_name,
                                           const NodeConfig& conf,
                                           const RosNodeParams& params)
  : BT::ConditionNode(instance_name, conf), node_(params.nh)
{
  // check port remapping
  auto portIt = config().input_ports.find("topic_name");
  if(portIt != config().input_ports.end())
  {
    const std::string& bb_topic_name = portIt->second;

    if(bb_topic_name.empty() || bb_topic_name == "__default__placeholder__")
    {
      if(params.default_port_value.empty())
      {
        throw std::logic_error("Both [topic_name] in the InputPort and the RosNodeParams "
                               "are empty.");
      }
      else
      {
        createPublisher(params.default_port_value);
      }
    }
    else if(!isBlackboardPointer(bb_topic_name))
    {
      // If the content of the port "topic_name" is not
      // a pointer to the blackboard, but a static string, we can
      // create the client in the constructor.
      createPublisher(bb_topic_name);
    }
    else
    {
      topic_name_may_change_ = true;
      // createPublisher will be invoked in the first tick().
    }
  }
  else
  {
    if(params.default_port_value.empty())
    {
      throw std::logic_error("Both [topic_name] in the InputPort and the RosNodeParams "
                             "are empty.");
    }
    else
    {
      createPublisher(params.default_port_value);
    }
  }
}

template <class T>
inline bool RosTopicPubNode<T>::createPublisher(const std::string& topic_name)
{
  if(topic_name.empty())
  {
    throw RuntimeError("topic_name is empty");
  }

  publisher_ = node_->create_publisher<T>(topic_name, 1);
  prev_topic_name_ = topic_name;
  return true;
}

template <class T>
inline NodeStatus RosTopicPubNode<T>::tick()
{
  // まず、subscriber_が有効か、ポートのtopic_nameが変更されていないかを確認します。
  // そうでなければ、新しいsubscriberを作成します。
  if(!publisher_ || (status() == NodeStatus::IDLE && topic_name_may_change_))
  {
    std::string topic_name;
    getInput("topic_name", topic_name);
    if(prev_topic_name_ != topic_name)
    {
      createPublisher(topic_name);
    }
  }

  T msg;
  if(!setMessage(msg))
  {
    return NodeStatus::FAILURE;
  }
  publisher_->publish(msg);
  return NodeStatus::SUCCESS;
}

}  // namespace BT
