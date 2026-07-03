
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "../nodes/nodes.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <behaviortree_cpp/xml_parsing.h>


using namespace BT;


class bt_node : public rclcpp::Node{
    public:
    bt_node(const std::string & name) : Node(name){
        RCLCPP_INFO(this->get_logger(),"created bt_node");



        timer = this->create_wall_timer(std::chrono::milliseconds(100),std::bind(&bt_node::timer_callback,this));
        }

    
        void timer_callback(){
            if(rclcpp::ok()){
                //RCLCPP_INFO(this->get_logger(),"tick");
                tree.tickOnce();
            }

        
        }


        void setTree(Tree&& tree){
            this->tree = std::move(tree);
            RCLCPP_INFO(rclcpp::get_logger("kfs_bt"), "Behavior tree set successfully.");

        }

        rclcpp::TimerBase::SharedPtr timer;
        Tree tree;
};

int main(int argc, char **argv){
    rclcpp::init(argc,argv);

    auto node = std::make_shared<bt_node>("bt_node");




    BehaviorTreeFactory factory;

    auto blackboard = Blackboard::create();


    RosNodeParams param;
    param.nh = node;

    factory.registerNodeType<approach>("approach",param);
    factory.registerNodeType<conveyor>("conveyor",param);
    factory.registerNodeType<move>("move",param);
    factory.registerNodeType<smooth_move>("smooth_move",param);
    factory.registerNodeType<finish>("finish");

    // XMLファイルから実行するツリー名を取得
	// main_tree_to_execute 属性があればそれを使用、なければ最初の BehaviorTree ID を使用
	auto get_tree_name = [](const std::string& path) -> std::string {
		std::ifstream f(path);
		std::string xml((std::istreambuf_iterator<char>(f)), {});
		auto p = xml.find("main_tree_to_execute=\"");
		if (p != std::string::npos) {
			p += 22;
			return xml.substr(p, xml.find('"', p) - p);
		}
		p = xml.find("<BehaviorTree ID=\"");
		if (p != std::string::npos) {
			p += 18;
			return xml.substr(p, xml.find('"', p) - p);
		}
		return "Main";
	};


    // メインツリーをロード
	std::string main_file = ament_index_cpp::get_package_share_directory("bt_pkg") + "/config/main3.xml";
	factory.registerBehaviorTreeFromFile(main_file);

	std::string tree_name = get_tree_name(main_file);
	auto tree = factory.createTree(tree_name, blackboard);

	node->setTree(std::move(tree));






    rclcpp::spin(node);
    rclcpp::shutdown();
}