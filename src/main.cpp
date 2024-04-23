#include "./ldrp/perception.hpp"

#include <memory>

#include <rclcpp/rclcpp.hpp>


int main(int argc, char** argv) {

	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<PerceptionNode>());
	rclcpp::shutdown();

	return 0;

}
