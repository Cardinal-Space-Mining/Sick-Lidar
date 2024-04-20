/** FOR REFERENCE
 * pub/sub example: https://docs.ros.org/en/iron/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html
 * params example: https://docs.ros.org/en/iron/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-CPP.html
*/

#include "./perception.hpp"


PerceptionNode::PerceptionNode() : rclcpp::Node("perception_node") {

	// setup subs/pubs
	// declare params

	this->accumulator.reset(1.f);	// use params

}

PerceptionNode::~PerceptionNode() {}


void PerceptionNode::scan_cb(const sensor_msgs::msg::PointCloud2::SharedPtr scan) {

	

}

void PerceptionNode::trfm_cb(const geometry_msgs::msg::TransformStamped::SharedPtr trfm) {



}