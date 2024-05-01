#include <functional>
#include <memory>
#include <span>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>
#include <networktables/FloatArrayTopic.h>
#include <networktables/RawTopic.h>


class NTBridgeNode : public rclcpp::Node {
public:
	NTBridgeNode(
		bool use_nt_client = false,
		int nt_client_team = -1,
		const char* nt_client_server = "localhost",
		unsigned int nt_client_port = 0
	) : Node("nt_bridge_node") {
		nt::NetworkTableInstance inst = nt::NetworkTableInstance::GetDefault();

		bool using_client = false;
		if(use_nt_client) {
			if(nt_client_team >= 0) {
				inst.StartClient4("nt_bridge_node");
				inst.SetServerTeam(
					nt_client_team,
					nt_client_port
				);
				using_client = true;
				RCLCPP_INFO(this->get_logger(), "Started NT client using team number: %d", nt_client_team);
			}
			if(nt_client_server != nullptr) {
				inst.StartClient4("nt_bridge_node");
				inst.SetServer(
					nt_client_server,
					nt_client_port
				);
				using_client = true;
				RCLCPP_INFO(this->get_logger(), "Started NT client using server: %s", nt_client_server);
			}
		}
		if(!using_client) {
			inst.StartServer();
			RCLCPP_INFO(this->get_logger(), "Started NT server.");
		}

		this->nt_scan_sub = inst.GetRawTopic("uesim/scan").Subscribe( "PointXYZ_[]", {} );
		this->nt_pose_sub = inst.GetFloatArrayTopic("uesim/pose").Subscribe( {} );
		this->nt_imu_sub = inst.GetFloatArrayTopic("uesim/imu").Subscribe( {} );
		this->nt_grid_pub = inst.GetRawTopic("nt-bridge/grid").Publish( "Grid_U8" );

		inst.AddListener(this->nt_scan_sub, nt::EventFlags::kValueAll, std::bind(&NTBridgeNode::nt_scan_cb, this, std::placeholders::_1));
		inst.AddListener(this->nt_pose_sub, nt::EventFlags::kValueAll, std::bind(&NTBridgeNode::nt_pose_cb, this, std::placeholders::_1));
		inst.AddListener(this->nt_imu_sub, nt::EventFlags::kValueAll, std::bind(&NTBridgeNode::nt_imu_cb, this, std::placeholders::_1));

		this->scan_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/uesim/scan", 1);
		this->pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/uesim/pose", 1);
		this->imu_pub = this->create_publisher<sensor_msgs::msg::Imu>("/uesim/imu", 1);
		this->grid_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/ldrp/obstacle_grid", 1, std::bind(&NTBridgeNode::ros_grid_cb, this, std::placeholders::_1));
	}
	~NTBridgeNode() = default;

protected:
	void nt_scan_cb(const nt::Event& event) {
		const nt::Value& val = event.GetValueEventData()->value;
		if( val.IsRaw() ) {
			std::span<const uint8_t> data = val.GetRaw();
			const size_t
				nelems = data.size() / 16,
				nbytes = nelems * 16;

			sensor_msgs::msg::PointCloud2 cloud;
			cloud.data.resize(nbytes);
			memcpy(cloud.data.data(), data.data(), nbytes);

			cloud.width = nelems;
			cloud.height = 1;
			cloud.is_bigendian = false;
			cloud.point_step = 16;
			cloud.row_step = nbytes;
			cloud.is_dense = true;
			cloud.header.frame_id = "1";

			cloud.fields.resize(3);
			cloud.fields[0].name = "x";
			cloud.fields[0].offset = 0;
			cloud.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
			cloud.fields[0].count = 1;
			cloud.fields[1].name = "y";
			cloud.fields[1].offset = 4;
			cloud.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
			cloud.fields[1].count = 1;
			cloud.fields[2].name = "z";
			cloud.fields[2].offset = 8;
			cloud.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
			cloud.fields[2].count = 1;

			cloud.header.stamp.sec = static_cast<int32_t>( val.time() / 1000000L );
			cloud.header.stamp.nanosec = static_cast<uint32_t>( val.time() % 1000000L ) * 1000U;
			cloud.header.frame_id = "lidar";

			this->scan_pub->publish(cloud);

			RCLCPP_INFO(this->get_logger(), "Published Scan!");
		}
	}
	void nt_pose_cb(const nt::Event& event) {
		const nt::Value& val = event.GetValueEventData()->value;
		if( val.IsFloatArray() ) {
			std::span<const float> data = val.GetFloatArray();

			if(data.size() >= 7) {
				geometry_msgs::msg::PoseStamped pose;
				
				pose.pose.position.x = static_cast<double>( data[0] );
				pose.pose.position.y = static_cast<double>( data[1] );
				pose.pose.position.z = static_cast<double>( data[2] );

				pose.pose.orientation.w = static_cast<double>( data[3] );
				pose.pose.orientation.x = static_cast<double>( data[4] );
				pose.pose.orientation.y = static_cast<double>( data[5] );
				pose.pose.orientation.z = static_cast<double>( data[6] );

				pose.header.stamp.sec = static_cast<int32_t>( val.time() / 1000000L );
				pose.header.stamp.nanosec = static_cast<uint32_t>( val.time() % 1000000L ) * 1000U;
				pose.header.frame_id = "world";

				this->pose_pub->publish(pose);

				RCLCPP_INFO(this->get_logger(), "Published Pose!");
			}
		}
	}
	void nt_imu_cb(const nt::Event& event) {
		const nt::Value& val = event.GetValueEventData()->value;
		if( val.IsFloatArray() ) {
			std::span<const float> data = val.GetFloatArray();

			if(data.size() >= 10) {
				sensor_msgs::msg::Imu imu;

				imu.orientation.w = static_cast<double>( data[0] );
				imu.orientation.x = static_cast<double>( data[1] );
				imu.orientation.y = static_cast<double>( data[2] );
				imu.orientation.z = static_cast<double>( data[3] );

				imu.angular_velocity.x = static_cast<double>( data[4] );
				imu.angular_velocity.y = static_cast<double>( data[5] );
				imu.angular_velocity.z = static_cast<double>( data[6] );

				imu.linear_acceleration.x = static_cast<double>( data[7] );
				imu.linear_acceleration.y = static_cast<double>( data[8] );
				imu.linear_acceleration.z = static_cast<double>( data[9] );

				imu.header.stamp.sec = static_cast<int32_t>( val.time() / 1000000L );
				imu.header.stamp.nanosec = static_cast<int32_t>( val.time() % 1000000L ) * 1000U;
				imu.header.frame_id = "world";

				this->imu_pub->publish(imu);

				RCLCPP_INFO(this->get_logger(), "Published IMU data!");
			}
		}
	}
	void ros_grid_cb(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr& grid) {
		const size_t len = static_cast<size_t>( grid->info.width * grid->info.height ) + 20;
		uint8_t* _data = new uint8_t[len];

		reinterpret_cast<int32_t*>(_data)[0] = static_cast<int32_t>( grid->info.width );	// x cells
		reinterpret_cast<int32_t*>(_data)[1] = static_cast<int32_t>( grid->info.height );	// y cells
		reinterpret_cast<float*>(_data)[2] = static_cast<float>( grid->info.origin.position.x );	// x origin
		reinterpret_cast<float*>(_data)[3] = static_cast<float>( grid->info.origin.position.y );	// y origin
		reinterpret_cast<float*>(_data)[4] = static_cast<float>( grid->info.resolution );
		memcpy(_data + 20, grid->data.data(), grid->data.size());

		this->nt_grid_pub.Set(
			std::span<const uint8_t>{
				_data,
				_data + len
			},
			(static_cast<int64_t>(grid->header.stamp.sec) * 1000000L) + (static_cast<int64_t>(grid->header.stamp.nanosec) / 1000L)
		);

		delete[] _data;
		RCLCPP_INFO(this->get_logger(), "Published Grid!");
	}


protected:
	nt::RawSubscriber nt_scan_sub;
	nt::FloatArraySubscriber nt_pose_sub, nt_imu_sub;
	nt::RawPublisher nt_grid_pub;

	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr scan_pub;
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub;
	rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
	rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_sub;


};


int main(int argc, char** argv) {

	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<NTBridgeNode>());
	rclcpp::shutdown();

	return 0;

}
