from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
	current_pkg = FindPackageShare('sick_perception')

	pointcloud_topic_cfg = LaunchConfiguration('pointcloud_topic', default='/uesim/scan')
	declare_pointcloud_topic_arg = DeclareLaunchArgument(
		'pointcloud_topic',
		default_value = pointcloud_topic_cfg,
		description = 'Input point cloud topic name'
	)

	pose_topic_cfg = LaunchConfiguration('pose_topic', default = '/uesim/pose')
	declare_pose_topic_arg = DeclareLaunchArgument(
		'pose_topic',
		default_value = pose_topic_cfg,
		description = 'Input pose topic name'
	)

	params_yaml_path = PathJoinSubstitution([current_pkg, 'config', 'params.yaml'])

	ldrp_node = Node(
		name = 'ldrp',
		package = 'sick_perception',
		executable = 'ldrp_node',
		output = 'screen',
		parameters = [params_yaml_path],
		remappings = [
			('scan', pointcloud_topic_cfg),
			('pose', pose_topic_cfg)
		]
	)

	return LaunchDescription([
		declare_pointcloud_topic_arg,
		declare_pose_topic_arg,
		ldrp_node
	])