/*******************************************************************************
*   Copyright (C) 2024 Cardinal Space Mining Club                              *
*                                                                              *
*   Unless required by applicable law or agreed to in writing, software        *
*   distributed under the License is distributed on an "AS IS" BASIS,          *
*   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
*   See the License for the specific language governing permissions and        *
*   limitations under the License.                                             *
*                                                                              *
*                                ;xxxxxxx:                                     *
*                               ;$$$$$$$$$       ...::..                       *
*                               $$$$$$$$$$x   .:::::::::::..                   *
*                            x$$$$$$$$$$$$$$::::::::::::::::.                  *
*                        :$$$$$&X;      .xX:::::::::::::.::...                 *
*                .$$Xx++$$$$+  :::.     :;:   .::::::.  ....  :                *
*               :$$$$$$$$$  ;:      ;xXXXXXXXx  .::.  .::::. .:.               *
*              :$$$$$$$$: ;      ;xXXXXXXXXXXXXx: ..::::::  .::.               *
*             ;$$$$$$$$ ::   :;XXXXXXXXXXXXXXXXXX+ .::::.  .:::                *
*              X$$$$$X : +XXXXXXXXXXXXXXXXXXXXXXXX; .::  .::::.                *
*               .$$$$ :xXXXXXXXXXXXXXXXXXXXXXXXXXXX.   .:::::.                 *
*                X$$X XXXXXXXXXXXXXXXXXXXXXXXXXXXXx:  .::::.                   *
*                $$$:.XXXXXXXXXXXXXXXXXXXXXXXXXXX  ;; ..:.                     *
*                $$& :XXXXXXXXXXXXXXXXXXXXXXXX;  +XX; X$$;                     *
*                $$$::XXXXXXXXXXXXXXXXXXXXXX: :XXXXX; X$$;                     *
*                X$$X XXXXXXXXXXXXXXXXXXX; .+XXXXXXX; $$$                      *
*                $$$$ ;XXXXXXXXXXXXXXX+  +XXXXXXXXx+ X$$$+                     *
*              x$$$$$X ;XXXXXXXXXXX+ :xXXXXXXXX+   .;$$$$$$                    *
*             +$$$$$$$$ ;XXXXXXx;;+XXXXXXXXX+    : +$$$$$$$$                   *
*              +$$$$$$$$: xXXXXXXXXXXXXXX+      ; X$$$$$$$$                    *
*               :$$$$$$$$$. +XXXXXXXXX:      ;: x$$$$$$$$$                     *
*               ;x$$$$XX$$$$+ .;+X+      :;: :$$$$$xX$$$X                      *
*              ;;;;;;;;;;X$$$$$$$+      :X$$$$$$&.                             *
*              ;;;;;;;:;;;;;x$$$$$$$$$$$$$$$$x.                                *
*              :;;;;;;;;;;;;.  :$$$$$$$$$$X                                    *
*               .;;;;;;;;:;;    +$$$$$$$$$                                     *
*                 .;;;;;;.       X$$$$$$$:                                     *
*                                                                              *
*******************************************************************************/
/** FOR REFERENCE
 * pub/sub example: https://docs.ros.org/en/iron/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html
 * params example: https://docs.ros.org/en/iron/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-CPP.html
*/

#include "./perception.hpp"
#include "./filtering.hpp"

#include <limits>
#include <memory>
#include <type_traits>
#include <ratio>
#include <chrono>

#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>


namespace util {

    template <typename T>
    struct identity { typedef T type; };

    template <typename T>
    void declare_param(rclcpp::Node* node, const std::string param_name, T& param, const typename identity<T>::type& default_value)
    {
        node->declare_parameter(param_name, default_value);
        node->get_parameter(param_name, param);
    }


    template<typename FT = float>
    static inline int64_t floatSecondsToIntMicros(const FT v)
    {
        static_assert(std::is_floating_point_v<FT>, "");
        return static_cast<int64_t>(v * static_cast<FT>(1e6));
    }

    template<typename T = uint32_t>
    static inline int64_t constructTimestampMicros(const T seconds, const T nanoseconds)
    {
        return (static_cast<int64_t>(seconds) * 1000000L) + (static_cast<int64_t>(nanoseconds) / 1000L);
    }

    inline tf2::TimePoint toTf2TimePoint(const builtin_interfaces::msg::Time& t)
    {
        return tf2::TimePoint{
            std::chrono::seconds{ t.sec } +
            std::chrono::nanoseconds{ t.nanosec } };
    }


    void _test(rclcpp::Node* node)
    {
        pcl::PointCloud<pcl::PointXYZ> _c;
        sensor_msgs::msg::PointCloud2 _c2;
        pcl::toROSMsg(_c, _c2);
        RCLCPP_INFO(node->get_logger(),
            "\n-------------------------------- TESTING --------------------------------"
            "\nPointXYZ \"frame_id\": %s"
            "\nPointCloud2 total fields: %ld"
            "\nPointCloud2 field #1: { name: %s, offset: %d, datatype: %d, count: %d }"
            "\nPointCloud2 field #2: { name: %s, offset: %d, datatype: %d, count: %d }"
            "\nPointCloud2 field #3: { name: %s, offset: %d, datatype: %d, count: %d }"
            "\n-------------------------------------------------------------------------",
            _c.header.frame_id.c_str(),
            _c2.fields.size(),
            _c2.fields[0].name.c_str(),
            _c2.fields[0].offset,
            _c2.fields[0].datatype,
            _c2.fields[0].count,
            _c2.fields[1].name.c_str(),
            _c2.fields[1].offset,
            _c2.fields[1].datatype,
            _c2.fields[1].count,
            _c2.fields[2].name.c_str(),
            _c2.fields[2].offset,
            _c2.fields[2].datatype,
            _c2.fields[2].count
        );
    }

};


PerceptionNode::PerceptionNode() :
    rclcpp::Node("perception_node"),
    tf_buffer{ std::make_shared<rclcpp::Clock>(RCL_ROS_TIME) },
    tf_listener{ tf_buffer }
{
    // RCLCPP_INFO(this->get_logger(), "Perception Node Initialization!");

    std::string scan_topic;
    util::declare_param(this, "scan_topic", scan_topic, "lidar_scan");
    // setup subs/pubs
    this->scan_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        scan_topic, rclcpp::SensorDataQoS{},
        [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr& scan){ this->scan_callback(scan); } );
    this->grid_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "obstacle_grid", rclcpp::SensorDataQoS{} );

    //get parameters
    util::declare_param(this, "max_tf_wait_time_s", this->_config.max_tf_wait_time_s, this->_config.max_tf_wait_time_s);
    util::declare_param(this, "map_resolution_cm", this->_config.map_resolution_cm, this->_config.map_resolution_cm);
    util::declare_param(this, "voxel_size_cm", this->_config.voxel_size_cm, this->_config.voxel_size_cm);
    util::declare_param(this, "max_z_thresh_cm", this->_config.max_z_thresh_cm, this->_config.max_z_thresh_cm);
    util::declare_param(this, "min_z_thresh_cm", this->_config.min_z_thresh_cm, this->_config.min_z_thresh_cm);
    util::declare_param(this, "pmf_max_range_cm", this->_config.pmf_max_range_cm, this->_config.pmf_max_range_cm);
    util::declare_param(this, "pmf_window_base", this->_config.pmf_window_base, this->_config.pmf_window_base);
    util::declare_param(this, "pmf_max_window_size_cm", this->_config.pmf_max_window_size_cm, this->_config.pmf_max_window_size_cm);
    util::declare_param(this, "pmf_cell_size_cm", this->_config.pmf_cell_size_cm, this->_config.pmf_cell_size_cm);
    util::declare_param(this, "pmf_init_distance_cm", this->_config.pmf_init_distance_cm, this->_config.pmf_init_distance_cm);
    util::declare_param(this, "pmf_max_distance_cm", this->_config.pmf_max_distance_cm, this->_config.pmf_max_distance_cm);
    util::declare_param(this, "pmf_slope", this->_config.pmf_slope, this->_config.pmf_slope);
    util::declare_param(this, "map_frame", this->_config.map_frame, this->_config.map_frame);
    util::declare_param(this, "lidar_frame", this->_config.lidar_origin_frame, this->_config.lidar_origin_frame);

    // log params
    RCLCPP_INFO(this->get_logger(),
        "\n--------- Configured Params ---------"
        "\nMap resolution (cm): %f"
        "\nVoxel size (cm): %f"
        "\nMax Z threshold (cm): %f"
        "\nMin Z threshold (cm): %f"
        "\nPMF range cutoff (cm): %f"
        "\nPMF window base: %f"
        "\nPMF max window size (cm): %f"
        "\nPMF cell size (cm): %f"
        "\nPMF init distance (cm): %f"
        "\nPMF max distance (cm): %f"
        "\nPMF slope: %f"
        "\nTf wait time (s): %f"
        "\n-------------------------------------",
        this->_config.map_resolution_cm,
        this->_config.voxel_size_cm,
        this->_config.max_z_thresh_cm,
        this->_config.min_z_thresh_cm,
        this->_config.pmf_max_range_cm,
        this->_config.pmf_window_base,
        this->_config.pmf_max_window_size_cm,
        this->_config.pmf_cell_size_cm,
        this->_config.pmf_init_distance_cm,
        this->_config.pmf_max_distance_cm,
        this->_config.pmf_slope,
        this->_config.max_tf_wait_time_s
    );

    this->accumulator.reset(this->_config.map_resolution_cm * 1e-2f);	// use params

}

PerceptionNode::~PerceptionNode() {}



void PerceptionNode::scan_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& scan)
{
    // RCLCPP_INFO(this->get_logger(), "Scan callback called!");

    pcl::PointCloud<pcl::PointXYZ> point_cloud{};
    Eigen::Vector3f origin = Eigen::Vector3f::Zero();
    try
    {
        sensor_msgs::msg::PointCloud2 scan_{};

        auto tf = this->tf_buffer.lookupTransform(
            this->_config.map_frame,
            scan->header.frame_id,
            util::toTf2TimePoint(scan->header.stamp),
            std::chrono::nanoseconds{ static_cast<int64_t>(this->_config.max_tf_wait_time_s * 1e9) } );

        // TODO: use last transform as final attempt

        tf2::doTransform(*scan, scan_, tf);
        pcl::fromROSMsg(scan_, point_cloud);

        origin.x() = -tf.transform.translation.x;   // use this transform at least for origin
        origin.y() = -tf.transform.translation.y;
        origin.z() = -tf.transform.translation.z;
    }
    catch(const std::exception& e)
    {
        RCLCPP_INFO(this->get_logger(), "[SICK PERCEPTION]: Failed to transform scan.\n\twhat(): %s", e.what());
        return;
    }

    try
    {
        // lookup actual lidar origin
        auto tf = this->tf_buffer.lookupTransform(
            this->_config.lidar_origin_frame,
            this->_config.map_frame,
            util::toTf2TimePoint(scan->header.stamp) );

        origin.x() = tf.transform.translation.x;
        origin.y() = tf.transform.translation.y;
        origin.z() = tf.transform.translation.z;
    }
    catch(const std::exception& e)
    {
        RCLCPP_INFO(this->get_logger(), "[SICK PERCEPTION]: Failed to lookup scan origin.");
    }

    this->process_and_export(point_cloud, origin);
}

void PerceptionNode::process(const pcl::PointCloud<pcl::PointXYZ>& cloud, const Eigen::Vector3f& origin)
{
    // RCLCPP_INFO(this->get_logger(), "Processing...");

    static const pcl::Indices DEFAULT_NO_SELECTION = pcl::Indices{};

    pcl::PointCloud<pcl::PointXYZ>
        voxel_cloud;
    pcl::Indices
        z_high_filtered{},
        z_low_subset_filtered{},
        z_mid_filtered_obstacles{},
        pre_pmf_range_filtered{},
        pmf_filtered_ground{},
        pmf_filtered_obstacles{},
        combined_obstacles{};

#if LDRP_ENABLE_TUNING
    float
        voxel_size_m			= (float) this->get_parameter("voxel_size_cm").as_double() * 1e-2f,
        max_z_thresh_m			= (float) this->get_parameter("max_z_thresh_cm").as_double() * 1e-2f,
        min_z_thresh_m			= (float) this->get_parameter("min_z_thresh_cm").as_double() * 1e-2f,
        max_pmf_range_m			= (float) this->get_parameter("pmf_max_range_cm").as_double() * 1e-2f,
        pmf_window_base			= (float) this->get_parameter("pmf_window_base").as_double(),
        pmf_max_window_size_m	= (float) this->get_parameter("pmf_max_window_size_cm").as_double() * 1e-2f,
        pmf_cell_size_m			= (float) this->get_parameter("pmf_cell_size_cm").as_double() * 1e-2f,
        pmf_init_distance_m		= (float) this->get_parameter("pmf_init_distance_cm").as_double() * 1e-2f,
        pmf_max_distance_m		= (float) this->get_parameter("pmf_max_distance_cm").as_double() * 1e-2f,
        pmf_slope				= (float) this->get_parameter("pmf_slope").as_double()
    ;
#else
    float
        voxel_size_m			= (float) this->_config.voxel_size_cm * 1e-2f,
        max_z_thresh_m			= (float) this->_config.max_z_thresh_cm * 1e-2f,
        min_z_thresh_m			= (float) this->_config.min_z_thresh_cm * 1e-2f,
        max_pmf_range_m			= (float) this->_config.pmf_max_range_cm * 1e-2f,
        pmf_window_base			= (float) this->_config.pmf_window_base,
        pmf_max_window_size_m	= (float) this->_config.pmf_max_window_size_cm * 1e-2f,
        pmf_cell_size_m			= (float) this->_config.pmf_cell_size_cm * 1e-2f,
        pmf_init_distance_m		= (float) this->_config.pmf_init_distance_cm * 1e-2f,
        pmf_max_distance_m		= (float) this->_config.pmf_max_distance_cm * 1e-2f,
        pmf_slope				= (float) this->_config.pmf_slope;
    ;
#endif


    /** WARNING: The below algorithm is tricky since an empty, "null" selection
    * accomplishes different things depending on the specific method call! 
    * The current control-flow is specifically engineered to avoid misinterpretations,
    * so modifying this code at all may result in unintended and hard to find bugs! */

    // voxelize points
    voxel_filter(
        cloud, DEFAULT_NO_SELECTION, voxel_cloud,	// <-- default selection = use all points
        voxel_size_m, voxel_size_m, voxel_size_m
    );

    // filter points under "high cut" thresh
    carteZ_filter(
        voxel_cloud, DEFAULT_NO_SELECTION, z_high_filtered,		// <-- default selection = use all points
        -std::numeric_limits<float>::infinity(),
        max_z_thresh_m
    );
    if(z_high_filtered.size() <= 0) return;		// no points to process

    // further filter points below "low cut" thresh
    carteZ_filter(
        voxel_cloud, z_high_filtered, z_low_subset_filtered,
        -std::numeric_limits<float>::infinity(),
        min_z_thresh_m
    );

    const bool
        only_z_mid = z_low_subset_filtered.size() <= 0,		// no points at "groud level" --> skip advanced processing and just add these to the grid
        has_z_mid = z_low_subset_filtered.size() != z_high_filtered.size();		// are there any points at all in the "wall range"? --> if not, don't attempt to negate the selection, or add points to the grid
    if(!only_z_mid) {	// need points in order to continue (otherwise this gets misinterpretted as "use all points")

        if(has_z_mid) {
            // get the points inbetween high and low thresholds --> treated as wall obstacles
            pc_negate_selection(
                z_high_filtered,
                z_low_subset_filtered,
                z_mid_filtered_obstacles
            );
        }

        // filter close enough points for PMF
        pc_filter_distance(
            voxel_cloud.points,
            z_low_subset_filtered,
            pre_pmf_range_filtered,		// NOTE: this will contain all indices even if the entire cloud is selected (no default empty select)
            0.f, max_pmf_range_m,
            origin
        );
        if(pre_pmf_range_filtered.size() > 0) {		// need points in order to continue (otherwise this gets reinterpretted as "use all points")

            // apply pmf to selected points
            progressive_morph_filter(
                voxel_cloud, pre_pmf_range_filtered, pmf_filtered_ground,	// see the note above -- an empty output selection means no ground points
                pmf_window_base,
                pmf_max_window_size_m,
                pmf_cell_size_m,
                pmf_init_distance_m,
                pmf_max_distance_m,
                pmf_slope,
                false
            );
            
            // obstacles = (base - ground)
            if(pmf_filtered_ground.size() == 0) {	// the case where all subset points are obstacles...
                pmf_filtered_obstacles.clear();
                std::swap(pre_pmf_range_filtered, pmf_filtered_obstacles);	// setup the "accumulator" selection as containing the selection (null base) to trigger "case 2" of incrementRatio best efficiency
            } else {
                pc_negate_selection(	// normal set difference computation
                    pre_pmf_range_filtered,
                    pmf_filtered_ground,
                    pmf_filtered_obstacles
                );
            }	// in the case that all prefiltered points are ground, incrementRatio opperates correctly by default

            this->accumulator.incrementRatio<pcl::PointXYZ, std::ratio<3, 2>, 50, 100>(	// insert PMF obstacles
                voxel_cloud,
                pre_pmf_range_filtered,		// base
                pmf_filtered_obstacles		// subset (accumulator)
            );

        } else {}	// no points to process for PMF...

    } else {}	// no low (groud) points to process
    if(has_z_mid) {		// only attempt if z_mid_filtered_obstacles is not empty to prevent a misinterprettation by incrementRatio (this would increment all the cells in the grid)
        this->accumulator.incrementRatio<pcl::PointXYZ, std::ratio<3, 2>, 50, 100>(	// insert z-thresh obstacles (increment both halves of ratio)
            voxel_cloud,
            DEFAULT_NO_SELECTION,		// trigger "case 2" of incrementRatio (recently changed)
            (only_z_mid ? z_high_filtered : z_mid_filtered_obstacles)	// accumulator --> increments both for this selection since base is null
        );
    }
}

void PerceptionNode::process_and_export(const pcl::PointCloud<pcl::PointXYZ>& cloud, const Eigen::Vector3f& origin)
{
    this->process(cloud, origin);

    nav_msgs::msg::OccupancyGrid out_grid;

    const size_t _area = static_cast<size_t>(this->accumulator.area());
    const Eigen::Vector2f& _origin = this->accumulator.origin();
    const Eigen::Vector2i& _grid_size = this->accumulator.size();

    out_grid.info.resolution = this->accumulator.cellRes();
    out_grid.info.origin.position.x = _origin.x();
    out_grid.info.origin.position.y = _origin.y();
    out_grid.info.width = _grid_size.x();
    out_grid.info.height = _grid_size.y();

    out_grid.data.resize(_area);
    memcpy(out_grid.data.data(), this->accumulator.buffData(), _area);

    out_grid.header.frame_id = this->_config.map_frame;
    out_grid.header.stamp = this->get_clock()->now();

    this->grid_pub->publish(out_grid);

    // RCLCPP_INFO(this->get_logger(),
    //     "Grid Updated!"
    //     "\n\tOrigin: (%f, %f)"
    //     "\n\tDims: (%d, %d)"
    //     "\n\tSize: (%f, %f)",
    //     _origin.x(),
    //     _origin.y(),
    //     _grid_size.x(),
    //     _grid_size.y(),
    //     _grid_size.x() * this->accumulator.cellRes(),
    //     _grid_size.y() * this->accumulator.cellRes()
    // );
}
