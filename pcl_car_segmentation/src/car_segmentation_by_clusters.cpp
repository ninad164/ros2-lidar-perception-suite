#include <map>
#include <filesystem>
#include <chrono>
#include <memory>
#include <string>
#include <sys/types.h>
#include <pwd.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/string.hpp"
#include "pcl_conversions/pcl_conversions.h"

#include "pcl/filters/voxel_grid.h"
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/octree/octree_pointcloud.h>
#include <pcl/octree/octree.h>
#include <pcl/point_cloud.h>

#include <pcl/features/normal_3d.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>

#include "visualization_msgs/msg/marker_array.hpp"

#include <Eigen/Dense> 
#include <vector>

using namespace std::chrono_literals;
typedef pcl::PointXYZ PointT;
struct Track {
    int id;
    Eigen::Vector3f centroid;
    int age;
};
int car_id = 0;
struct passwd *pw = getpwuid(getuid());
const char *home_dir = pw -> pw_dir;


class CarSegmentation: public rclcpp ::Node{
    public:
        std::string folder_path = std::string(home_dir) + "/ros2_ws/src/ros2_pcl_segmentation/pcl_car_segmentation/car_dataset_clouds/";

        CarSegmentation():  Node("car_segmentation"){

            cluster_voxel_leaf_size_ = this->declare_parameter<double>("cluster_voxel_leaf_size", 0.2);
            cluster_tolerance_ = this->declare_parameter<double>("cluster_tolerance", 0.6);

            min_cluster_size_ = this->declare_parameter<int>("min_cluster_size", 100);
            max_cluster_size_ = this->declare_parameter<int>("max_cluster_size", 700);

            min_length_ = this->declare_parameter<double>("min_length", 1.0);
            max_length_ = this->declare_parameter<double>("max_length", 5.0);
            min_width_ = this->declare_parameter<double>("min_width", 1.0);
            max_width_ = this->declare_parameter<double>("max_width", 5.0);
            min_height_ = this->declare_parameter<double>("min_height", 1.0);
            max_height_ = this->declare_parameter<double>("max_height", 2.0);

            input_topic_ = this->declare_parameter<std::string>(
                "input_topic",
                "/pcl_car_segmentation/filtered_cloud"
            );

            output_topic_ = this->declare_parameter<std::string>(
                "output_topic",
                "/pcl_car_segmentation/car_segmentation"
            );
            
            if (!std::filesystem::exists(folder_path)) {
                std::filesystem::create_directories(folder_path);
                std::cerr << "Directory created: " << folder_path << std::endl;
            }

            filtered_cloud_subscriber = this -> create_subscription<sensor_msgs::msg::PointCloud2>(
                input_topic_, 10, std::bind(&CarSegmentation::point_cloud_callback, this, std::placeholders::_1)
            );

            car_segmentation_publisher = this -> create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, 10);
            bounding_box_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>(
                "/pcl_car_segmentation/bounding_boxes",
                10
            );
        }

    private:
        double cluster_voxel_leaf_size_;
        double cluster_tolerance_;

        int min_cluster_size_;
        int max_cluster_size_;

        double min_length_;
        double max_length_;
        double min_width_;
        double max_width_;
        double min_height_;
        double max_height_;

        std::string input_topic_;
        std::string output_topic_;
    
        std::map<int, Track> active_tracks_;
        int next_track_id_ = 0;
        float tracking_distance_threshold_ = 2.5;
        
        int assign_track_id(const Eigen::Vector3f& centroid)
        {
            int matched_id = -1;
            float min_distance = tracking_distance_threshold_;

            for (auto& [id, track] : active_tracks_) {
                float distance = (track.centroid - centroid).norm();

                if (distance < min_distance) {
                    min_distance = distance;
                    matched_id = id;
                }
            }

            if (matched_id == -1) {
                matched_id = next_track_id_++;

                Track new_track;
                new_track.id = matched_id;
                new_track.centroid = centroid;
                new_track.age = 0;

                active_tracks_[matched_id] = new_track;
            } else {
                active_tracks_[matched_id].centroid = centroid;
                active_tracks_[matched_id].age++;
            }

            return matched_id;
        }
        
        void save_cluster(pcl::PointCloud<PointT> :: Ptr cluster, std::string file_name){
        file_name = folder_path + file_name;
        pcl::io::savePCDFileASCII(file_name, *cluster);
        std::cerr << "Saved " << cluster -> points.size() << " data points to " << file_name << std::endl;
    }

    void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr input_cloud){

        auto start_time = std::chrono::steady_clock::now();
        int accepted_clusters = 0;
        pcl::PointCloud<PointT> :: Ptr pcl_cloud (new pcl:: PointCloud<PointT>) ;
        pcl::fromROSMsg(*input_cloud, *pcl_cloud);

        pcl::PointCloud<PointT> :: Ptr single_segmented_cluster (new pcl:: PointCloud<PointT>) ;
        pcl::PointCloud<PointT> :: Ptr all_clusters (new pcl:: PointCloud<PointT>) ;
        std::vector<pcl::PointIndices> cluster_indices;
        visualization_msgs::msg::MarkerArray marker_array;
        int marker_id = 0;
        pcl::EuclideanClusterExtraction<PointT> ecludian_cluster_extractor;
        pcl::search::KdTree<PointT>::Ptr tree (new pcl:: search ::KdTree<PointT>());

        // Apply Voxel Grid Filter
        pcl::VoxelGrid<PointT> voxel_grid_filter;
        voxel_grid_filter.setInputCloud(pcl_cloud);
        voxel_grid_filter.setLeafSize(cluster_voxel_leaf_size_, cluster_voxel_leaf_size_, cluster_voxel_leaf_size_);
        voxel_grid_filter.filter(*pcl_cloud);

        tree -> setInputCloud(pcl_cloud);

        ecludian_cluster_extractor.setClusterTolerance(cluster_tolerance_);
        ecludian_cluster_extractor.setMinClusterSize(min_cluster_size_);
        ecludian_cluster_extractor.setMaxClusterSize(max_cluster_size_);
        ecludian_cluster_extractor.setSearchMethod(tree);
        ecludian_cluster_extractor.setInputCloud(pcl_cloud);
        ecludian_cluster_extractor.extract(cluster_indices);

        size_t min_cloud_threshold = min_cluster_size_;
        size_t max_cloud_threshold = max_cluster_size_;

        for (size_t i = 0; i < cluster_indices.size(); i++){
            if (cluster_indices[i].indices.size() > min_cloud_threshold + 20 && cluster_indices[i].indices.size() < max_cloud_threshold - 20){
                pcl::PointCloud<PointT> :: Ptr reasonable_cluster (new pcl:: PointCloud<PointT>) ;
                pcl::ExtractIndices<PointT> extract_indices;
                pcl::IndicesPtr indices (new std::vector<int>(cluster_indices[i].indices.begin(), cluster_indices[i].indices.end()));

                extract_indices.setInputCloud(pcl_cloud);
                extract_indices.setIndices(indices);
                extract_indices.setNegative(false);
                extract_indices.filter(*reasonable_cluster);

                Eigen::Vector4f min_pt, max_pt;
                pcl::getMinMax3D<PointT>(*reasonable_cluster, min_pt, max_pt);

                float length = max_pt[0] - min_pt[0];
                float width = max_pt[1] - min_pt[1];
                float height = max_pt[2] - min_pt[2];

                Eigen::Vector3f centroid(
                    (min_pt[0] + max_pt[0]) / 2.0f,
                    (min_pt[1] + max_pt[1]) / 2.0f,
                    (min_pt[2] + max_pt[2]) / 2.0f
                );

                int track_id = assign_track_id(centroid);

                if (length >= min_length_ && length <= max_length_ &&
                width >= min_width_ && width <= max_width_ &&
                height >= min_height_ && height <= max_height_) {
                        std::string file_name = "car_cluster_" + std::to_string(cluster_indices[i].indices.size()) + "_" + std::to_string(length) + "_" + std::to_string(width) + "_" + std::to_string(height) + ".pcd";
                        //save_cluster(reasonable_cluster, file_name);
                        visualization_msgs::msg::Marker marker;

                        marker.header = input_cloud->header;
                        marker.ns = "car_bounding_boxes";
                        marker.id = marker_id++;
                        marker.type = visualization_msgs::msg::Marker::CUBE;
                        marker.action = visualization_msgs::msg::Marker::ADD;

                        marker.pose.position.x = (min_pt[0] + max_pt[0]) / 2.0;
                        marker.pose.position.y = (min_pt[1] + max_pt[1]) / 2.0;
                        marker.pose.position.z = (min_pt[2] + max_pt[2]) / 2.0;

                        marker.pose.orientation.w = 1.0;

                        marker.scale.x = length;
                        marker.scale.y = width;
                        marker.scale.z = height;

                        marker.color.r = 1.0;
                        marker.color.g = 0.0;
                        marker.color.b = 0.0;
                        marker.color.a = 0.5;

                        marker.lifetime = rclcpp::Duration::from_seconds(0.1);

                        marker_array.markers.push_back(marker);

                        visualization_msgs::msg::Marker text_marker;

                        text_marker.header = input_cloud->header;
                        text_marker.ns = "car_track_ids";
                        text_marker.id = marker_id++;
                        text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
                        text_marker.action = visualization_msgs::msg::Marker::ADD;

                        text_marker.pose.position.x = centroid.x();
                        text_marker.pose.position.y = centroid.y();
                        text_marker.pose.position.z = centroid.z() + 1.5;

                        text_marker.pose.orientation.w = 1.0;

                        text_marker.scale.z = 0.8;

                        text_marker.color.r = 1.0;
                        text_marker.color.g = 1.0;
                        text_marker.color.b = 1.0;
                        text_marker.color.a = 1.0;

                        text_marker.text = "ID: " + std::to_string(track_id);

                        text_marker.lifetime = rclcpp::Duration::from_seconds(0.1);

                        marker_array.markers.push_back(text_marker);
                        
                        *all_clusters += *reasonable_cluster;
                        accepted_clusters++;
                    }

            }
        }

        sensor_msgs::msg::PointCloud2::SharedPtr car_segmentation_msg (new sensor_msgs::msg::PointCloud2);
        pcl::toROSMsg(*all_clusters, *car_segmentation_msg);
        car_segmentation_msg -> header = input_cloud -> header;
        car_segmentation_publisher -> publish(*car_segmentation_msg);
        bounding_box_publisher->publish(marker_array);

        auto end_time = std::chrono::steady_clock::now();

        auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            end_time - start_time
        ).count();

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "Input points: %zu | Accepted clusters: %d | Output points: %zu | Processing time: %ld ms",
            pcl_cloud->size(),
            accepted_clusters,
            all_clusters->size(),
            duration_ms
        );
    }
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr car_segmentation_publisher;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr bounding_box_publisher;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>:: SharedPtr filtered_cloud_subscriber;
};

int main(int argc, char **argv){
    
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CarSegmentation>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
