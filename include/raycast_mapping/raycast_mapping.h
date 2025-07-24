/**
 * @file raycast_mapping.h
 * @author Toshiki Nakamura
 * @brief C++ implementation of Ray Casting Update Algorithm for 2D Mapping
 * @copyright Copyright (c) 2024
 */

#ifndef RAYCAST_MAPPING_RAYCAST_MAPPING_H
#define RAYCAST_MAPPING_RAYCAST_MAPPING_H

#include <nav_msgs/OccupancyGrid.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <utility>
#include <vector>

/**
 * @struct PrecastData
 * @brief Struct for precast data
 */
struct PrecastData
{
  float dist;
  int grid_index;
};

/**
 * @struct PrecastDB
 * @brief Struct for precast database
 */
struct PrecastDB
{
  nav_msgs::MapMetaData info;
  float yaw_reso;
  std::vector<std::vector<PrecastData>> bins;
};

/**
 * @class LocalMapCreator
 * @brief Class for creating local map
 */
class LocalMapCreator
{
public:
  /**
   * @brief Construct a new Local Map Creator object
   */
  LocalMapCreator(void);

private:
  /**
   * @brief Timer callback function
   *
   */
  void timer_callback(const ros::TimerEvent &);

  /**
   * @brief Create precast database
   *
   * @param map_reso Map resolution
   * @param map_size Map size
   * @return PrecastDB Precast database
   */
  PrecastDB create_precast_db(const float map_reso, const float map_size);

  /**
   * @brief Initialize map
   *
   * @param map_info Map information
   * @return nav_msgs::OccupancyGrid Initialized map
   */
  nav_msgs::OccupancyGrid init_map(const nav_msgs::MapMetaData &map_info);

  /**
   * @brief Convert x, y to grid index
   *
   * @param x X
   * @param y Y
   * @param map_info Map information
   * @return int Grid index
   */
  int xy_to_grid_index(const float x, const float y, const nav_msgs::MapMetaData &map_info);


  /**
   * @brief Make images for visualization
   *
   * @param free_color cost-free color
   * @param image_path Path to images
   */
  nav_msgs::OccupancyGrid make_images(cv::Vec3b free_color, cv::Vec3b trimming_color, std::string image_path, const PrecastDB &precast_db);  // 必要に応じてパスを変更

  std::string frame_id_;
  float map_reso_;  // [m/cell]
  float map_size_;
  float yaw_reso_;  // [rad/cell]
  PrecastDB precast_db_;

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher map_pub_;
  ros::Subscriber cloud_sub_;
  ros::Timer timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

#endif  // RAYCAST_MAPPING_RAYCAST_MAPPING_H
