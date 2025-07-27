/**
 * @file raycast_mapping.h
 * @author Toshiki Nakamura
 * @brief C++ implementation of Ray Casting Update Algorithm for 2D Mapping
 * @copyright Copyright (c) 2024
 */

#ifndef RAYCAST_MAPPING_RAYCAST_MAPPING_HPP
#define RAYCAST_MAPPING_RAYCAST_MAPPING_HPP

#include <chrono>
#include <opencv2/opencv.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <memory> //shared_ptr用 
#include <optional>
#include <vector>

// 時間をs,msで表せる
using namespace std::chrono_literals;


/**
 * @struct PrecastDB
 * @brief Struct for precast database
 * @param info Map metadata

 */
struct PrecastDB
{
  nav_msgs::msg::MapMetaData info;
};

/**
 * @class LocalMapCreator
 * @brief Class for creating local map
 */


class LocalMapCreator : public rclcpp::Node
{
  public:
  /**
   * @brief Construct a new Local Map Creator object
   */
  LocalMapCreator();

  private:
  /**
   * @brief Timer callback function
   *
   */
  void timer_callback();

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
  nav_msgs::msg::OccupancyGrid init_map(const nav_msgs::msg::MapMetaData &map_info);

  /**
   * @brief Convert x, y to grid index
   *
   * @param x X
   * @param y Y
   * @param map_info Map information
   * @return int Grid index
   */
  int xy_to_grid_index(const float x, const float y, const nav_msgs::msg::MapMetaData &map_info);


  /**
   * @brief Make grid map
   *
   * @param free_color cost-free color
   * @param trimming_color color for trimming
   * @param image_path Path to images
   * @param precast_db Precast database
   * @return nav_msgs::msg::OccupancyGrid Occupancy grid message
   */
  nav_msgs::msg::OccupancyGrid make_images(cv::Vec3b free_color, cv::Vec3b trimming_color, std::string image_path, const PrecastDB &precast_db);  // 必要に応じてパスを変更

  std::string frame_id_;
  float map_reso_;  // [m/cell]
  float map_size_;
  float yaw_reso_;  // [rad/cell]
  PrecastDB precast_db_;


  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

};

#endif  // RAYCAST_MAPPING_RAYCAST_MAPPING_HPP
