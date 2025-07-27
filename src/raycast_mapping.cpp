/**
 * @file raycast_mapping.cpp
 * @author Toshiki Nakamura
 * @brief C++ implementation of Ray Casting Update Algorithm for 2D Mapping
 * @copyright Copyright (c) 2024
 */

#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include "raycast_mapping.hpp"


// const rclcpp::NodeOptions & options を使う方法も今後検討する
LocalMapCreator::LocalMapCreator()
:Node("raycast_mapping")
{

  this->declare_parameter("frame_id", "base_footprint");
  this->declare_parameter("map_reso", 0.05);
  this->declare_parameter("map_size", 30.0);

  this->get_parameter("frame_id", frame_id_);
  this->get_parameter("map_reso", map_reso_);
  this->get_parameter("map_size", map_size_);

  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/local_map",rclcpp::QoS(1).reliable());
  timer_ = this->create_wall_timer(0.5s, std::bind(&LocalMapCreator::timer_callback, this));

  precast_db_ = create_precast_db(map_reso_, map_size_);


  //追記

  //カラーマップの定義（Cityscapesのようなデータセットの場合）
  // Cscolor = np.array([
  //     [128, 64, 128],  // Road
  //     [244, 35, 232],  // Sidewalk
  //     [70, 70, 70],    // Building
  //     [102, 102, 156], // Wall
  //     [190, 153, 153], // Fence
  //     [153, 153, 153], // Pole
  //     [250, 170, 30],  // Traffic light
  //     [220, 220, 0],   // Traffic sign
  //     [107, 142, 35],  // Vegetation
  //     [152, 251, 152], // Terrain
  //     [70, 130, 180],  // Sky
  //     [220, 20, 60],   // Person
  //     [255, 0, 0],     // Rider
  //     [0, 0, 142],     // Car
  //     [0, 0, 70],      // Truck
  //     [0, 60, 100],    // Bus
  //     [0, 80, 100],    // Train
  //     [0, 0, 230],     // Motorcycle
  //     [119, 11, 32],   // Bicycle
  // ],dtype=float)

}



void LocalMapCreator::timer_callback()
{
  cv::Vec3b trimming_color(0,0,0);  // trimming range
  // cv::Vec3b free_color(127, 63, 127);  // Road
  cv::Vec3b free_color(254, 127, 0);  // floor

  // std::string image_path = "/home/user/ws/src/bev_meiji_outdoor/bev_1750893090.087051_seg_only.png";  // 必要に応じてパスを変更
  // std::string image_path = "/home/user/ws/src/bev_meiji_outdoor/bev_1750893196.839462_seg_only.png";  // 必要に応じてパスを変更
  // std::string image_path = "/home/user/ws/src/bev_meiji_indoor_8class/bev_1750316090.940470.png";
  std::string image_path = "/home/user/ws/src/bev_meiji_indoor_8class/bev_1750316451.496942.png";
  // This function is intentionally left empty.
  // It can be used for periodic tasks if needed.
  nav_msgs::msg::OccupancyGrid local_map = make_images(free_color,trimming_color,image_path, precast_db_);
  local_map.header.frame_id = frame_id_;
  local_map.header.stamp = this->get_clock()->now();
  map_pub_->publish(local_map);
}


//将来的には鳥瞰図のときのマップ設定をこっちにも反映させたい
PrecastDB LocalMapCreator::create_precast_db(const float map_reso, const float map_size)
{
  PrecastDB precast_db;
  precast_db.info.resolution = map_reso;
  precast_db.info.width = static_cast<int>(round(map_size / map_reso));
  precast_db.info.height = static_cast<int>(round(map_size / map_reso));
  precast_db.info.origin.position.x = -map_size / 2.0;
  precast_db.info.origin.position.y = -map_size / 2.0;

  // ROS_WARN_STREAM("Precast DB has been created");
  return precast_db;
}

nav_msgs::msg::OccupancyGrid LocalMapCreator::init_map(const nav_msgs::msg::MapMetaData &map_info)
{
  nav_msgs::msg::OccupancyGrid local_map;
  local_map.info = map_info;
  local_map.data.reserve(map_info.width * map_info.height);
  for (int i = 0; i < map_info.width * map_info.height; i++)
    local_map.data.push_back(0);  // free

  return local_map;
}

//参考になるかも
int LocalMapCreator::xy_to_grid_index(const float x, const float y, const nav_msgs::msg::MapMetaData &map_info)
{
  const int index_x = static_cast<int>(floor((x - map_info.origin.position.x) / map_info.resolution));
  const int index_y = static_cast<int>(floor((y - map_info.origin.position.y) / map_info.resolution));
  return index_x + (index_y * map_info.width);
}



nav_msgs::msg::OccupancyGrid LocalMapCreator::make_images(cv::Vec3b free_color, cv::Vec3b trimming_color, std::string image_path, const PrecastDB &precast_db)  // 必要に応じてパスを変更
{
 
  nav_msgs::msg::OccupancyGrid local_map = init_map(precast_db.info);
  // 画像の読み込み
  cv::Mat image = cv::imread(image_path, cv::IMREAD_COLOR);
  if (image.empty()) {
      std::cerr << "画像を読み込めませんでした: " << image_path << std::endl;
  }

   // 色の変更処理（例：赤っぽいピクセル → 青に変更）
  for (int y = 0; y < image.rows; ++y) {
    for (int x = 0; x < image.cols; ++x) {
      cv::Vec3b& pixel = image.at<cv::Vec3b>(y, x);
      if (pixel == trimming_color) {  // trimming_colorと一致するピクセルを検出
        pixel = free_color;  // free_colorに変更
      }
      if (pixel != free_color) {  // free_colorと一致するピクセルを検出
        int x_img = image.cols - 1 - x;  // ← 左右反転
        int y_img = y;
        int x_rot = y_img;
        int y_rot = precast_db.info.width - 1 - x_img;
        int index = x_rot + y_rot * precast_db.info.width;
        local_map.data[index] = 100;  // occupied
        }
    }
  }

  // 画像の表示
  cv::imshow("Modified Image", image);
  cv::waitKey(10000); // キー入力待ち


  return local_map;
}
