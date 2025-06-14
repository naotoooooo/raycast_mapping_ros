/**
 * @file raycast_mapping.cpp
 * @author Toshiki Nakamura
 * @brief C++ implementation of Ray Casting Update Algorithm for 2D Mapping
 * @copyright Copyright (c) 2024
 */

#include <string>
#include <utility>
#include <vector>
#include <opencv2/opencv.hpp>


#include "raycast_mapping/raycast_mapping.h"

LocalMapCreator::LocalMapCreator(void) : private_nh_("~"), tf_listener_(tf_buffer_)
{
  private_nh_.param<std::string>("frame_id", frame_id_, "base_footprint");
  private_nh_.param<float>("map_reso", map_reso_, 0.05);
  private_nh_.param<float>("map_size", map_size_, 10.0);
  private_nh_.param<float>("yaw_reso", yaw_reso_, 0.087);

  map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/local_map", 1);
  // cloud_sub_ = nh_.subscribe("/cloud", 1, &LocalMapCreator::cloud_callback, this);
  // タイマーの設定（1秒ごとにコールバックを呼び出す）
  timer_ = nh_.createTimer(ros::Duration(1.0), &LocalMapCreator::timer_callback, this);

  ROS_INFO_STREAM(ros::this_node::getName() << " node has started..");
  ROS_INFO_STREAM("frame_id: " << frame_id_);
  ROS_INFO_STREAM("map_reso: " << map_reso_);
  ROS_INFO_STREAM("map_size: " << map_size_);
  ROS_INFO_STREAM("yaw_reso: " << yaw_reso_);

  precast_db_ = create_precast_db(map_reso_, map_size_, yaw_reso_);


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




  // cv::Vec3b free_color(127, 63, 127);  // Road
  
  // std::string image_path = "/home/user/ws/src/birds_eye_output_seg102.png";  // 必要に応じてパスを変更
  // make_images(free_color,image_path);


}



void LocalMapCreator::timer_callback(const ros::TimerEvent &)
{
  cv::Vec3b free_color(127, 63, 127);  // Road
  
  std::string image_path = "/home/user/ws/src/birds_eye_output_seg102.png";  // 必要に応じてパスを変更
  // This function is intentionally left empty.
  // It can be used for periodic tasks if needed.
  nav_msgs::OccupancyGrid local_map = make_images(free_color,image_path, precast_db_);
  local_map.header.frame_id = frame_id_;
  local_map.header.stamp = ros::Time::now();
  map_pub_.publish(local_map);
}



PrecastDB LocalMapCreator::create_precast_db(const float map_reso, const float map_size, const float yaw_reso)
{
  PrecastDB precast_db;
  precast_db.info.resolution = map_reso;
  precast_db.info.width = static_cast<int>(round(map_size / map_reso));
  precast_db.info.height = static_cast<int>(round(map_size / map_reso));
  precast_db.info.origin.position.x = -map_size / 2.0;
  precast_db.info.origin.position.y = -map_size / 2.0;
  precast_db.yaw_reso = yaw_reso;
  precast_db.bins.resize(2.0 * M_PI / yaw_reso + 1);

  for (int i = 0; i < precast_db.info.width * precast_db.info.height; i++)
  {
    const std::pair<float, float> dist_and_angle = calc_dist_and_angle(i, precast_db.info);
    const int angle_id = calc_angle_id(dist_and_angle.second, yaw_reso);

    PrecastData data;
    data.dist = dist_and_angle.first;
    data.grid_index = i;
    try
    {
      precast_db.bins[angle_id].push_back(data);
    }
    catch (const std::exception &e)
    {
      ROS_ERROR_STREAM(e.what());
      exit(1);
    }
  }

  ROS_WARN_STREAM("Precast DB has been created");
  return precast_db;
}

std::pair<float, float> LocalMapCreator::calc_dist_and_angle(const int index, const nav_msgs::MapMetaData &map_info)
{
  const int index_x = index % map_info.width;
  const int index_y = static_cast<int>(index / map_info.width);
  const float x = index_x * map_info.resolution + map_info.origin.position.x + map_info.resolution / 2.0;
  const float y = index_y * map_info.resolution + map_info.origin.position.y + map_info.resolution / 2.0;
  return std::make_pair(hypot(x, y), atan2(y, x));
}

int LocalMapCreator::calc_angle_id(float angle, const float yaw_reso)
{
  angle = angle < 0 ? angle + 2.0 * M_PI : angle;
  return static_cast<int>(floor(angle / yaw_reso));
}


nav_msgs::OccupancyGrid LocalMapCreator::init_map(const nav_msgs::MapMetaData &map_info)
{
  nav_msgs::OccupancyGrid local_map;
  local_map.info = map_info;
  local_map.data.reserve(map_info.width * map_info.height);
  for (int i = 0; i < map_info.width * map_info.height; i++)
    local_map.data.push_back(0);  // free

  return local_map;
}


//参考になるかも
int LocalMapCreator::xy_to_grid_index(const float x, const float y, const nav_msgs::MapMetaData &map_info)
{
  const int index_x = static_cast<int>(floor((x - map_info.origin.position.x) / map_info.resolution));
  const int index_y = static_cast<int>(floor((y - map_info.origin.position.y) / map_info.resolution));
  return index_x + (index_y * map_info.width);
}



nav_msgs::OccupancyGrid LocalMapCreator::make_images(cv::Vec3b free_color,std::string image_path, const PrecastDB &precast_db)  // 必要に応じてパスを変更
{
  // // 固定された画像ファイルパス
  // std::string image_path = "/home/user/ws/src/birds_eye_output_seg102.png";  // 必要に応じてパスを変更
  std::cout << "hello" << std::endl;
  nav_msgs::OccupancyGrid local_map = init_map(precast_db.info);
  // 画像の読み込み
  cv::Mat image = cv::imread(image_path, cv::IMREAD_COLOR);
  if (image.empty()) {
      std::cerr << "画像を読み込めませんでした: " << image_path << std::endl;
  }
  // 色の変更処理（例：赤っぽいピクセル → 青に変更）
  // for (int y = 0; y < image.rows; ++y) {
  //     for (int x = 0; x < image.cols; ++x) {
  //         cv::Vec3b& pixel = image.at<cv::Vec3b>(y, x);
  //         std::cout << "Pixel at (" << x << ", " << y << "): "
  //                   << "B: " << static_cast<int>(pixel[0]) << ", "
  //                   << "G: " << static_cast<int>(pixel[1]) << ", "
  //                   << "R: " << static_cast<int>(pixel[2]) << std::endl;
  //         if (pixel != free_color) {  // free_colorと一致するピクセルを検出
  //             pixel[0] = 0; // Blue
  //             pixel[1] = 0;   // Green
  //             pixel[2] = 0;   // Red
  //         }
  //     }
  // }


   // 色の変更処理（例：赤っぽいピクセル → 青に変更）
  for (int y = 0; y < image.rows; ++y) {
      for (int x = 0; x < image.cols; ++x) {
          cv::Vec3b& pixel = image.at<cv::Vec3b>(y, x);
          std::cout << "Pixel at (" << x << ", " << y << "): "
                    << "B: " << static_cast<int>(pixel[0]) << ", "
                    << "G: " << static_cast<int>(pixel[1]) << ", "
                    << "R: " << static_cast<int>(pixel[2]) << std::endl;
          if (pixel != free_color) {  // free_colorと一致するピクセルを検出
              local_map.data[x+y*200] = 100;  // occupied
          }
      }
  }

  // 画像の表示
  cv::imshow("Modified Image", image);
  cv::waitKey(10000); // キー入力待ち


  return local_map;
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "raycast_mapping");
  LocalMapCreator local_map_creator;
  ros::spin();

  return 0;
}
