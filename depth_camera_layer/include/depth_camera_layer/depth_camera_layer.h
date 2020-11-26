#ifndef DEPTH_CAMERA_LAYER_H
#define DEPTH_CAMERA_LAYER_H

#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/layered_costmap.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/message_filter.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <message_filters/subscriber.h>
#include <depth_camera_layer/DepthCameraLayerConfig.h>
#include <dynamic_reconfigure/server.h>
#include <list>
#include <string>
#include <vector>
#include <utility>
#include "robot_msg/SlamStatus.h"
#include "boost/thread/mutex.hpp"

namespace depth_camera_layer
{

struct MarkerPoints
{
  ros::Time marker_time;
  std::vector< std::pair<double,double> > points;
};

class DepthCameraLayer : public costmap_2d::CostmapLayer
{
public:

  DepthCameraLayer();
  virtual ~DepthCameraLayer();

  virtual void onInitialize();
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw,
                            double* min_x, double* min_y, double* max_x, double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
  virtual void reset();
  virtual void deactivate();
  virtual void activate();

private:

  void pointCloud2Callback(const sensor_msgs::PointCloud2ConstPtr& msg);
  void slamStatusCallBack(const robot_msg::SlamStatus::ConstPtr& msg);
  void clearHistoryObs();
  bool getSensor2GlobalFrameTf(std::string sensor_frame, geometry_msgs::TransformStamped& sensor_frame_to_global_frame);
  void updateWithMax(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

  //dynamatic params
  double obstacle_keep_time_;
  double min_obstacle_height_;
  double max_obstacle_height_;
  double min_obstacle_range_;
  double max_obstacle_range_;
  double expected_update_rate_;

  //static params
  std::string topic_name_ = "";
  std::string sensor_frame_ = "";

  ros::NodeHandle* nh_;
  bool is_build_;
  bool rec_flag_;
  bool rolling_window_;
  double last_receive_msg_time_sec_;
  double receive_msg_interval_time_sec_;
  std::string global_frame_;

  ros::Subscriber sub_slam_mode_;
  message_filters::Subscriber<sensor_msgs::PointCloud2>* sub_ptr_ = nullptr;
  tf2_ros::MessageFilter<sensor_msgs::PointCloud2>* notifiers_ptr_ = nullptr;

  boost::mutex clear_mutex_;
  boost::mutex set_params_mutex_;
  boost::mutex receive_message_mutex_;
  boost::mutex do_message_mutex_;
  sensor_msgs::PointCloud2 rec_point_cloud_;
  std::vector< MarkerPoints > marker_points_buffer_;

  dynamic_reconfigure::Server<depth_camera_layer::DepthCameraLayerConfig> *dsrv_;
  void reconfigureCB(depth_camera_layer::DepthCameraLayerConfig &config, uint32_t level);
};
}
#endif
