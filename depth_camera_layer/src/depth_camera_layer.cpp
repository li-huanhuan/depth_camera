#include <depth_camera_layer/depth_camera_layer.h>
#include <boost/algorithm/string.hpp>
#include <geometry_msgs/PointStamped.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <angles/angles.h>
#include <algorithm>
#include <limits>

PLUGINLIB_EXPORT_CLASS(depth_camera_layer::DepthCameraLayer, costmap_2d::Layer)

namespace depth_camera_layer
{

DepthCameraLayer::DepthCameraLayer()
{
  this->dsrv_ = nullptr;
}

DepthCameraLayer::~DepthCameraLayer()
{
  if(this->dsrv_ != nullptr)
  {
    delete this->dsrv_;
    this->dsrv_ = nullptr;
  }
  if(this->notifiers_ptr_ != nullptr)
  {
    delete notifiers_ptr_;
    notifiers_ptr_ = nullptr;
  }
  if(this->sub_ptr_ != nullptr)
  {
    delete sub_ptr_;
    sub_ptr_ = nullptr;
  }
  if(this->nh_)
  {
    delete nh_;
    nh_ = nullptr;
  }
}

void DepthCameraLayer::onInitialize()
{
  rolling_window_ = layered_costmap_->isRolling();
  global_frame_ = layered_costmap_->getGlobalFrameID();
  default_value_ = costmap_2d::NO_INFORMATION;
  matchSize();
  is_build_ = false;
  rec_flag_ = false;
  current_ = true;
  last_receive_msg_time_sec_ = ros::Time::now().toSec();

  nh_ = new ros::NodeHandle("~/" + name_);

  nh_->param<bool>("enabled", enabled_, bool());
  nh_->param("topic_name", topic_name_, std::string());
  nh_->param("sensor_frame", sensor_frame_, std::string(""));
  nh_->param("obstacle_keep_time", obstacle_keep_time_, 0.0);
  nh_->param("min_obstacle_height", min_obstacle_height_, 0.0);
  nh_->param("max_obstacle_height", max_obstacle_height_, 2.0);
  nh_->param("min_obstacle_range", min_obstacle_range_, 0.0);
  nh_->param("max_obstacle_range", max_obstacle_range_, 2.0);
  nh_->param("expected_update_rate", expected_update_rate_, 0.0);

  ros::NodeHandle g_nh;

  if(topic_name_ == "")
    ROS_ERROR("depth camera layer point cloud topic is empty.Please reset parameters:topic_name and restart.");
  else
  {
    sub_ptr_ = new message_filters::Subscriber< sensor_msgs::PointCloud2 >(g_nh, topic_name_, 1);
    notifiers_ptr_ = new tf2_ros::MessageFilter<sensor_msgs::PointCloud2>(*sub_ptr_, *tf_, global_frame_, 1, g_nh);
    notifiers_ptr_->registerCallback(boost::bind(&DepthCameraLayer::pointCloud2Callback, this,_1));
  }

  sub_slam_mode_ = g_nh.subscribe<robot_msg::SlamStatus>("/slam_status",1,&DepthCameraLayer::slamStatusCallBack,this);

  dsrv_ = new dynamic_reconfigure::Server<depth_camera_layer::DepthCameraLayerConfig>(*nh_);
  dynamic_reconfigure::Server<depth_camera_layer::DepthCameraLayerConfig>::CallbackType cb =
    boost::bind(&DepthCameraLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}

void DepthCameraLayer::reconfigureCB(depth_camera_layer::DepthCameraLayerConfig &config, uint32_t level)
{
  boost::mutex::scoped_lock lock(set_params_mutex_);
  obstacle_keep_time_= config.obstacle_keep_time;

  min_obstacle_height_ = config.min_obstacle_heigh;
  max_obstacle_height_ = config.max_obstacle_heigh;

  min_obstacle_range_ = config.min_obstacle_range;
  max_obstacle_range_ = config.max_obstacle_range;
  if(enabled_ != config.enabled)
  {
    enabled_ = config.enabled;
    if(this->is_build_)
      return;
    if(enabled_)
    {
      reset();
    }
    else
    {
      deactivate();
    }
  }
}

void DepthCameraLayer::clearHistoryObs()
{
   boost::mutex::scoped_lock lock(clear_mutex_);
   size_t buffer_size = this->marker_points_buffer_.size();
   if(buffer_size == 0)
     return;
   if(!enabled_)
   {
     for(size_t i=0;i<buffer_size;i++)
     {
       size_t points_size = this->marker_points_buffer_[i].points.size();
       for(size_t j=0;j<points_size;j++)
       {
         unsigned int mx,my;
         if(worldToMap(this->marker_points_buffer_[i].points[j].first,this->marker_points_buffer_[i].points[j].second,mx,my))
         {
            setCost(mx, my, costmap_2d::FREE_SPACE);
         }
       }
     }
     this->marker_points_buffer_.clear();
     return;
   }

   for(size_t k=0;k<buffer_size;k++)
   {
     ros::Time now = ros::Time::now();
     MarkerPoints& marker_points = this->marker_points_buffer_.front();
     if( (now.toSec() - marker_points.marker_time.toSec() ) >= this->obstacle_keep_time_)
     {
       size_t points_size = marker_points.points.size();
       for(size_t j=0;j<points_size;j++)
       {
         unsigned int mx,my;
         if(worldToMap(marker_points.points[j].first,marker_points.points[j].second,mx,my))
         {
            setCost(mx, my, costmap_2d::FREE_SPACE);
         }
       }
       this->marker_points_buffer_.erase( this->marker_points_buffer_.begin() );
     }
     else
     {
       return;
     }
   }
}

bool DepthCameraLayer::getSensor2GlobalFrameTf(std::string sensor_frame,geometry_msgs::TransformStamped& sensor_frame_to_global_frame)
{
  if(sensor_frame != "")
  {
    uint8_t count = 0;
    while(ros::ok() && count < 10)
    {
      try
      {
        sensor_frame_to_global_frame = tf_->lookupTransform(global_frame_, ros::Time(0), sensor_frame_,ros::Time(0),sensor_frame_,ros::Duration(0.1));
        break;
      }
      catch(tf2::LookupException& ex)
      {
        ROS_ERROR("No Transform available Error: %s\n", ex.what());
      }
      catch(tf2::ConnectivityException& ex)
      {
        ROS_ERROR("Connectivity Error: %s\n", ex.what());
      }
      catch(tf2::ExtrapolationException& ex)
      {
        ROS_ERROR("Connectivity Error: %s\n", ex.what());
      }
      count++;
      ros::Duration(1).sleep();
    }
    if(count >= 10)
    {
      ROS_ERROR("Failed to obtain the changing relationship between the two frame，"
                "%s to %s.Please reset parameters:sensor_frame.",global_frame_.c_str(),sensor_frame.c_str());
      return false;
    }
    return true;
  }
  else
  {
    ROS_ERROR("The sensor frame is empty.");
    return false;
  }
}

void DepthCameraLayer::updateWithMax(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_)
    return;

  unsigned char* master_array = master_grid.getCharMap();
  unsigned int span = master_grid.getSizeInCellsX();

  for (int j = min_j; j < max_j; j++)
  {
    unsigned int it = j * span + min_i;
    for (int i = min_i; i < max_i; i++)
    {
      if (costmap_[it] == costmap_2d::NO_INFORMATION)
      {
        it++;
        continue;
      }

      unsigned char old_cost = master_array[it];
      if (old_cost == costmap_2d::NO_INFORMATION || old_cost < costmap_[it])
        master_array[it] = costmap_[it];
      it++;
    }
  }
}

void DepthCameraLayer::pointCloud2Callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
  ROS_INFO_ONCE("depth layer receive point cloud");
  if(!enabled_)
    return;
  sensor_msgs::PointCloud2 cloud = *(msg);
  if(cloud.data.empty())
    return;
  if(cloud.fields.size() != 3)
  {
    return;
  }
  if(cloud.fields[0].name != "x" || cloud.fields[1].name != "y" || cloud.fields[2].name != "z")
  {
    return;
  }
  sensor_msgs::PointCloud2 global_cloud;
  if(sensor_frame_ != "")
  {
    cloud.header.frame_id = sensor_frame_;
  }
  try
  {
    tf_->transform(cloud,global_cloud,global_frame_,ros::Duration(0.5));
  }
  catch (...)
  {
    ROS_ERROR("epthCameraLayer::pointCloud2Callback err.");
    return;
  }
  global_cloud.header.stamp = cloud.header.stamp;
  receive_msg_interval_time_sec_ = ros::Time::now().toSec() - last_receive_msg_time_sec_;
  last_receive_msg_time_sec_ = ros::Time::now().toSec();
  boost::mutex::scoped_lock lock(receive_message_mutex_);
  rec_point_cloud_ = global_cloud;
  rec_flag_ = true;
}

void DepthCameraLayer::slamStatusCallBack(const robot_msg::SlamStatus::ConstPtr &msg)
{
  bool flag = false;
  if(msg->status == "building")
    flag = true;
  if(flag != is_build_)
  {
    is_build_ = flag;
    std::cout << "depth camera layer::isbuild:" << is_build_ << std::endl;
    if(is_build_)
    {
      deactivate();
    }
    else if(enabled_)
    {
      reset();
    }
  }
}

void DepthCameraLayer::updateBounds(double robot_x, double robot_y, double robot_yaw,
                                    double* min_x, double* min_y, double* max_x, double* max_y)
{
  // 清除历史障碍物
  clearHistoryObs();

  if(rolling_window_)
    updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);

  useExtraBounds(min_x, min_y, max_x, max_y);

  if (!enabled_ || !rec_flag_)
    return;

  // 通过更新该层时间间隔判断该层是否为current
  if(expected_update_rate_ > 0.0)
  {
    double set_time_interval_sec = 1.0/expected_update_rate_;
    if(receive_msg_interval_time_sec_ > set_time_interval_sec && (ros::Time::now().toSec() - last_receive_msg_time_sec_) > set_time_interval_sec)
    {
      current_ = false;
      ROS_WARN("The point cloud layer sensor update frequency(%f) is less than the set frequency(%f).",1.0/receive_msg_interval_time_sec_,set_time_interval_sec);
    }
  }
  else
  {
    current_ = true;
  }

  sensor_msgs::PointCloud2 cloud;
  {
    boost::mutex::scoped_lock lock(do_message_mutex_);
    cloud = rec_point_cloud_;
    rec_flag_ = false;
  }

  // 通过最新传感器数据标记指障碍物
  if(cloud.fields.size() != 3)
  {
    std::cout << "fields size err:" << cloud.fields.size() << std::endl;
    return;
  }

  if(cloud.fields[0].name != "x" || cloud.fields[1].name != "y" || cloud.fields[2].name != "z")
  {
    std::cout << "fields err:" << std::endl;
    return;
  }

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

  MarkerPoints marker_points;
  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
  {
    double px = static_cast<double>(*iter_x);
    double py = static_cast<double>(*iter_y);
    double pz = static_cast<double>(*iter_z);

    // if the obstacle is too high or too far away from the robot we won't add it
    if (pz > max_obstacle_height_ || pz < min_obstacle_height_)
    {
      continue;
    }

    // compute the squared distance from the hitpoint to the pointcloud's origin
    double sq_dist = sqrt( (px-robot_x) * (px-robot_x) + (py-robot_y) * (py-robot_y) );
    if (sq_dist >= max_obstacle_range_ || sq_dist <= min_obstacle_range_)
    {
      continue;
    }

    unsigned int mx, my;
    if (!worldToMap(px, py, mx, my))
    {
      continue;
    }
    setCost(mx,my,costmap_2d::LETHAL_OBSTACLE); // 标记障碍物
    std::pair<double,double> pair_tem(px, py);
    marker_points.points.push_back(pair_tem);
    touch(px, py, min_x, min_y, max_x, max_y);
    addExtraBounds(*min_x, *min_y, *max_x, *max_y);
  }

  if(!marker_points.points.empty())
  {
    marker_points.marker_time = ros::Time::now();
    marker_points_buffer_.push_back(marker_points);
    marker_points.points.clear();
  }
}

void DepthCameraLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_)
    return;
  updateWithMax(master_grid, min_i, min_j, max_i, max_j);
}

void DepthCameraLayer::reset()
{
  deactivate();
  resetMaps();
  current_ = true;
  activate();
}

void DepthCameraLayer::deactivate()
{
  if(sub_ptr_ != nullptr)
    sub_ptr_->unsubscribe();
}

void DepthCameraLayer::activate()
{
  if(sub_ptr_ != nullptr)
    sub_ptr_->subscribe();
}

}

