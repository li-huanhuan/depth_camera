#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <image_geometry/pinhole_camera_model.h>
#include <boost/thread.hpp>
#include <depth_image_slice_proc/depth_conversions_slice.h>
#include <sensor_msgs/point_cloud2_iterator.h>

//add
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <Eigen/Geometry>

namespace depth_image_slice_proc {

struct SliceParam
{
  std::string frame_base;
  std::string frame_depth_cam;
  double size_leaf;
  double offset_x;
  double offset_y;
  double offset_z_min;
  double offset_z_max;
};

class PointCloudXyzSliceNodelet : public nodelet::Nodelet
{
  // Subscriptions
  boost::shared_ptr<image_transport::ImageTransport> it_;
  image_transport::CameraSubscriber sub_depth_;

  // pub
  ros::Publisher pub_point_slice_;

  int queue_size_;

  // Publications
  boost::mutex connect_mutex_;

  image_geometry::PinholeCameraModel model_;

  Eigen::Matrix4f baseTcloud_eigen_;
  SliceParam slice_param_;

  virtual void onInit();

  void getBaseToCameraTF(Eigen::Matrix4f& baseTcloud_eigen,SliceParam slice_param);

  void connectCb();

  void depthCb(const sensor_msgs::ImageConstPtr& depth_msg, const sensor_msgs::CameraInfoConstPtr& info_msg);
};

void PointCloudXyzSliceNodelet::onInit()
{
  ros::NodeHandle& nh         = getNodeHandle();
  ros::NodeHandle& private_nh = getPrivateNodeHandle();
  it_.reset(new image_transport::ImageTransport(nh));

  // Read parameters
  private_nh.param<std::string>("frame_base", slice_param_.frame_base, "base_footprint");
  private_nh.param<std::string>("frame_depth_cam", slice_param_.frame_depth_cam, "camera_rgb_optical_frame");
  private_nh.param("slice_x",  slice_param_.offset_x, 1.0);
  private_nh.param("slice_y", slice_param_.offset_y, 1.0);
  private_nh.param("slice_z_min", slice_param_.offset_z_min, 0.03);
  private_nh.param("slice_z_max", slice_param_.offset_z_max, 0.30);
  private_nh.param("size_leaf", slice_param_.size_leaf, 0.01);

  getBaseToCameraTF(baseTcloud_eigen_, slice_param_);

  // Monitor whether anyone is subscribed to the output
  ros::SubscriberStatusCallback connect_cb = boost::bind(&PointCloudXyzSliceNodelet::connectCb, this);

  // Make sure we don't enter connectCb() between advertising and assigning to pub_point_cloud_
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  pub_point_slice_ = nh.advertise<sensor_msgs::PointCloud2>("cloud_slice", 1, connect_cb, connect_cb);
}

void PointCloudXyzSliceNodelet::getBaseToCameraTF(Eigen::Matrix4f& baseTcloud_eigen, SliceParam slice_param)
{
  // get transformation from base frame to depth camera frame;
  tf::TransformListener* tf_ = new tf::TransformListener(ros::Duration(10));
  tf::StampedTransform transform;

  while(ros::ok())
  {
    if(tf_->waitForTransform(slice_param.frame_base, ros::Time::now(), slice_param.frame_depth_cam, ros::Time::now(), slice_param.frame_depth_cam, ros::Duration(1)))
    {
      tf_->lookupTransform(slice_param.frame_base,slice_param.frame_depth_cam,ros::Time::now(),transform);
      break;
    }

    ROS_WARN("PointCloudXyzSliceNodelet frame %s to %s unavailable",slice_param_.frame_base.c_str(),slice_param_.frame_depth_cam.c_str());
    ros::Duration(0.5).sleep();
  }

  delete tf_;
  tf_ = nullptr;

  // transform the cloud of obstacle to base frame;
  baseTcloud_eigen.setIdentity();
  tf::Quaternion q = transform.getRotation();
  baseTcloud_eigen.block(0, 0, 3, 3) = Eigen::Quaternionf(q.w(), q.x(), q.y(),q.z()).toRotationMatrix();
  baseTcloud_eigen(0, 3) = transform.getOrigin().getX();
  baseTcloud_eigen(1, 3) = transform.getOrigin().getY();
  baseTcloud_eigen(2, 3) = transform.getOrigin().getZ();

  std::cout << "baseTdepth:" << std::endl;
  std::cout << "Tx:" << transform.getOrigin().getX() << std::endl;
  std::cout << "Ty:" << transform.getOrigin().getY() << std::endl;
  std::cout << "Tz:" << transform.getOrigin().getZ() << std::endl;
  std::cout << "Rx:" << transform.getRotation().getX() << std::endl;
  std::cout << "Ry:" << transform.getRotation().getY() << std::endl;
  std::cout << "Rz:" << transform.getRotation().getZ() << std::endl;
  std::cout << "Rw:" << transform.getRotation().getW() << std::endl;
}

// Handles (un)subscribing when clients (un)subscribe
void PointCloudXyzSliceNodelet::connectCb()
{
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  if (pub_point_slice_.getNumSubscribers() == 0)
  {
    sub_depth_.shutdown();
  }
  else if (!sub_depth_)
  {
    image_transport::TransportHints hints("raw", ros::TransportHints(), getPrivateNodeHandle());
    sub_depth_ = it_->subscribeCamera("image_rect", static_cast<uint32_t>(queue_size_) , &PointCloudXyzSliceNodelet::depthCb, this, hints);
  }
}

void PointCloudXyzSliceNodelet::depthCb(const sensor_msgs::ImageConstPtr& depth_msg,
                                        const sensor_msgs::CameraInfoConstPtr& info_msg)
{
  sensor_msgs::PointCloud2::Ptr cloud_msg(new sensor_msgs::PointCloud2);
  cloud_msg->header = depth_msg->header;
  cloud_msg->height = depth_msg->height;
  cloud_msg->width  = depth_msg->width;
  cloud_msg->is_dense = false;
  cloud_msg->is_bigendian = false;

  sensor_msgs::PointCloud2Modifier pcd_modifier(*cloud_msg);
  pcd_modifier.setPointCloud2FieldsByString(1, "xyz");

  // Update camera model
  model_.fromCameraInfo(info_msg);

  if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
  {
    convert<uint16_t>(depth_msg, cloud_msg, model_);
  }
  else if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
  {
    convert<float>(depth_msg, cloud_msg, model_);
  }
  else
  {
    NODELET_ERROR_THROTTLE(5, "Depth image has unsupported encoding [%s]", depth_msg->encoding.c_str());
    return;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_data(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*cloud_msg, *cloud_data);

  // down-sample the cloud of raw data;
  pcl::VoxelGrid<pcl::PointXYZ> filter_voxel;
  filter_voxel.setLeafSize(slice_param_.size_leaf, slice_param_.size_leaf, slice_param_.size_leaf);
  filter_voxel.setInputCloud(cloud_data);
  filter_voxel.filter(*cloud_data);

  if(cloud_data->empty())
  {
    sensor_msgs::PointCloud2::Ptr msg_cloud_obstacle_slice(new sensor_msgs::PointCloud2);
    msg_cloud_obstacle_slice->header.stamp = depth_msg->header.stamp;
    msg_cloud_obstacle_slice->header.frame_id = slice_param_.frame_base; //fields
    msg_cloud_obstacle_slice->width = 0;
    msg_cloud_obstacle_slice->height = 1; //is_bigendian
    msg_cloud_obstacle_slice->fields.resize(3);
    msg_cloud_obstacle_slice->fields[0].name = "x";
    msg_cloud_obstacle_slice->fields[0].offset = 0;
    msg_cloud_obstacle_slice->fields[0].datatype = 7;
    msg_cloud_obstacle_slice->fields[0].count = 1;

    msg_cloud_obstacle_slice->fields[1].name = "y";
    msg_cloud_obstacle_slice->fields[1].offset = 4;
    msg_cloud_obstacle_slice->fields[1].datatype = 7;
    msg_cloud_obstacle_slice->fields[1].count = 1;

    msg_cloud_obstacle_slice->fields[2].name = "z";
    msg_cloud_obstacle_slice->fields[2].offset = 8;
    msg_cloud_obstacle_slice->fields[2].datatype = 7;
    msg_cloud_obstacle_slice->fields[2].count = 1;

    msg_cloud_obstacle_slice->is_bigendian = false;
    msg_cloud_obstacle_slice->point_step = 16;
    msg_cloud_obstacle_slice->row_step = 0;
    msg_cloud_obstacle_slice->is_dense = true;
    pub_point_slice_.publish(msg_cloud_obstacle_slice);
    return;
  }

  // estimate the plane model parameters together with the inlier markings;
  pcl::SACSegmentation<pcl::PointXYZ> seg_solver;
  seg_solver.setOptimizeCoefficients(true);
  seg_solver.setModelType(pcl::SACMODEL_PLANE);
  seg_solver.setMethodType(pcl::SAC_RANSAC);
  seg_solver.setMaxIterations(200);
  seg_solver.setDistanceThreshold(0.01);
  seg_solver.setInputCloud(cloud_data);

  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  pcl::ModelCoefficients::Ptr model_coefficients(new pcl::ModelCoefficients());
  seg_solver.segment(*inliers, *model_coefficients);

  // extract the inliers for the plane model;
  pcl::ExtractIndices<pcl::PointXYZ> extract_indices_plane;
  extract_indices_plane.setInputCloud(cloud_data);
  extract_indices_plane.setIndices(inliers);

  /********************
  // 过滤出来的地面;
  extract_indices_plane.setNegative(false);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());
  extract_indices_plane.filter(*cloud_plane);
  *******************/

  // 过滤掉地面之后;
  extract_indices_plane.setNegative(true);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_obstacle(new pcl::PointCloud<pcl::PointXYZ>());
  extract_indices_plane.filter(*cloud_obstacle);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_obstacle_base_slice(new pcl::PointCloud<pcl::PointXYZ>());
  size_t points_size = cloud_obstacle->points.size();
  for (size_t k = 0; k < points_size; k++)
  {
    Eigen::Vector4f point(cloud_obstacle->points[k].x, cloud_obstacle->points[k].y, cloud_obstacle->points[k].z, 1.0);
    Eigen::Vector4f point_base = baseTcloud_eigen_ * point;

    if(point_base[2] > slice_param_.offset_z_min &&
       point_base[2] < slice_param_.offset_z_max &&
       point_base[1] > slice_param_.offset_y * (-1) &&
       point_base[1] < slice_param_.offset_y &&
       point_base[0] < slice_param_.offset_x)
    {
       cloud_obstacle_base_slice->points.push_back(pcl::PointXYZ(point_base[0], point_base[1],point_base[2]));
    }
  }

  cloud_obstacle_base_slice->width = cloud_obstacle_base_slice->points.size();
  cloud_obstacle_base_slice->height = 1;

  // publish the obstacle slice;
  sensor_msgs::PointCloud2::Ptr msg_cloud_obstacle_slice(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(*cloud_obstacle_base_slice, *msg_cloud_obstacle_slice);
  msg_cloud_obstacle_slice->header.stamp = depth_msg->header.stamp;
  msg_cloud_obstacle_slice->header.frame_id = slice_param_.frame_base;
  pub_point_slice_.publish(msg_cloud_obstacle_slice);
}

}

// Register as nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(depth_image_slice_proc::PointCloudXyzSliceNodelet,nodelet::Nodelet);
