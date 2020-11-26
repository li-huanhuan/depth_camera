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
#include <tf/transform_broadcaster.h>

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

  SliceParam slice_param_;
  ros::Time compute_camera_tf_time_;
  tf::StampedTransform transform_;
  Eigen::Matrix4f baseTcloud_eigen_;
  tf::TransformBroadcaster broadcaster_;

  virtual void onInit();

  void getBaseToCameraTF(Eigen::Matrix4f& baseTcloud_eigen,SliceParam slice_param);

  void connectCb();

  Eigen::Matrix3f hat(Eigen::Vector3f w);

  Eigen::Matrix3f computeRotationMatrix(Eigen::Vector3f vec_n, Eigen::Vector3f Z);

  tf::Quaternion computeDepthCameraOrientation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane);

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

  double tf_t_x, tf_t_y, tf_t_z;
  while(private_nh.hasParam("tf_t_x"))
  {
    ROS_WARN("The depth camera layer is waiting for TF conversion parameters.:%s",private_nh.getNamespace().c_str());
    ros::Duration(1).sleep();
  }
  private_nh.param("tf_t_x", tf_t_x, 0.207);
  private_nh.param("tf_t_y", tf_t_y, 0.0);
  private_nh.param("tf_t_z", tf_t_z, 0.8);

  transform_.setOrigin(tf::Vector3(tf_t_x, tf_t_y, tf_t_z));

  compute_camera_tf_time_ = ros::Time::now() - ros::Duration(100);

  // Monitor whether anyone is subscribed to the output
  ros::SubscriberStatusCallback connect_cb = boost::bind(&PointCloudXyzSliceNodelet::connectCb, this);

  // Make sure we don't enter connectCb() between advertising and assigning to pub_point_cloud_
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  pub_point_slice_ = nh.advertise<sensor_msgs::PointCloud2>("cloud_slice", 1, connect_cb, connect_cb);
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

Eigen::Matrix3f PointCloudXyzSliceNodelet::hat(Eigen::Vector3f w)
{
  Eigen::Matrix3f W = Eigen::Matrix3f::Zero();

  W(1, 2) = -1 * w[0];
  W(0, 2) = w[1];
  W(0, 1) = -1 * w[2];

  W(2, 1) = -1 * W(1, 2);
  W(2, 0) = -1 * W(0, 2);
  W(1, 0) = -1 * W(0, 1);

  return W;
}

Eigen::Matrix3f PointCloudXyzSliceNodelet::computeRotationMatrix(Eigen::Vector3f vec_n, Eigen::Vector3f Z)
{
  Eigen::Matrix3f R;
  Eigen::Vector3f cross = vec_n.cross(Z);
  Eigen::Vector3f w = cross / (cross.norm());
  Eigen::Matrix3f w_hat = hat(w);

  float rot_angle = std::acos(vec_n.dot(Z) / (vec_n.norm()));

  R = Eigen::Matrix3f::Identity() + w_hat * std::sin(rot_angle) + w_hat * w_hat * (1 - cos(rot_angle));

  return R;
}

tf::Quaternion PointCloudXyzSliceNodelet::computeDepthCameraOrientation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane)
{
  tf::Quaternion q_baseTcam;

  // compute the normal vector for the plane;
  int N = cloud_plane->points.size();
  Eigen::MatrixXf X = Eigen::MatrixXf::Zero(3, N);
  for (int k = 0; k < N; k++)
  {
    pcl::PointXYZ &p = cloud_plane->points[k];
    X(0, k) = p.x;
    X(1, k) = p.y;
    X(2, k) = p.z;
  }

  Eigen::Vector3f mean(X.row(0).sum() / N, X.row(1).sum() / N, X.row(2).sum() / N);

  Eigen::MatrixXf X_ = Eigen::MatrixXf::Zero(3, N);
  for (int k = 0; k < N; k++)
  {
    X_.col(k) = X.col(k) - mean;
  }
  Eigen::Matrix3f cov;
  cov = X_ * X_.transpose();

  Eigen::JacobiSVD<Eigen::Matrix3f> svd(cov, Eigen::ComputeFullU | Eigen::ComputeFullV);

  Eigen::Vector3f lambda = svd.singularValues();
  Eigen::Matrix3f U = svd.matrixU();
  Eigen::Vector3f vec_in_plane = U.col(0);
  Eigen::Vector3f vec_n = U.col(2);
  vec_n *= -1;

  // compute the rotation matrix from the plane normal vector to z-axis of base frame;
  Eigen::Vector3f Z(0, 0, 1);
  Eigen::Matrix3f R = computeRotationMatrix(vec_n, Z);

  Eigen::Quaternionf q_R(R);
  tf::Transform T_z_upwards;
  T_z_upwards.setRotation(tf::Quaternion(q_R.x(), q_R.y(), q_R.z(), q_R.w()));
  tf::Transform T_x_forwards;
  T_x_forwards.setRotation(tf::createQuaternionFromRPY(0, 0, -M_PI * 0.5));
  tf::Transform T_rot = T_x_forwards * T_z_upwards;

  tf::Quaternion q_z_forwards(q_R.x(), q_R.y(), q_R.z(), q_R.w());
  tf::Quaternion q_y_forwards = tf::createQuaternionFromYaw(-M_PI * 0.5);
  q_baseTcam = q_y_forwards * q_z_forwards;

  return q_baseTcam;
}

void PointCloudXyzSliceNodelet::depthCb(const sensor_msgs::ImageConstPtr& depth_msg, const sensor_msgs::CameraInfoConstPtr& info_msg)
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

  if( (ros::Time::now().toSec() - compute_camera_tf_time_.toSec()) >= 60)
  {
    ROS_INFO_ONCE("pub_tf_inc:%f",ros::Time::now().toSec() - compute_camera_tf_time_.toSec());
    // 过滤出来的地面;
    extract_indices_plane.setNegative(false);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());
    extract_indices_plane.filter(*cloud_plane);
    tf::Quaternion tf_qua = computeDepthCameraOrientation(cloud_plane);
    transform_.setRotation(tf_qua);

    // transform the cloud of obstacle to base frame;
    baseTcloud_eigen_.setIdentity();
    baseTcloud_eigen_.block(0, 0, 3, 3) = Eigen::Quaternionf(tf_qua.getW(), tf_qua.getX(), tf_qua.getY(),tf_qua.getZ()).toRotationMatrix();
    baseTcloud_eigen_(0, 3) = transform_.getOrigin().getX();
    baseTcloud_eigen_(1, 3) = transform_.getOrigin().getY();
    baseTcloud_eigen_(2, 3) = transform_.getOrigin().getZ();

    compute_camera_tf_time_ = ros::Time::now();
  }
  broadcaster_.sendTransform(tf::StampedTransform(transform_, depth_msg->header.stamp, this->slice_param_.frame_base, depth_msg->header.frame_id));

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
