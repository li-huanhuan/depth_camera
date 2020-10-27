#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include <Eigen/Geometry>

#include <boost/thread.hpp>

namespace depth_point_slice_proc {

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

class PointCloud2SliceNodelet : public nodelet::Nodelet
{
  ros::NodeHandle nh;
  boost::mutex connect_mutex_;
  ros::Subscriber sub_point_cloud_;
  ros::Publisher pub_point_slice_;
  Eigen::Matrix4f baseTcloud_eigen_;
  SliceParam params_;
  void connectCb();
  void pointCloudCb(sensor_msgs::PointCloud2::ConstPtr msg);
  virtual void onInit();
};

void PointCloud2SliceNodelet::connectCb()
{
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  if (pub_point_slice_.getNumSubscribers() == 0)
  {
    sub_point_cloud_.shutdown();
  }
  else
  {
    sub_point_cloud_ = nh.subscribe<sensor_msgs::PointCloud2>("point_cloud", 1, boost::bind(&PointCloud2SliceNodelet::pointCloudCb, this, _1));
  }
}

void PointCloud2SliceNodelet::onInit()
{
  nh = getMTNodeHandle();
  ros::NodeHandle& private_nh = getPrivateNodeHandle();
  //  sub_point_cloud_ = nh.subscribe<sensor_msgs::PointCloud2>("point_cloud", 1, boost::bind(&PointCloud2SliceNodelet::pointCloudCb, this, _1));

  // Monitor whether anyone is subscribed to the output
  ros::SubscriberStatusCallback connect_cb = boost::bind(&PointCloud2SliceNodelet::connectCb, this);
  // Make sure we don't enter connectCb() between advertising and assigning to pub_point_cloud_
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  pub_point_slice_ = nh.advertise<sensor_msgs::PointCloud2>("slice_point", 1, connect_cb, connect_cb);

  private_nh.param<std::string>("frame_base", params_.frame_base, "base_footprint");
  private_nh.param<std::string>("frame_depth_cam", params_.frame_depth_cam, "camera_rgb_optical_frame");
  private_nh.param("slice_x", params_.offset_x, 10.0);
  private_nh.param("slice_y", params_.offset_y, 0.5);
  private_nh.param("slice_z_min", params_.offset_z_min, 0.03);
  private_nh.param("slice_z_max", params_.offset_z_max, 1.0);
  private_nh.param("slice_leaf", params_.size_leaf, 0.05);

  tf::TransformListener* tf_ = new tf::TransformListener(ros::Duration(10));
  tf::StampedTransform transform;

  while(ros::ok())
  {
    if(tf_->waitForTransform(params_.frame_base, ros::Time::now(), params_.frame_depth_cam, ros::Time::now(), params_.frame_depth_cam, ros::Duration(1)))
    {
      tf_->lookupTransform(params_.frame_base,params_.frame_depth_cam,ros::Time::now(),transform);
      break;
    }
    ROS_WARN("frame %s to %s unavailable",params_.frame_base.c_str(),params_.frame_depth_cam.c_str());
    ros::Duration(0.5).sleep();
  }

  delete tf_;
  tf_ = nullptr;

  // transform the cloud of obstacle to base frame;
  baseTcloud_eigen_.setIdentity();
  baseTcloud_eigen_.block(0, 0, 3, 3) = Eigen::Quaternionf(transform.getRotation().getW(), transform.getRotation().getX(), transform.getRotation().getY(),transform.getRotation().getZ()).toRotationMatrix();
  baseTcloud_eigen_(0, 3) = transform.getOrigin().getX();
  baseTcloud_eigen_(1, 3) = transform.getOrigin().getY();
  baseTcloud_eigen_(2, 3) = transform.getOrigin().getZ();

  std::cout << "depth camera: " << params_.frame_base << " to " << params_.frame_depth_cam << ":" << std::endl;
  std::cout << "Tx:" << transform.getOrigin().getX() << std::endl;
  std::cout << "Ty:" << transform.getOrigin().getY() << std::endl;
  std::cout << "Tz:" << transform.getOrigin().getZ() << std::endl;
  std::cout << "Rx:" << transform.getRotation().getX() << std::endl;
  std::cout << "Ry:" << transform.getRotation().getY() << std::endl;
  std::cout << "Rz:" << transform.getRotation().getZ() << std::endl;
  std::cout << "Rw:" << transform.getRotation().getW() << std::endl;
}

void PointCloud2SliceNodelet::pointCloudCb(sensor_msgs::PointCloud2::ConstPtr msg)
{
  // data adaption;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_data(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*msg, *cloud_data);

  // down-sample the cloud of raw data;
  pcl::VoxelGrid<pcl::PointXYZ> filter_voxel;
  filter_voxel.setLeafSize(params_.size_leaf, params_.size_leaf, params_.size_leaf);
  filter_voxel.setInputCloud(cloud_data);
  filter_voxel.filter(*cloud_data);

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
  // filter out plane obstacle;
  extract_indices_plane.setNegative(false);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());
  extract_indices_plane.filter(*cloud_plane);
  *******************/

  // filter out non-plane obstacle;
  extract_indices_plane.setNegative(true);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_obstacle(new pcl::PointCloud<pcl::PointXYZ>());
  extract_indices_plane.filter(*cloud_obstacle);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_obstacle_base_slice(new pcl::PointCloud<pcl::PointXYZ>());
  //  cloud_obstacle_base_slice->points.reserve(cloud_obstacle->points.size());

  size_t points_size = cloud_obstacle->points.size();
  for (size_t k = 0; k < points_size; k++)
  {
    Eigen::Vector4f point(cloud_obstacle->points[k].x, cloud_obstacle->points[k].y, cloud_obstacle->points[k].z, 1.0);
    Eigen::Vector4f point_base = baseTcloud_eigen_ * point;
    if(point_base[2] > params_.offset_z_min &&
       point_base[2] < params_.offset_z_max &&
       point_base[1] > params_.offset_y * (-1) &&
       point_base[1] < params_.offset_y &&
       point_base[0] < params_.offset_x)
    {
       cloud_obstacle_base_slice->points.push_back(pcl::PointXYZ(point_base[0], point_base[1],point_base[2]));
    }
  }

  cloud_obstacle_base_slice->width = cloud_obstacle_base_slice->points.size();
  cloud_obstacle_base_slice->height = 1;

  // publish the obstacle slice;
  sensor_msgs::PointCloud2::Ptr msg_cloud_obstacle_slice(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(*cloud_obstacle_base_slice, *msg_cloud_obstacle_slice);
  msg_cloud_obstacle_slice->header.stamp = msg->header.stamp;
  msg_cloud_obstacle_slice->header.frame_id = params_.frame_base;
  pub_point_slice_.publish(msg_cloud_obstacle_slice);
}

}
// Register as nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(depth_point_slice_proc::PointCloud2SliceNodelet,nodelet::Nodelet);
