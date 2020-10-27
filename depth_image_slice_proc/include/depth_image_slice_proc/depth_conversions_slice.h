#ifndef DEPTH_CONVERSIONS_SLICE_H
#define DEPTH_CONVERSIONS_SLICE_H

#include <sensor_msgs/Image.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <image_geometry/pinhole_camera_model.h>
#include <depth_image_slice_proc/depth_traits_slice.h>

#include <limits>

namespace depth_image_slice_proc
{

typedef sensor_msgs::PointCloud2 PointCloud;

// Handles float or uint16 depths
template<typename T>
void convert(const sensor_msgs::ImageConstPtr& depth_msg,
             PointCloud::Ptr& cloud_msg,
             const image_geometry::PinholeCameraModel& model,
             double range_max = 0.0)
{
  // Use correct principal point from calibration
  float center_x = model.cx();
  float center_y = model.cy();

  // Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
  double unit_scaling = DepthTraits<T>::toMeters( T(1) );
  float constant_x = unit_scaling / model.fx();
  float constant_y = unit_scaling / model.fy();
  float bad_point = std::numeric_limits<float>::quiet_NaN();

  sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud_msg, "z");
  const T* depth_row = reinterpret_cast<const T*>(&depth_msg->data[0]);
  int row_step = depth_msg->step / sizeof(T);
  for (int v = 0; v < (int)cloud_msg->height; ++v, depth_row += row_step)
  {
    for (int u = 0; u < (int)cloud_msg->width; ++u, ++iter_x, ++iter_y, ++iter_z)
    {
      T depth = depth_row[u];

      // Missing points denoted by NaNs
      if (!DepthTraits<T>::valid(depth))
      {
        if (range_max != 0.0)
        {
          depth = DepthTraits<T>::fromMeters(range_max);
        }
        else
        {
          *iter_x = *iter_y = *iter_z = bad_point;
          continue;
        }
      }

      // Fill in XYZ
      *iter_x = (u - center_x) * depth * constant_x;
      *iter_y = (v - center_y) * depth * constant_y;
      *iter_z = DepthTraits<T>::toMeters(depth);
    }
  }
}

} // namespace depth_image_proc

#endif
