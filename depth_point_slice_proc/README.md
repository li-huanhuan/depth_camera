image_pipeline
==============
params:
  private_nh.param<std::string>("frame_base", params_.frame_base, "base_footprint"); //切片的时基于的坐标系
  private_nh.param<std::string>("frame_depth_cam", params_.frame_depth_cam, "camera_rgb_optical_frame"); //深度点云坐标系
  private_nh.param("slice_x", params_.offset_x, 10.0); //切片的x坐标，大于等于0
  private_nh.param("slice_y", params_.offset_y, 0.5); //切片的机器人左右的范围
  private_nh.param("slice_z_min", params_.offset_z_min, 0.03); //切片的最小高度
  private_nh.param("slice_z_max", params_.offset_z_max, 1.0); //切片的最大高度
  private_nh.param("slice_leaf", params_.size_leaf, 0.05); //稀疏深度点云的参数

     <arg name="manager" value="$(arg camera)_nodelet_manager" />

    <include file="$(find rgbd_launch)/launch/includes/manager.launch.xml">
       <arg name="name" value="$(arg manager)" />
       <arg name="debug" value="$(arg debug)" />
       <arg name="num_worker_threads"  value="$(arg num_worker_threads)" />
    </include>

     <node pkg="nodelet" type="nodelet" name="$(arg depth)_points_slice"
           args="load depth_point_slice_proc/point_cloud2_slice $(arg manager) --no-bond"
           respawn="$(arg respawn)">
       <remap from="point_cloud"     to="$(arg depth)/points"/>
       <remap from="slice_point"     to="$(arg depth)/cloud_slice" />

       <param name="frame_base" value="base_footprint" />
       <param name="frame_depth_cam" value="camera_rgb_optical_frame" />
       <param name="slice_x" value="10.0" />
       <param name="slice_y" value="1.0" />
       <param name="slice_z_min" value="0.03" />
       <param name="slice_z_max" value="0.30" />
       <param name="slice_leaf" value="0.05" />
     </node>


