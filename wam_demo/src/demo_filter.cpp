/*
 * The demo_filter node processes sensor data and publishes the
 * position of a graspable object identified in the data.
 */

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

ros::Publisher downsampled_pub;
ros::Publisher filtered_pub;
ros::Publisher object_position_pub;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_in)
{
  // convert sensor_msg to pcl type
  pcl::PCLPointCloud2::Ptr cloud_raw(new pcl::PCLPointCloud2);
  pcl_conversions::toPCL(*cloud_in, *cloud_raw);

  pcl::PCLPointCloud2 cloud_downsampled;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_depth(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCLPointCloud2 cloud_filtered_msg;

  // Downsample input cloud
  // http://wiki.ros.org/pcl
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud(cloud_raw);
  sor.setLeafSize (0.01, 0.01, 0.01);
  sor.filter(cloud_downsampled);
  // Publish the downsampled cloud
  downsampled_pub.publish(cloud_downsampled);

  // Through pass filter cloud to remove all points outside of the table
  // http://pointclouds.org/documentation/tutorials/passthrough.php
  // First extract the table depth-wise
  pcl::PassThrough<pcl::PointXYZ> pass_depth;
  pcl::fromPCLPointCloud2(cloud_downsampled, *cloud);
  pass_depth.setInputCloud(cloud);
  pass_depth.setFilterFieldName ("x"); // the axis for depth may be different in the point cloud. Display the filtered output in RViz to verify.
  pass_depth.setFilterLimits (0.2, 1.0); // adjust the limits so that the table is extracted from the point cloud depth-wise
  pass_depth.filter(*cloud_filtered_depth);
  // Then extract the table length-wise
  pcl::PassThrough<pcl::PointXYZ> pass_length;
  pass_length.setInputCloud(cloud_filtered_depth);
  pass_length.setFilterFieldName ("y"); // length axis may be different
  pass_length.setFilterLimits (-0.4, 0.4); // tune the limits
  pass_length.filter(*cloud_filtered);
  // [Publish]
  pcl::toPCLPointCloud2(*cloud_filtered, cloud_filtered_msg);
  filtered_pub.publish(cloud_filtered_msg);

  // Find highest point in cloud. This should roughly correspond to the position of the graspable object on the table.
  // Can loop through all points using cloud_filtered.size()
  // geometry_msgs::Point object_position;
  // object_position <- position;
  // object_position_pub.publish(object_position);
}

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "demo_filter");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud. The name might be different, but you can verify it by trying to view PointCloud2 data in RViz
  ros::Subscriber sub = nh.subscribe ("wam_base_kinect/depth_registered/points", 1, cloud_cb);

  // Create a ROS publisher for the processed point clouds
  downsampled_pub = nh.advertise<pcl::PCLPointCloud2> ("downsampled_cloud", 1);
  filtered_pub = nh.advertise<pcl::PCLPointCloud2> ("filtered_cloud", 1);

  // Publisher for object position
  object_position_pub = nh.advertise<geometry_msgs::Point> ("object_position", 1);

  // Spin
  ros::spin ();
}
