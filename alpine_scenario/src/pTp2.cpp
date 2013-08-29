#include <ros/ros.h>
#include <stdio.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/ref.hpp>

ros::Publisher pub; 
ros::Subscriber sub;

  void converter (const sensor_msgs::PointCloudConstPtr &cloud_in)
  {
    sensor_msgs::PointCloud2 cloud2;
    sensor_msgs::convertPointCloudToPointCloud2(*cloud_in, cloud2);
    pub.publish(cloud2);
  }


  int main (int argc, char** argv)
  {
    ROS_INFO("Initializing the converter_node");
    ros::init (argc, argv, "pointcloudconverter");
    // Create a ROS subscriber for the input point cloud
    ros::NodeHandle nh;
    sensor_msgs::PointCloud cloud;    
    sub = nh.subscribe ("/assembled_cloud", 1000, converter);
    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2> ("cloud1_assembled", 1);
    // Spin
    ros::spin ();
  }
