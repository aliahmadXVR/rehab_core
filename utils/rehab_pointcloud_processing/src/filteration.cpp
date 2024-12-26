// ####################### Statistical Outlier Removal ###############################
// #include <ros/ros.h>
// #include <iostream>
// #include <pcl/io/pcd_io.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <pcl_ros/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl/filters/statistical_outlier_removal.h>
// #include <pcl_conversions/pcl_conversions.h>

// ros::Publisher filtered_pub;

// void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& input_cloud_msg)
// {
//     // Convert ROS PointCloud2 message to PCL point cloud
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
//     pcl::fromROSMsg(*input_cloud_msg, *cloud);

//     // Create the filtering object
//     pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
//     sor.setInputCloud(cloud);
//     sor.setMeanK(50);
//     sor.setStddevMulThresh(1);

//     // Filter the point cloud
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
//     sor.filter(*cloud_filtered);

//     // Convert the filtered point cloud back to a ROS message
//     sensor_msgs::PointCloud2 filtered_cloud_msg;
//     pcl::toROSMsg(*cloud_filtered, filtered_cloud_msg);
//     filtered_cloud_msg.header = input_cloud_msg->header;

//     // Publish the filtered point cloud
//     filtered_pub.publish(filtered_cloud_msg);
// }

// int main(int argc, char** argv)
// {
//     ros::init(argc, argv, "filteration_node");
//     ros::NodeHandle nh;

//     std::cout << "Filteration Started!" << std::endl;

//     // Subscribe to the depth_points topic
//     ros::Subscriber cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth_registered/points", 1, cloudCallback);

//     // Publish the filtered point cloud on the filtered_points topic
//     filtered_pub = nh.advertise<sensor_msgs::PointCloud2>("/filtered_depth", 1);

//     ros::spin();

//     return 0;
// }
//##########################################################################

// ####################### Radius Outlier Removal ###############################
#include <ros/ros.h>
#include <iostream>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl_conversions/pcl_conversions.h>

ros::Publisher filtered_pub;

void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& input_cloud)
{
  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  pcl::fromROSMsg(*input_cloud, cloud);

  if (cloud.isOrganized())
  {
    pcl::PointCloud<pcl::PointXYZRGB> cloud_filtered;

    // Check for valid points in the input cloud
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(cloud, cloud_filtered, indices);

    if (cloud_filtered.empty()) {
      ROS_WARN("\n\nInput cloud is empty or contains only invalid points.");
      return;
    }

    pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;
    outrem.setInputCloud(cloud_filtered.makeShared());
    outrem.setRadiusSearch(0.10); //Increase if you want to filter out more points
    outrem.setMinNeighborsInRadius(5); //Increase if you want to filter out more points
    outrem.setKeepOrganized(true);

    try {
      outrem.filter(cloud_filtered);
    } catch (const std::exception& e) {
      ROS_ERROR_STREAM("\n\nAn Error occured during Radius outlier removal:" << e.what());
      return;
    }

    sensor_msgs::PointCloud2 filtered_cloud_msg;
    pcl::toROSMsg(cloud_filtered, filtered_cloud_msg);
    filtered_cloud_msg.header = input_cloud->header;
    filtered_pub.publish(filtered_cloud_msg);
  }
  else {ROS_WARN("\n\nInput Cloud is not Organized. Skipping Processing!!!!");
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "filteration_node");
  ros::NodeHandle nh;

  ros::Subscriber cloud_sub = nh.subscribe("/camera/depth_registered/points", 1, cloudCallback);
  filtered_pub = nh.advertise<sensor_msgs::PointCloud2>("/filtered_depth", 1);

  ros::spin();

  return 0;
}
