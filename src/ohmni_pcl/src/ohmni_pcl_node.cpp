#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>

ros::Publisher publisher_;

void cloud_callback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    // Convert
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_raw(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pass(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::fromROSMsg(*msg, *cloud_raw);  // ROS1: dùng fromROSMsg trực tiếp

    // PassThrough Z
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud_raw);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.1f, 3.0f);
    pass.filter(*cloud_pass);

    // PassThrough X
    pass.setInputCloud(cloud_pass);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(-3.0f, 3.0f);
    pass.filter(*cloud_raw);

    // PassThrough Y
    pass.setInputCloud(cloud_raw);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-3.0f, 3.0f);
    pass.filter(*cloud_pass);

    // VoxelGrid
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud_pass);
    sor.setLeafSize(0.05f, 0.05f, 0.05f);
    sor.filter(*cloud_filtered);

    ROS_INFO("PointCloud after filtering: %lu data points.", cloud_filtered->size());

    // Publish
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud_filtered, output);  // ROS1: dùng toROSMsg trực tiếp
    output.header = msg->header;
    publisher_.publish(output);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "pcl_processor");
    ros::NodeHandle nh;

    // ROS1: không có QoS, dùng queue_size
    ros::Subscriber sub = nh.subscribe("input", 1, cloud_callback);
    publisher_ = nh.advertise<sensor_msgs::PointCloud2>("output", 1);

    ros::spin();
    return 0;
}