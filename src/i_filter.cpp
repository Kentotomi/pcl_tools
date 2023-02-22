#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>

class PointCloudFilter : public rclcpp::Node
{
public:
  PointCloudFilter()
  : Node("point_cloud_filter")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "input_cloud", 10, std::bind(&PointCloudFilter::filter_callback, this, std::placeholders::_1));
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_cloud", 10);
  }

private:
    void filter_callback(const sensor_msgs::msg::PointCloud2::SharedPtr input_cloud)
    {
    // Convert PointCloud2 message to PCL point cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*input_cloud, *pcl_cloud);

    // Perform filtering
    pcl::PassThrough<pcl::PointXYZI> pass;
    pass.setInputCloud(pcl_cloud);
    pass.setFilterFieldName("intensity");
    pass.setFilterLimits(100.0, 255.0);
    pass.filter(*pcl_cloud);

    // Convert filtered PCL point cloud to PointCloud2 message
    sensor_msgs::msg::PointCloud2 filtered_cloud;
    pcl::toROSMsg(*pcl_cloud, filtered_cloud);
    filtered_cloud.header = input_cloud->header;

    // Publish filtered point cloud
    publisher_->publish(filtered_cloud);
    }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PointCloudFilter>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

int main(int argc, char** argv)
{
  // Initialize ROS node
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("point_cloud_filter");

  // Create subscriber to input point cloud topic
  std::string /livox/lidar = "/input_cloud";  // Change input topic name here
  auto subscriber = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    /livox/lidar, 10, std::bind(filter_callback, std::placeholders::_1));

  // Create publisher for filtered point cloud
  std::string /points_raw = "/filtered_cloud";  // Change output topic name here
  auto publisher = node->create_publisher<sensor_msgs::msg::PointCloud2>(/points_raw, 10);

  // Spin the node
  rclcpp::spin(node);

  return 0;
}