#include <iostream>
#include <napi.h>
#include <pcl/io/pcd_io.h>
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <unordered_map>


#include "ros/ros.h"
#include <pcl_conversions/pcl_conversions.h>
#include "sensor_msgs/PointCloud2.h"

std::unordered_map<std::string, ros::Publisher> pointcloud_publishers;
std::unordered_map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds;
std::shared_ptr<ros::NodeHandle> nodehandle;

void processPointCloud(const Napi::CallbackInfo& info )
{
  Napi::Env env = info.Env();

  Napi::Buffer<float> originalData = info[0].As<Napi::Buffer<float>>();
  Napi::String lidarId = info[1].As<Napi::String>();
  std::string lidarname = static_cast<std::string>(lidarId);
  float* data = originalData.Data();

  const auto& itr = pointcloud_publishers.find(lidarname);
  if (itr == pointcloud_publishers.end()) {
      pointcloud_publishers.emplace(lidarname,
        nodehandle->advertise<sensor_msgs::PointCloud2>(lidarname, 3));
  }


  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  for (unsigned int i = 0; i < originalData.Length(); i += 3) {
    cloud->push_back(pcl::PointXYZ(data[i], data[i + 1], data[i + 2]));
  }

  clouds.emplace(lidarname, cloud);
  sensor_msgs::PointCloud2 s_cloud;
  pcl::toROSMsg(*cloud, s_cloud);

  pointcloud_publishers[lidarname].publish(s_cloud);

  pcl::io::savePCDFileASCII("tempcloud.pcd", *cloud);
}

Napi::Object Init(Napi::Env env, Napi::Object exports) {
  const ros::M_string topic_remappings;

  ros::init(topic_remappings, "pointcloud_cpp");

  nodehandle = std::make_shared<ros::NodeHandle>();

  exports.Set(
    "processPointCloud", Napi::Function::New(env, processPointCloud)
  );
  return exports;
}

NODE_API_MODULE(addon, Init)

