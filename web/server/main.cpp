#include <iostream>
#include <napi.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <unordered_map>

std::unordered_map<std::string, int> pointcloud_counts;
std::unordered_map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds;


void processPointCloud(const Napi::CallbackInfo& info )
{
  Napi::Env env = info.Env();

  Napi::Buffer<float> originalData = info[0].As<Napi::Buffer<float>>();
  Napi::String lidarId = info[1].As<Napi::String>();
  std::string lidarname = static_cast<std::string>(lidarId);
  float* data = originalData.Data();

  const auto& itr = pointcloud_counts.find(lidarname);
  if (itr == pointcloud_counts.end()) {
      pointcloud_counts.emplace(lidarname, 0);
  }
  pointcloud_counts[lidarname]++;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  for (unsigned int i = 0; i < originalData.Length(); i += 3) {
    cloud->push_back(pcl::PointXYZ(data[i], data[i + 1], data[i + 2]));
  }

  clouds.emplace(lidarname, cloud);

  pcl::io::savePCDFileASCII("tempcloud.pcd", *cloud);
}

Napi::Object Init(Napi::Env env, Napi::Object exports) {
  exports.Set(
    "processPointCloud", Napi::Function::New(env, processPointCloud)
  );
  return exports;
}

NODE_API_MODULE(addon, Init)
