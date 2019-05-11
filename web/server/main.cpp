#include <iostream>
#include <napi.h>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

Napi::Buffer<float> processPointCloud(const Napi::CallbackInfo& info)
{
  Napi::Env env = info.Env();
  if (info.Length() != 1 || !info[0].IsBuffer()) {
     Napi::TypeError::New(env, "Buffer expected").ThrowAsJavaScriptException();
  }
  Napi::Buffer<float> originalData = info[0].As<Napi::Buffer<float>>();
  float* data = originalData.Data();

  pcl::PointCloud<pcl::PointXYZ> cloud;
  for (unsigned int i = 0; i < originalData.Length(); i += 3) {
    cloud.push_back(pcl::PointXYZ(data[i], data[i + 1], data[i + 2]));
  }

  pcl::io::savePCDFileASCII("tempcloud.pcd", cloud);

  cv::Mat image = cv::Mat(cloud.height, cloud.width, CV_8UC3);
  cv::imwrite("tempimage.png", image);

  return originalData;
}

Napi::Object Init(Napi::Env env, Napi::Object exports) {
  exports.Set(
    "processPointCloud", Napi::Function::New(env, processPointCloud)
  );
  return exports;
}

NODE_API_MODULE(addon, Init)
