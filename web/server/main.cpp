#include <iostream>
#include <napi.h>

void processPointCloud(const Napi::CallbackInfo& info)
{
  Napi::Env env = info.Env();
  if (info.Length() != 1 || !info[0].IsBuffer()) {
     Napi::TypeError::New(env, "Buffer expected").ThrowAsJavaScriptException();
  }
  Napi::Buffer<float> originalData = info[0].As<Napi::Buffer<float>>();
  float* data = originalData.Data();

  // Insert C++ code here, for example:
  // Set x axis to 0 to show modification
  // for (unsigned int i = 0; i < originalData.Length(); i += 3) {
  //  data[i] = 0;
  //}
}

Napi::Object Init(Napi::Env env, Napi::Object exports) {
  exports.Set(
    "processPointCloud", Napi::Function::New(env, processPointCloud)
  );
  return exports;
}

NODE_API_MODULE(addon, Init)
