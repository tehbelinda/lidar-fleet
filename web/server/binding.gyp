{
  "targets": [{
    "target_name": "addon",
    "cflags": [
      "-std=c++11"
    ],
    "cflags!": [
      "-fno-rtti",
      "-fno-exceptions"
    ],
    "cflags_cc!": [
      "-fno-rtti",
      "-fno-exceptions",
    ],
    "sources": [
      "main.cpp"
    ],
    "include_dirs": [
      "/usr/include/eigen3",
      "/usr/local/include/pcl-1.9",
      "/opt/ros/melodic/include",
      "<!@(node -p \"require('node-addon-api').include\")"
    ],
    "libraries": [
      "-Wl,-rpath,/usr/local/lib",
      "-Wl,-rpath,/opt/ros/melodic/lib",
      "-L/opt/ros/melodic/lib",
      "-lroscpp",
      "-lrosconsole",
      "-lrostime",
      "-lroscpp_serialization",
      "-lboost_system",
      "-lboost_thread",
      "-pthread",
      "-lactionlib",
      "-lpcl_common",
      "-lpcl_features",
      "-lpcl_filters",
      "-lpcl_io",
      "-lpcl_kdtree",
      "-lpcl_keypoints",
      "-lpcl_ml",
      "-lpcl_octree",
      "-lpcl_outofcore",
      "-lpcl_people",
      "-lpcl_recognition",
      "-lpcl_registration",
      "-lpcl_sample_consensus",
      "-lpcl_search",
      "-lpcl_segmentation",
      "-lpcl_stereo",
      "-lpcl_surface",
      "-lpcl_tracking",
      "-lpcl_visualization"
    ],
    "dependencies": [
      "<!(node -p \"require('node-addon-api').gyp\")"
    ],
    "defines": [ 'NAPI_DISABLE_CPP_EXCEPTIONS' ]
}]
}
