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
      "/usr/local/include/opencv2",
      "/usr/local/include/pcl-1.9",
      "<!@(node -p \"require('node-addon-api').include\")"
    ],
    "libraries": [
      "-Wl,-rpath,/usr/local/lib",
      "-lopencv_core",
      "-lopencv_calib3d",
      "-lopencv_dnn",
      "-lopencv_features2d",
      "-lopencv_flann",
      "-lopencv_highgui",
      "-lopencv_imgcodecs",
      "-lopencv_imgproc",
      "-lopencv_ml",
      "-lopencv_objdetect",
      "-lopencv_photo",
      "-lopencv_shape",
      "-lopencv_stitching",
      "-lopencv_superres",
      "-lopencv_video",
      "-lopencv_videoio",
      "-lopencv_videostab",
      "-lopencv_viz",
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
