{
  "targets": [{
    "target_name": "addon",
    "cflags!": [
      "-std=c++11",
      "-fno-exceptions"
    ],
    "cflags_cc!": [ "-fno-exceptions" ],
    "sources": [
        "main.cpp"
    ],
    'include_dirs': [
        "<!@(node -p \"require('node-addon-api').include\")"
    ],
    'libraries': [],
    'dependencies': [
        "<!(node -p \"require('node-addon-api').gyp\")"
    ],
    'defines': [ 'NAPI_DISABLE_CPP_EXCEPTIONS' ]
}]
}
