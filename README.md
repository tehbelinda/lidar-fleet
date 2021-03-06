# Lidar Fleet Display

Visualiser for a fleet of lidars

- Host server uses websockets to accept point data and forwards them along to connected browsers
- Browser app for visualising the point cloud data 
- ROS node for for converting PointCloud2 messages to websocket arraybuffer

TODO: 
- Expand to multiple point clouds using lidar id

## General Setup

Install [nodejs](https://github.com/nodesource/distributions/blob/master/README.md#debinstall)

## Web

Build: `docker-compose build`

Run: `docker-compose up`

Go to `http://<host>:3002`

### Run without Docker

**Server**

```bash
cd web/server
npm install
npm run server
```

**Browser**

```bash
cd web/app
npm install
npm run build
```

For the dev version:
```bash
cd web/app
npm install
npm start
```

## Lidar

### Websocket Client

Converts PointCloud2 messages (and by default PCL as well) into an arraybuffer of Points for sending along through websocket

To build:

```bash
cd ros
catkin_make
source devel/setup.bash
```

Using the env_template file as a guide, fill in the .env file for your environment, eg HOST, PORT, etc:
```bash
cd src
cp env_template .env
vim .env
```

For example:
```
HOST=localhost
PORT=3002
LIDAR_ID=foo
LIDAR_TOPIC=/livox/lidar
```

Still in the src folder, install dependencies
```bash
npm install
```

To start the client: 
```bash
npm run start
```
OR without the env var part in the ros way...
```bash
rosrun lidar_websocket lidar_client.js
```

### Livox

Follow README for [Livox-SDK-ROS](https://github.com/Livox-SDK/Livox-SDK-ROS)

