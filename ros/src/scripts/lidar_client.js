#!/usr/bin/env node

const dotenv = require('dotenv');
const rosnodejs = require('rosnodejs');
const WebSocket = require('ws');

rosnodejs.loadAllPackages();
const sensor_msgs = rosnodejs.require('sensor_msgs').msg;

dotenv.config();
const HOST = process.env.HOST || 'localhost';
const PORT = process.env.PORT || 3002;
const LIDAR_ID = process.env.LIDAR_ID || 'foo';
const LIDAR_TOPIC = process.env.LIDAR_TOPIC || '/pointcloud';

const RECONNECT_INTERVAL = 10 * 1000; // 10s
let socket;
function connect() {
  if (socket) return;
  console.log('Creating websocket to', HOST, PORT);
  socket = new WebSocket('ws://' + HOST + ':' + PORT + '/lidar/' + LIDAR_ID);
  socket.on('open', () => {
    console.log('Connected to server as', LIDAR_ID);
  });
  socket.on('error', () => {
    console.log('Error from server');
    socket = null;
    setTimeout(connect, RECONNECT_INTERVAL);
  });
  socket.on('close', () => {
    console.log('Disconnected from server');
    socket = null;
    setTimeout(connect, RECONNECT_INTERVAL);
  });
};
connect();

rosnodejs.initNode('/ros_websocket').then((rosNode) => {
  console.log('Ros websocket node started'); 
  rosNode.subscribe(LIDAR_TOPIC, sensor_msgs.PointCloud2, (msg) => {
    if (!socket || socket.readyState != WebSocket.OPEN) {
      return;
    }

    // Convert to arraybuffer of points
    const buf = msg.data;
    const numPoints = msg.width;
    const positions = new Float32Array(numPoints * 3);
    const fields = {};
    for (let field of msg.fields) {
      fields[field.name] = field;
    }
    const xOffset = fields.x.offset;
    const yOffset = fields.y.offset;
    const zOffset = fields.z.offset;
    const scale = 1; // For scaling point dim space
    let byteOffset;
    for (let i = 0; i < numPoints; i++) {
      byteOffset = i * msg.point_step;
      positions[i * 3 + 0] = buf.readFloatLE(byteOffset + xOffset) * scale;
      positions[i * 3 + 1] = buf.readFloatLE(byteOffset + yOffset) * scale;
      positions[i * 3 + 2] = buf.readFloatLE(byteOffset + zOffset) * scale;
    }
    socket.send(positions);
  });
});
