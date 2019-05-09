const http = require('http');
const express = require('express');
const WebSocket = require('ws');

const { processPointCloud } = require('./build/Release/addon');

const PORT = process.env.PORT || 3002;

const app = express();
const httpServer = http.Server(app);
const websocketServer = new WebSocket.Server({server: httpServer});

app.use(express.static('../public'));

httpServer.listen(PORT, () => {
  console.log('Server started on', PORT);
});

const lidarSockets = {};
const displaySockets = {};
const infoSockets = [];

const sendLidarInfo = (socket) => {
  if (socket.readyState != WebSocket.OPEN) {
    console.warn('Client not ready for lidar info', socket.readyState);
    return;
  }
  socket.send(JSON.stringify({
    type: 'lidars',
    lidars: Object.keys(lidarSockets)
  }));
};

websocketServer.on('connection', (socket, req) => {
  console.log('Got connection on', req.url);
  if (req.url.startsWith('/ws/lidar')) {
    // Keep track of connected lidar
    const lidarId = req.url.replace('/ws/lidar/', '');
    console.log('Attaching lidar id', lidarId);
    lidarSockets[lidarId] = socket;

    let pointData = null;
    socket.on('message', (data) => {
      // Just stash the message and be ready for the next one
      pointData = data;
    });

    // Re-broadcast latest point cloud data to client displays at a regular interval
    const intervalId = setInterval(() => {
      if (!pointData) {
        return;
      }

      processPointCloud(pointData);

      if (!displaySockets[lidarId]) {
        return;
      }

      for (let displaySocket of displaySockets[lidarId]) {
        try {
          displaySocket.send(pointData);
        } catch(error) {
          console.error(error);
        }
      }
    }, 100);

    // Remove lidar socket on disconnection
    socket.on('close', () => {
      console.log('Removing lidar id', lidarId);
      clearInterval(intervalId);
      pointData = null;
      delete lidarSockets[lidarId];
      // Updates clients about available lidars
      for (let infoSocket of infoSockets) {
        sendLidarInfo(infoSocket);
      }
    });

    // Updates clients about available lidars
    for (let infoSocket of infoSockets) {
      sendLidarInfo(infoSocket);
    }
  } else if (req.url.startsWith('/ws/view')) {
    // Keep track of which lidar was requested for viewing
    const lidarId = req.url.replace('/ws/view/', '');
    if (!displaySockets[lidarId]) {
      displaySockets[lidarId] = [];
    }
    const index = displaySockets[lidarId].push(socket) - 1;
    console.log(`Attaching view ${index} for ${lidarId}`);

    // Remove display socket on disconnection
    socket.on('close', () => {
      console.log(`Removing client display ${index} for ${lidarId}`);
      displaySockets[lidarId].splice(index, 1);
    });
  } else if (req.url.startsWith('/ws')) {
    const index = infoSockets.push(socket) - 1;
    console.log(`Attaching info ${index}`);

    // Remove info socket on disconnection
    socket.on('close', () => {
      console.log(`Removing info ${index}`);
      infoSockets.splice(index, 1);
    });

    // Alert new connections of available lidars
    sendLidarInfo(socket);
  }
});
