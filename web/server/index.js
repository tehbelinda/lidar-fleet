const http = require('http');
const express = require('express');
const WebSocket = require('ws');

const PORT = process.env.PORT || 3002;

const app = express();
const httpServer = http.Server(app);
const websocketServer = new WebSocket.Server({server: httpServer});

app.use(express.static('../public'));

httpServer.listen(PORT, () => {
  console.log('Server started on', PORT);
});

let lidarSockets = {};
let displaySockets = [];

websocketServer.on('connection', (socket, req) => {
  console.log('Got connection on', req.url);
  if (req.url.startsWith('/lidar')) {
    // Keep track of connected lidar
    const lidarId = req.url.replace('/lidar/', '');
    console.log('Attaching lidar id', lidarId);
    lidarSockets[lidarId] = socket;

    // Re-broadcast latest point cloud data to client displays
    socket.on('message', (data) => {
      for (let displaySocket of displaySockets) {
        displaySocket.send(data);
      }
    });

    // Remove lidar socket on disconnection
    socket.on('close', () => {
      console.log('Removing lidar id', lidarId);
      delete lidarSockets[lidarId];
    });
  } else {
    const index = displaySockets.push(socket) - 1;
    console.log('Attaching client display', index);

    // Remove display socket on disconnection
    socket.on('close', () => {
      console.log('Removing client display', index);
      displaySockets.splice(index, 1);
    });
  }
});
