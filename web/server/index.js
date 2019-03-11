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
let displaySockets = {};

websocketServer.on('connection', (socket, req) => {
  console.log('Got connection on', req.url);
  if (req.url.startsWith('/lidar')) {
    // Keep track of connected lidar
    const lidarId = req.url.replace('/lidar/', '');
    console.log('Attaching lidar id', lidarId);
    lidarSockets[lidarId] = socket;

    // Re-broadcast latest point cloud data to client displays
    socket.on('message', (data) => {
      if (!displaySockets[lidarId]) {
        return;
      }

      for (let displaySocket of displaySockets[lidarId]) {
        try {
          displaySocket.send(data);
        } catch(error) {
          console.error(error);
        }
      }
    });

    // Remove lidar socket on disconnection
    socket.on('close', () => {
      console.log('Removing lidar id', lidarId);
      delete lidarSockets[lidarId];
    });
  } else if (req.url.startsWith('/view')) {
    // Keep track of which lidar was requested for viewing
    const lidarId = req.url.replace('/view/', '');
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
  }
});
