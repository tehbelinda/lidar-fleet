FROM node:carbon

# Install dependencies
WORKDIR /usr/src
COPY app/package*.json ./app/
COPY server/package*.json ./server/
WORKDIR /usr/src/app
RUN npm install
WORKDIR /usr/src/server
RUN npm install

# Copy rest of files
WORKDIR /usr/src
COPY public public
COPY app/. app/.
COPY server/. server/.

# Build app
WORKDIR /usr/src/app
RUN npm run build

# Build and start server
WORKDIR /usr/src/server
RUN npm run compile
EXPOSE 3002
CMD [ "npm", "start" ]
