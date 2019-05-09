FROM ubuntu:18.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
 && apt-get upgrade -y \
 && apt-get install -y wget cmake build-essential vim libeigen3-dev libflann-dev \
    libpcap0.8-dev libvtk6-dev libboost-all-dev \
    unzip libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev

RUN wget -qO - https://deb.nodesource.com/setup_11.x | bash - \
 && apt-get -y install nodejs

RUN mkdir /pcl
WORKDIR /pcl

RUN wget https://github.com/PointCloudLibrary/pcl/archive/pcl-1.9.1.tar.gz
RUN tar -xvzf pcl-1.9.1.tar.gz
WORKDIR pcl-pcl-1.9.1
WORKDIR build
RUN cmake -DCMAKE_BUILD_TYPE=Release ..
RUN make -j8
RUN make -j8 install

WORKDIR /
RUN rm -rf pcl

# Install OpenCV
WORKDIR /tmp
RUN wget https://github.com/opencv/opencv/archive/3.4.6.zip && unzip 3.4.6.zip && rm 3.4.6.zip
RUN mkdir opencv-3.4.6/build && cd opencv-3.4.6/build \
  && cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local .. \
  && make -j8 \
  && make install

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
