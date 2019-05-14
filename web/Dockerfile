FROM ubuntu:18.04

RUN apt-get update
RUN apt-get upgrade -y

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get install -y wget cmake build-essential vim libeigen3-dev libflann-dev \
  libpcap0.8-dev libvtk6-dev libboost-all-dev less

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
