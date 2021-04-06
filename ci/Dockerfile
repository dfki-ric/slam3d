FROM ubuntu:latest
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get -y update && apt-get install -y \
    cmake \
    doxygen \
    g++ \
    git \
    libpcl-dev \
    libsuitesparse-dev \
    libgdal-dev\
 && rm -rf /var/lib/apt/lists/*

WORKDIR /source
RUN git clone https://github.com/RainerKuemmerle/g2o.git

WORKDIR g2o/build
RUN cmake -DCMAKE_BUILD_TYPE=Release .. \
 && make install

WORKDIR /source
RUN git clone https://github.com/ethz-asl/libnabo.git

WORKDIR libnabo/build
RUN cmake -DCMAKE_BUILD_TYPE=Release .. \
 && make install

WORKDIR /source
RUN git clone https://github.com/ethz-asl/libpointmatcher.git

WORKDIR libpointmatcher/build
RUN cmake -DCMAKE_BUILD_TYPE=Release .. \
 && make install

WORKDIR /
RUN rm -rf /source

