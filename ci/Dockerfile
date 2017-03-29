FROM ubuntu:latest
RUN apt-get -y update && apt-get install -y \
    cmake \
    doxygen \
    g++ \
    git \
    libpcl-dev \
    libqglviewer-dev \
    libsuitesparse-dev \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /source
RUN git clone https://github.com/RainerKuemmerle/g2o.git

WORKDIR g2o/build
RUN cmake -DCMAKE_BUILD_TYPE=Release .. \
 && make install

WORKDIR /
RUN rm -rf /source

