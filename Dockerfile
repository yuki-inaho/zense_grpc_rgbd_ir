FROM ubuntu:18.04

WORKDIR /app

ENV DEBIAN_FRONTEND=noninteractive

RUN sed -i -r 's|(archive\|security)\.ubuntu\.com/|ftp.jaist.ac.jp/pub/Linux/|' /etc/apt/sources.list

RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y build-essential apt-utils ca-certificates \
    cmake git libgtk2.0-dev pkg-config \
    libswscale-dev wget autoconf automake pkg-config unzip curl \
    python-dev python-pip python-numpy python3 python3-pip python3-dev python3-distutils \
    ffmpeg libavcodec-dev libavformat-dev \
    libjpeg-dev libdc1394-22-dev libv4l-dev \
    python-opencv libopencv-dev python-pycurl \
    libpng-dev libswscale-dev libtbb2 libtbb-dev libtiff-dev \ 
    libatlas-base-dev gfortran webp qt5-default libvtk6-dev zlib1g-dev \
    libhdf5-100 libhdf5-cpp-100 hdf5-tools hdf5-helpers libhdf5-dev libhdf5-doc \
    libxml2-dev libxslt1-dev zlib1g-dev software-properties-common emacs &&\
    rm -rf /var/lib/apt/lists/*

RUN rm -f /usr/bin/python3 && ln -s /usr/bin/python3.6 /usr/bin/python3 && \
    rm -f /usr/bin/python && ln -s /usr/bin/python3.6 /usr/bin/python && \
    rm -f /usr/local/bin/pip && ln -s /usr/local/bin/pip3.6 /usr/local/bin/pip && \
    rm -f /usr/local/bin/pip3 && ln -s /usr/local/bin/pip3.6 /usr/local/bin/pip3

WORKDIR /app
ENV OPENCV_VERSION="3.4.3"
RUN mkdir -p /app/opencv-$OPENCV_VERSION/build
RUN curl -L https://github.com/opencv/opencv/archive/$OPENCV_VERSION.tar.gz | tar xz && curl -L https://github.com/opencv/opencv_contrib/archive/$OPENCV_VERSION.tar.gz | tar xz

WORKDIR /app/opencv-$OPENCV_VERSION/build
RUN cmake -DWITH_TBB=ON \
    -DOPENCV_EXTRA_MODULES_PATH=../../opencv_contrib-$OPENCV_VERSION/modules \
    -DBUILD_TESTS=OFF \
    -DBUILD_PERF_TESTS=OFF \
    -DCMAKE_BUILD_TYPE=RELEASE \
    -DCMAKE_INSTALL_PREFIX=/usr/local \
    -DPYTHON_EXECUTABLE=$(which python) \
    -DPYTHON_INCLUDE_DIR=$(python -c "from distutils.sysconfig import get_python_inc; print(get_python_inc())") \
    -DPYTHON_PACKAGES_PATH=$(python -c "from distutils.sysconfig import get_python_lib; print(get_python_lib())") \
     .. 

RUN make -j4 install && make clean && ldconfig

RUN rm -rf /app/opencv-${OPENCV_VERSION}

RUN python -m pip install grpcio && \
    python -m pip install grpcio-tools

# install protobuf first, then grpc
WORKDIR /app
ENV GRPC_RELEASE_TAG v1.28.1
RUN git clone -b ${GRPC_RELEASE_TAG} https://github.com/grpc/grpc && \
        cd grpc && git submodule update --init

# Install abseil
RUN mkdir -p grpc/third_party/abseil-cpp/build && \
    cd grpc/third_party/abseil-cpp/build && \
    cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_POSITION_INDEPENDENT_CODE=TRUE .. && \
    make -j4 install

# Install cares
RUN mkdir -p grpc/third_party/cares/cares/build && \
    cd grpc/third_party/cares/cares/build && \
    cmake -DCMAKE_BUILD_TYPE=Release .. && \
    make -j4 install

# Install zlib
RUN mkdir -p grpc/third_party/zlib/build && \
    cd grpc/third_party/zlib/build && \
    cmake -DCMAKE_BUILD_TYPE=Release .. && \
    make -j4 install

# Install protobuf
RUN echo "--- installing protobuf ---" && \
    cd /app/grpc/third_party/protobuf && \
    cmake ./cmake -DCMAKE_BUILD_TYPE=Release -Dprotobuf_BUILD_TESTS=OFF && \
    make -j4 && make install && make clean && ldconfig


RUN echo "--- installing grpc ---" && \
    mkdir -p /app/grpc/cmake/build && \
    cd /app/grpc/cmake/build && \
    cmake ../.. -DCMAKE_BUILD_TYPE=Release \
                -DCMAKE_INSTALL_PREFIX=/usr/local/grpc \
                -DgRPC_INSTALL=ON \
                -DgRPC_BUILD_TESTS=OFF \
                -DgRPC_ABSL_PROVIDER=package \
                -DgRPC_CARES_PROVIDER=package \
                -DgRPC_PROTOBUF_PROVIDER=package \
                -DgRPC_SSL_PROVIDER=package \
                -DgRPC_ZLIB_PROVIDER=package && \
    make -j4 && make install && make clean && ldconfig

RUN echo "--- installing zense sdk ---"
COPY . /app
RUN mkdir -p /etc/udev/rules.d
RUN ./install_zense_sdk.sh

ENV PKG_CONFIG_PATH /usr/local/grpc/lib/pkgconfig:$PKG_CONFIG_PATH
ENV PATH /usr/local/grpc/bin:$PATH

RUN apt-get install -y python3-tk tk-dev

ENV LC_ALL=C.UTF-8
ENV LANG=C.UTF-8

RUN ./setup.sh && \
    cd /app/scripts && python gen_protobuf_codes.py

CMD [ "/bin/bash" ]
