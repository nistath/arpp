FROM osrf/ros:foxy-desktop

# Use GCC 10 for C++20 support
RUN apt-get update -yq \
 && apt-get install -yq gcc-10 g++-10 \
 && ln -sfn /usr/bin/cpp-10 /usr/bin/cpp \
 && ln -sfn /usr/bin/g++-10 /usr/bin/g++ \
 && ln -sfn /usr/bin/gcc-10 /usr/bin/gcc
