FROM ubuntu:16.04
LABEL Description="Docker image with development environment for PLA"


# first install dependencies
RUN apt update && apt install -y --no-install-recommends git libboost-all-dev \
libyaml-cpp-dev make automake autoconf g++ default-jdk ant wget libtool gdb curl parallel evince

# Cleanup
RUN apt-get clean && \
  rm -rf /var/lib/apt
