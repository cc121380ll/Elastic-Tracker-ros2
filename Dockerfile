FROM osrf/ros:humble-desktop-jammy
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN curl -fL --retry 3 https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb -o /tmp/cuda-keyring.deb \
 && dpkg -i /tmp/cuda-keyring.deb \
 && apt-get update \
 && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends cuda-toolkit-11-8 ros-humble-pcl-ros ros-humble-pcl-conversions \
 && rm /tmp/cuda-keyring.deb
ENV CUDA_HOME=/usr/local/cuda-11.8
ENV PATH=/usr/local/cuda-11.8/bin:${PATH}
ENV ROS_DOMAIN_ID=86
ENV ROS_LOCALHOST_ONLY=1
WORKDIR /work/elastic-tracker
CMD ["sleep", "infinity"]
