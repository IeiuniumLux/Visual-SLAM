FROM youyu/ubuntu:16.04
# Or use your own build image
# FROM orb_slam2:build
MAINTAINER Yu You <youyu.youyu@gmail.com>

WORKDIR /opt
RUN cd /opt \
#&& git clone https://github.com/raulmur/ORB_SLAM2.git ORB_SLAM2 \
&& git clone https://github.com/yuyou/ORB_SLAM2.git ORB_SLAM2 \
&& cd ORB_SLAM2 && chmod +x build.sh && sh build.sh
