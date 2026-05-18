FROM osrf/ros:humble-desktop

# 换国内最快源
RUN sed -i "s@http://.*archive.ubuntu.com@https://mirrors.aliyun.com@g" /etc/apt/sources.list && \
    sed -i "s@http://.*security.ubuntu.com@https://mirrors.aliyun.com@g" /etc/apt/sources.list && \
    apt-get update

# 安装依赖
RUN apt-get install -y \
    ros-humble-moveit \
    ros-humble-moveit-ros \
    ros-humble-moveit-planners \
    ros-humble-moveit-simple-controller-manager \
    ros-humble-moveit-configs-utils \
    ros-humble-moveit-ros-visualization \
    ros-humble-moveit-ros-warehouse \
    ros-humble-moveit-setup-assistant \
    ros-humble-moveit-ros-perception \
    ros-humble-moveit-msgs \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-controller-manager \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-robot-state-publisher \
    ros-humble-urdf \
    ros-humble-xacro \
    ros-humble-rviz2 \
    ros-humble-rviz-common \
    ros-humble-rviz-default-plugins \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    python3-serial \
    ros-humble-trajectory-msgs \
    ros-humble-sensor-msgs \
    ros-humble-std-msgs \
    ros-humble-geometry-msgs \
    ros-humble-warehouse-ros \
    ros-humble-warehouse-ros-sqlite \
    git build-essential cmake wget \
    # QT 图形界面 + X11 支持
    python3-pyqt5 \
    libxcb-xinerama0 \
    libqt5gui5 \
    libxkbcommon-x11-0 \
    libgl1-mesa-glx \
    x11-apps \
    # 中文字体
    fonts-wqy-zenhei \
    fonts-wqy-microhei \
    && rm -rf /var/lib/apt/lists/*

ENV QT_QPA_PLATFORM=xcb
ENV LANG=zh_CN.UTF-8
ENV LANGUAGE=zh_CN:zh
ENV LC_ALL=zh_CN.UTF-8

WORKDIR /ros2_ws

# NLopt
RUN cd /tmp && \
    wget http://mirrors.cqupt.edu.cn/deepin/beige/pool/main/n/nlopt/nlopt_2.7.1.orig.tar.gz -O nlopt-2.7.1.tar.gz && \
    tar -xzf nlopt-2.7.1.tar.gz && \
    cd nlopt-2.7.1 && mkdir build && cd build && \
    cmake -DCMAKE_INSTALL_PREFIX=/usr/local .. && \
    make -j$(nproc) && make install && \
    ldconfig && \
    rm -rf /tmp/nlopt-2.7.1.tar.gz /tmp/nlopt-2.7.1

COPY . .

RUN . /opt/ros/humble/setup.sh && \
    colcon build

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

CMD ["bash"]
