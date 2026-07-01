FROM ohmnilabsvn/ohmni_ros:ohmni_ros_tbcontrol_0.0.13

ENV OHMNI_WS=/home/ohmni_navigation_lidar/melodic
ENV ROS_DISTRO=melodic

SHELL ["/bin/bash", "-lc"]

RUN mkdir -p /home/ohmni_navigation_lidar

COPY melodic ${OHMNI_WS}
COPY docker/entrypoint.sh /ros_entrypoint.sh
COPY docker/ros_launch.sh /ros_launch.sh

RUN chmod +x /ros_entrypoint.sh /ros_launch.sh && \
    source /opt/ros/melodic/setup.bash && \
    cd "${OHMNI_WS}" && \
    if [ -f src/CMakeLists.txt ]; then catkin_make; fi

WORKDIR ${OHMNI_WS}

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["tail", "-f", "/dev/null"]
