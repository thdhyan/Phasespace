# PhaseSpace -> ROS 2 Jazzy image (phasespace_ros2 package + rviz2).
# Build from the repo root: docker build -f docker/Dockerfile -t phasespace-rviz .
FROM osrf/ros:jazzy-desktop

ENV PYTHONDONTWRITEBYTECODE=1 \
    QT_X11_NO_MITSHM=1

WORKDIR /ws
COPY Ros_2/phasespace_node_ros2.py src/Ros_2/
COPY phasespace_ros2 src/phasespace_ros2
RUN . /opt/ros/jazzy/setup.sh && colcon build --base-paths src && rm -rf build log

RUN echo "source /ws/install/setup.bash" >> /root/.bashrc
ENTRYPOINT ["bash", "-c", "source /ws/install/setup.bash && exec \"$@\"", "--"]
CMD ["ros2", "launch", "phasespace_ros2", "phasespace.launch.py", "rviz:=false"]
