# PhaseSpace LED -> RViz image (ROS 2 Humble + rviz2).
# Build from the repo root: docker build -f docker/Dockerfile -t phasespace-rviz .
FROM osrf/ros:humble-desktop

ENV PYTHONDONTWRITEBYTECODE=1 \
    QT_X11_NO_MITSHM=1

WORKDIR /ws
COPY Ros_2/phasespace_node_ros2.py Ros_2/
COPY docker/phasespace_rviz.py docker/phasespace.rviz docker/

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["python3", "docker/phasespace_rviz.py"]
