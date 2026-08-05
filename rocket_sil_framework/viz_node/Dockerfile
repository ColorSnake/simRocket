FROM ros:humble

# Install MCAP storage plugin for Foxglove compatibility
RUN apt-get update && apt-get install -y \
    ros-humble-rosbag2-storage-mcap \
    && rm -rf /var/lib/apt/lists/*
