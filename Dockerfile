FROM ros:noetic

# Install some basic dependencies
RUN apt-get update && apt-get -y upgrade && apt-get -y install \
  curl ssh python3-pip \
  && rm -rf /var/lib/apt/lists/*

# Set root password
RUN echo 'root:root' | chpasswd

# Permit SSH root login
RUN sed -i 's/#*PermitRootLogin prohibit-password/PermitRootLogin yes/g' /etc/ssh/sshd_config

# Install catkin-tools
RUN apt-get update && apt-get install -y python3-catkin-tools tmux \
  && rm -rf /var/lib/apt/lists/*

# Git requirements
RUN apt-get install -y git
RUN git config --global user.name "Mattia Catellani"
RUN git config --global user.email "215802@studenti.unimore.it"
RUN git config --global color.ui true


# Get packages for building
WORKDIR /catkin_ws
RUN cd ~/catkin_ws ; mkdir src
RUN cd ~/catkin_ws/src/ ; git clone https://github.com/MatCat960/pf_coverage.git
RUN cd ~/catkin_ws/src/pf_coverage ; git checkout docker-devel
RUN cd ~/catkin_ws/src/ ; git clone https://github.com/MatCat960/particle_filter.git
RUN cd ~/catkin_ws/src/particle_filter ; git checkout ros1-noetic
RUN cd ~/catkin_ws/src/ ; git clone https://github.com/ARSControl/gaussian_mixture_model.git


# Build the workspace
RUN apt-get update \
  && rosdep update \
  && rosdep install --from-paths src -iy \
  && rm -rf /var/lib/apt/lists/*
RUN catkin config --extend /opt/ros/noetic && catkin build --no-status

# Automatically source the workspace when starting a bash session
RUN echo "source /catkin_ws/devel/setup.bash" >> /etc/bash.bashrc