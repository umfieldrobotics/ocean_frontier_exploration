# Dockerfile for creating a development container with GPU support and user-space tooling
# This container is designed for robotics/computer vision development with NVIDIA GPU access

# Base image argument - typically an NVIDIA CUDA image for GPU support
#ARG BASE_IMAGE=nvidia/cuda:13.1.0-runtime-ubuntu24.04 
ARG BASE_IMAGE=nvidia/cuda:13.1.0-runtime-ubuntu24.04
#nvcr.io/nvidia/isaac-sim:5.1.0
FROM ${BASE_IMAGE}

# User configuration arguments - these are passed from build.sh to match host user
ARG USER_NAME=<whatever>
ARG USER_ID=1000

# Prevent anything requiring user input during package installation
ENV DEBIAN_FRONTEND=noninteractive
ENV TERM=linux
ENV OMNI_KIT_ACCEPT_EULA=YES


# Set timezone to avoid prompts during package installation
ENV TZ=America
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# Install basic development and utility packages
RUN apt-get -y update \
    && apt-get -y install \
      python3-pip \
      sudo \
      vim \
      wget \
      curl \
      software-properties-common \
      doxygen \
      git \
      git-lfs \ 
    && rm -rf /var/lib/apt/lists/*                                                  # Clean up apt cache to reduce image size

# Install computer vision and graphics libraries (OpenGL, rendering, 3D processing)
RUN apt-get -y update \
    && apt-get -y install \
        libglew-dev \
        libassimp-dev \
        libboost-all-dev \
        libgtk-3-dev \
        libglfw3-dev \
        libavdevice-dev \
        libavcodec-dev \
        libeigen3-dev \
        libxxf86vm-dev \
        libembree-dev \
    && rm -rf /var/lib/apt/lists/*                                                  # Clean up apt cache

# Install build tools
RUN apt-get -y update \
    && apt-get -y install \ 
        cmake \
    && rm -rf /var/lib/apt/lists/*                                                  # Clean up apt cache
 
# Create a non-root user matching the host user ID for seamless file permissions
# This prevents permission issues when accessing mounted volumes
RUN useradd -m -l -u ${USER_ID} -s /bin/bash ${USER_NAME} \
    && usermod -aG video ${USER_NAME} \
    && export PATH=$PATH:/home/${USER_NAME}/.local/bin

# Grant passwordless sudo access to the user for convenience
RUN echo "${USER_NAME} ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers

# Switch to non-root user for all subsequent commands and container runtime
USER ${USER_NAME}
WORKDIR /home/${USER_NAME}

#WORKDIR /home/${USER_NAME}/sonar_camera_fusion_proj/
#Install IsaacSim & Oceansim
RUN git clone https://github.com/isaac-sim/IsaacSim.git isaacsim \
    && cd isaacsim \
    && git lfs install \
    && git lfs pull
RUN sudo apt-get -y update \
    && sudo apt-get install -y software-properties-common \
    && sudo apt-get -y update \
    && sudo add-apt-repository universe \
    && sudo apt-get -y update \
    && sudo apt-get -y install build-essential \
    && sudo apt-get -y install gcc-11 g++-11
RUN sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-11 200
RUN sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-11 200
ENV EULA_STATUS=1
RUN cd isaacsim \
    && touch .eula_accepted \
    && ./build.sh
RUN sudo apt-get install -y \
    libxt6 \
    libx11-6 \
    libxext6
#ARG CACHEBUST=1
RUN cd isaacsim/_build/linux-x86_64/release && mkdir extsUser && cd extsUser \
    && git clone https://github.com/umfieldrobotics/OceanSim.git \
    && cd OceanSim \
    && pip install --break-system-packages gdown \
    && export PATH="$HOME/.local/bin:$PATH" \
    && mkdir OceanSim_assets \
   && gdown --folder 1qg4-Y_GMiybnLc1BFjx0DsWfR0AgeZzA -O OceanSim_assets \
   #&& cd isaacsim/_build/linux-x86_64/release/extsUser \
   && python3 config/register_asset_path.py OceanSim_assets/
#ARG CACHEBUST=1
#COPY assets /home/${USER_NAME}/isaacsim/_build/linux-x86_64/release/extsUser/OceanSim/OceanSim_assets
#ARG CACHEBUST=1
# RUN mkdir OceanSim_assets \
#     && cd /home/${USER_NAME}/isaacsim/_build/linux-x86_64/release/extsUser/OceanSim/ \
#     && python3 config/register_asset_path.py OceanSim_assets/
# ENV DEBIAN_FRONTEND=noninteractive
RUN sudo apt install software-properties-common \
    && sudo add-apt-repository -y universe \
    && sudo apt update \
    && sudo apt install curl -y \
    && ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | \
        grep -F "tag_name" | \
        awk -F\" '{print $4}') \
    && curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb" \
    && sudo dpkg -i /tmp/ros2-apt-source.deb \
    && sudo apt update \
    && sudo apt upgrade -y \
    && sudo apt install ros-jazzy-desktop -y \
    && sudo apt install ros-dev-tools -y \
    && echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc \
    && pip install --break-system-packages opencv-python

RUN ls
ARG CACHEBUST=1
#https://drive.google.com/drive/folders/1qg4-Y_GMiybnLc1BFjx0DsWfR0AgeZzA
# Ensure the user owns their home directory/path/to/OceanSim_assets
#RUN sudo chown -R ${USER_NAME} /home/${USER_NAME}
ARG CACHEBUST=1
# Copy and set up the entrypoint script that runs when the container starts
#used for entrypoint
ARG SCRIPT_DIR
ENV SCRIPT_DIR=${SCRIPT_DIR}
RUN echo "export SCRIPT_DIR=${SCRIPT_DIR}" >> ~/.bashrc
COPY ./entrypoint.sh /entrypoint.sh
RUN sudo chmod +x /entrypoint.sh
USER root
ENTRYPOINT ["/entrypoint.sh"]
USER ${USER_NAME}
CMD ["/bin/bash", "-i"]
