FROM --platform=linux/arm64 ubuntu:22.04

# Avoid interactive prompts
ENV DEBIAN_FRONTEND=noninteractive

# Install prerequisites
RUN apt-get update && apt-get install -y \
    wget \
    lsb-release \
    gnupg \
    curl \
    sudo \
    software-properties-common \
    python3 \
    python3-pip \
    mesa-utils \
    xvfb \
    && rm -rf /var/lib/apt/lists/*

# Add Gazebo repository
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Update and install Gazebo (try different versions)
RUN apt-get update && apt-get install -y \
    gz-garden || apt-get install -y ignition-gazebo6 || apt-get install -y gazebo11 \
    && rm -rf /var/lib/apt/lists/*

# Install web server dependencies
RUN pip3 install websockets==11.0.3 asyncio

# Install VNC server and GUI components
RUN apt-get update && apt-get install -y \
    x11vnc \
    xvfb \
    fluxbox \
    websockify \
    novnc \
    xterm \
    firefox \
    mesa-utils \
    x11-apps \
    dbus-x11 \
    xfonts-base \
    xfonts-75dpi \
    xfonts-100dpi \
    net-tools \
    glx-utils \
    mesa-utils-extra \
    libegl1-mesa \
    libgl1-mesa-glx \
    libglu1-mesa \
    && rm -rf /var/lib/apt/lists/*

# Create VNC password file (optional, for basic security)
RUN mkdir -p /root/.vnc && \
    x11vnc -storepasswd gazebo123 /root/.vnc/passwd

# Create working directory
WORKDIR /gazebo

# Copy web server script
COPY web_server.py /gazebo/
COPY start.sh /gazebo/
COPY test_vnc.sh /gazebo/
COPY test_desktop.sh /gazebo/

# Make scripts executable
RUN chmod +x /gazebo/start.sh /gazebo/test_vnc.sh /gazebo/test_desktop.sh

# Expose ports
EXPOSE 8080 7681 11345 6080 5900

# Set environment variables
ENV GZ_PARTITION=gz_web
ENV GZ_IP=0.0.0.0

# Start command
CMD ["/gazebo/start.sh"]