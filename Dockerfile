FROM osrf/ros:jazzy-desktop

WORKDIR /workspace/

RUN <<EOF
apt-get update
apt-get install -y gosu
apt-get clean && rm -rf /var/lib/apt/lists/*
EOF

RUN apt-get update && apt-get install -y python3-serial

# Add entrypoint script
COPY entrypoint.sh  /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

# Edit /etc/skel/.bashrc (aka default .bashrc)
RUN <<EOF
cat <<BASHRC >> /etc/skel/.bashrc
# Helpful aliases
alias cb='cd /workspace/ && colcon build --symlink-install'
alias cs='cd /workspace/ && source install/setup.bash'
alias cbs='cb && cs'

# Source setup bash
source /opt/ros/jazzy/setup.bash
BASHRC
EOF

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]

CMD ["bash"]
