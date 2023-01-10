#!/bin/bash
set -e

## Check if the named volume has already been attached to a container
# if it has already, just source the already built workspace
# if not (first time run), then setup the workspace from scratch
VOLUME_EXEC_COUNTER="$HOME/ros_workspace/volume_exec_counter"

if test -f "$VOLUME_EXEC_COUNTER"; then # if workspace already built
    counter=$(cat $VOLUME_EXEC_COUNTER)
    ((counter++))
    # Avoid the user's misunderstanding that a new image build => a new volume
    echo "volume has been attached $counter times since its creation"
    echo $counter > $VOLUME_EXEC_COUNTER
    echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
    echo "source ~/ros_workspace/devel/setup.bash --extend" >> ~/.bashrc

  else  # setup the catkin workspace for the first time
    echo "setting up catkin workspace for the first time"
    cd ~/ros_workspace
    source /opt/ros/noetic/setup.bash
    # Build artifacts are saved in the workspace volume
    catkin config --init
    catkin build
    source devel/setup.bash --extend

    echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
    echo "source ~/ros_workspace/devel/setup.bash --extend" >> ~/.bashrc
    # mark the volume as not empty/already setup
    echo 1 > $VOLUME_EXEC_COUNTER
fi

# Convenience aliases (add your own to learn from each other!)
echo "alias cc='cd ~/ros_workspace/; catkin build'" >> ~/.bashrc
echo "alias cx='catkin build --no-deps'" >> ~/.bashrc  # usage: cx package_name  <- when you are iterating on a single package and know that no other dependency changed
echo "alias ll='ls -lath'" >> ~/.bashrc
echo "alias kk=clear" >> ~/.bashrc
echo "alias sourceb='source ~/.bashrc'" >> ~/.bashrc
echo "alias jj='jupyter notebook'" >> ~/.bashrc
echo "alias tt='tmux new'" >> ~/.bashrc
echo "alias rc='roscore &'" >> ~/.bashrc
echo "alias ts='rosrun turtlesim turtlesim_node'" >> ~/.bashrc
echo "alias ww='cd /artefacts-client && pip install -e . && cd'" >> ~/.bashrc
echo "alias cdw='cd ~/ros_workspace/src/turtle_odometry/test'" >> ~/.bashrc

# Setup rqt and plotjuggler for high DPI screens
echo "alias rqt_graph='QT_SCALE_FACTOR=1.5 rqt_graph'" >> ~/.bashrc
echo "alias pj='rosrun plotjuggler plotjuggler'" >> ~/.bashrc

# Setup for tmux
echo "set-option -g default-command \"exec /bin/bash\"" > ~/.tmux.conf
{
  echo "# if running bash"
  echo "  if [ -n \"$BASH_VERSION\" ]; then"
  echo "    # include .bashrc if it exists"
  echo "    if [ -f \"$HOME/.bashrc\" ]; then"
  echo "    . \"$HOME/.bashrc\""
  echo "    fi"
  echo "fi"
} >> ~/.profile

exec "$@"
