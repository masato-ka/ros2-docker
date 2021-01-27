
WORKSPACE_NAME="$1"
command="$2"
ARGS=${@:3:($#-2)}
#BUILD_TYPE="ament_python"
#NODE_NAME="robo_node"
#PKG_NAME="robo_pkg"
DOCKER_IMAGE="masato-ka/ros:foxy"
#DOCKER_IMAGE="masato-ka/ros-foxy-devel"
DOCKER_IMAGE_HOME="/home/ubuntu"
DOCKER_COMMAND="docker run --rm"




#ros-docker workspace create [argument ros2 create]
function create (){
  args="$*"
  echo $args
  $DOCKER_COMMAND  -v $PWD/$WORKSPACE_NAME/src:$DOCKER_IMAGE_HOME/$WORKSPACE_NAME/src $DOCKER_IMAGE /bin/bash -c \
    "cd ${DOCKER_IMAGE_HOME}/${WORKSPACE_NAME}/src && \
     ros2 pkg create ${args}"
}

#ros-docker workspace build [argument ros2 build]
function build () {
    args="$*"
    $DOCKER_COMMAND -v $PWD/$WORKSPACE_NAME:$DOCKER_IMAGE_HOME/$WORKSPACE_NAME -m 8192mb $DOCKER_IMAGE /bin/bash -c \
        "cd ${DOCKER_IMAGE_HOME}/${WORKSPACE_NAME}; \
        colcon build ${args}"
}

#ros-docker workspace run [argument ros2 run]
function run () {
  args="$*"
  $DOCKER_COMMAND -v $PWD/$WORKSPACE_NAME:$DOCKER_IMAGE_HOME/$WORKSPACE_NAME $DOCKER_IMAGE /bin/bash -c \
      "cd ${DOCKER_IMAGE_HOME}/${WORKSPACE_NAME} &&\
       source install/setup.sh &&\
       ros2 run ${args}"
}

#eos-docker workspace launch [argument ros2 launch]
function launch () {
  args="$*"
  $DOCKER_COMMAND -v $PWD/$WORKSPACE_NAME:$DOCKER_IMAGE_HOME/$WORKSPACE_NAME $DOCKER_IMAGE /bin/bash -c \
      "cd ${DOCKER_IMAGE_HOME}/${WORKSPACE_NAME} &&\
      source install/setup.sh &&\
      ros2 launch ${args}"
}

#ros-docker workspace bundle [argument ros2 bundle]
function bundle() {
  args="$*"
  $DOCKER_COMMAND -v $PWD/$WORKSPACE_NAME:$DOCKER_IMAGE_HOME/$WORKSPACE_NAME $DOCKER_IMAGE /bin/bash -c \
  "cd ${DOCKER_IMAGE_HOME}/${WORKSPACE_NAME} &&\
   colcon bundle ${args}"
}

#ros-docker workspace vcs
function vcs () {
  $DOCKER_COMMAND -v $PWD/$WORKSPACE_NAME:$DOCKER_IMAGE_HOME/$WORKSPACE_NAME $DOCKER_IMAGE /bin/bash -c \
    "cd ${DOCKER_IMAGE_HOME}/${WORKSPACE_NAME} &&\
     vcs import < .rosinstall"
}

#ros-docker workspace rviz
function rviz () {
  # Xvfb+VNC Trick See in https://qiita.com/yuyakato/items/55762a92f03506f583ee
    args="$*"
    $DOCKER_COMMAND -p 5900:5900 -v $PWD/$WORKSPACE_NAME:$DOCKER_IMAGE_HOME/$WORKSPACE_NAME $DOCKER_IMAGE /bin/bash -c \
    "cd ${DOCKER_IMAGE_HOME}/${WORKSPACE_NAME} &&\
     export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:/opt/ros/foxy/ &&\
     (xvfb-run --auth-file /tmp/xvfb-run -- rviz2 ${args}) & \
     x11vnc -display :99 -auth /tmp/xvfb-run"
}

#ros-docker workspace gazebo
function gazebo () {
  # Xvfb+VNC Trick See in https://qiita.com/yuyakato/items/55762a92f03506f583ee
    args="$*"
    $DOCKER_COMMAND -p 5900:5900 -v $PWD/$WORKSPACE_NAME:$DOCKER_IMAGE_HOME/$WORKSPACE_NAME $DOCKER_IMAGE /bin/bash -c \
    "cd ${DOCKER_IMAGE_HOME}/${WORKSPACE_NAME} &&\
     (xvfb-run -s '-screen 0 1280x720x24' --auth-file /tmp/xvfb-run -- gazebo ${args}) & \
     x11vnc -display :99 -auth /tmp/xvfb-run"
}

#ros-docker workspace gazebo launch
function gazebo_launch () {
  # Xvfb+VNC Trick See in https://qiita.com/yuyakato/items/55762a92f03506f583ee
    args="$*"
    $DOCKER_COMMAND -p 5900:5900 -v $PWD/$WORKSPACE_NAME:$DOCKER_IMAGE_HOME/$WORKSPACE_NAME $DOCKER_IMAGE /bin/bash -c \
    "cd ${DOCKER_IMAGE_HOME}/${WORKSPACE_NAME} &&\
     (xvfb-run -s '-screen 0 1280x720x24' --auth-file /tmp/xvfb-run -- ros2 launch ${args}) & \
     x11vnc -display :99 -auth /tmp/xvfb-run"
}


#ros-docker workspace rosdep [argument ros2 rosdep]
function rosdep () {
    args="$*"
    $DOCKER_COMMAND -v $PWD/$WORKSPACE_NAME:$DOCKER_IMAGE_HOME/$WORKSPACE_NAME $DOCKER_IMAGE /bin/bash -c \
    "cd ${DOCKER_IMAGE_HOME}/${WORKSPACE_NAME} &&\
    rosdep ${rgs}"
}

case $command in
    create)
        echo "Start create ros package."
        create $ARGS
    ;;
    build)
        echo "Start build"
        build $ARGS
    ;;
    run)
        echo "Start run"
        run $ARGS
    ;;
    launch)
        echo "Start launch"
        launch $ARGS
    ;;
    bundle)
        echo "Start bundle"
        bundle $ARGS
    ;;
    rviz2)
        echo "Start rviz2"
        rviz $ARGS
    ;;
    gazebo)
        echo "Start Gazebo"
        gazebo $ARGS
    ;;
    gazebo_launch)
        echo "Start gazebo launch"
        gazebo $ARGS
    ;;
    vcs)
        echo "Start vcs"
        vcs
    ;;
    rosdep)
        echo "Start rosdep"
        rosdep $ARGS
    ;;
    *)
    echo "No such command. choose from (create build run launch bundle)."
    ;;
esac