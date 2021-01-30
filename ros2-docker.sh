#!/bin/bash
DOCKER_IMAGE="masato-ka/ros:foxy"
DOCKER_IMAGE_HOME="/home/ubuntu"
DOCKER_COMMAND="docker run --rm"


while getopts ":hi:gt:" OPT
do
  case $OPT in
    i) OPT_FLAG_i=1;OPT_VALUE_i=$OPTARG ;;
    g) OPT_FLAG_g=1;;
    t) OPT_FLAB_t=1;OPT_VALUE_t=$OPTARG ;;
    h) echo  "Help";;
    :) echo  "[ERROR] Option argument is undefined.";;   #
    \?) echo "[ERROR] Undefined options.";;
  esac
done

# getopts分の引数値移動
shift $(($OPTIND - 1))

# オプションの解析結果を表示
if [[ -n "${OPT_FLAG_i+UNDEF}" ]];then
  echo "custom docker image=${OPT_VALUE_i}"
  DOCKER_IMAGE=$OPT_VALUE_i
fi
if [[ -n "${OPT_FLAG_g+UNDEF}" ]];then
  echo "GUI mode"
  GUI=1
fi
if [[ -n "${OPT_FLAG_t+UNDEF}" ]];then
  echo "GUI mode"
  TARGET_IMAGE_NAME=$OPT_VALUE_t
fi

WORKSPACE_NAME="$1"
command="$2"
ARGS=${@:3:($#-2)}


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
        rosdep install --from-paths src --ignore-src -r -y; \
        colcon build ${args}"
}

#ros-docker [options -i, -g] workspace run [argument ros2 run]
function run () {
  args="$*"
  if [[ -n "${GUI}" ]];then
  # gui run
  $DOCKER_COMMAND -v $PWD/$WORKSPACE_NAME:$DOCKER_IMAGE_HOME/$WORKSPACE_NAME $DOCKER_IMAGE /bin/bash -c \
      "cd ${DOCKER_IMAGE_HOME}/${WORKSPACE_NAME} &&\
       source install/setup.sh &&\
       (xvfb-run --auth-file /tmp/xvfb-run -- ros2 run ${args}) &\
       sleep 5 &&\
       x11vnc -display :99 -auth /tmp/xvfb-run"
  else
  # cui run
  $DOCKER_COMMAND -v $PWD/$WORKSPACE_NAME:$DOCKER_IMAGE_HOME/$WORKSPACE_NAME $DOCKER_IMAGE /bin/bash -c \
      "cd ${DOCKER_IMAGE_HOME}/${WORKSPACE_NAME} &&\
       source install/setup.sh &&\
       ros2 run ${args}"
  fi
}

#eos-docker [options -i, -g] workspace launch [argument ros2 launch]
function launch () {
  args="$*"
  if [[ -n "${GUI}" ]];then
  $DOCKER_COMMAND -p 5900:5900 -v $PWD/$WORKSPACE_NAME:$DOCKER_IMAGE_HOME/$WORKSPACE_NAME $DOCKER_IMAGE /bin/bash -c \
    "cd ${DOCKER_IMAGE_HOME}/${WORKSPACE_NAME} &&
    source install/setup.sh &&\
     (xvfb-run --server-args=\"-screen 0, 1280x1024x24\" --auth-file /tmp/xvfb-run -- ros2 launch ${args}) &\
     sleep 5 &&\
     x11vnc -display :99 -auth /tmp/xvfb-run"
  else
  $DOCKER_COMMAND -v $PWD/$WORKSPACE_NAME:$DOCKER_IMAGE_HOME/$WORKSPACE_NAME $DOCKER_IMAGE /bin/bash -c \
      "cd ${DOCKER_IMAGE_HOME}/${WORKSPACE_NAME} &&\
      source install/setup.sh &&\
      ros2 launch ${args}"
  fi
}

#ros-docker workspace bundle [argument ros2 bundle]
function bundle() {
  args="$*"
  $DOCKER_COMMAND -v $PWD/$WORKSPACE_NAME:$DOCKER_IMAGE_HOME/$WORKSPACE_NAME $DOCKER_IMAGE /bin/bash -c \
  "cd ${DOCKER_IMAGE_HOME}/${WORKSPACE_NAME} &&\
   colcon bundle ${args}"
}

#ros-docker workspace vcs
#TODO rosinstallのファイル名変わる、配置場所は必ずしもワークスペース以下ではない、かつ、クローン先も変わる。
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

#ros-docker workspace rosdep [argument ros2 rosdep]
function rosdep () {
    args="$*"
    tmp_container="TEMP"
    commit_image_name="$TARGET_IMAGE_NAME"
    docker run --name ${tmp_container} -v $PWD/$WORKSPACE_NAME:$DOCKER_IMAGE_HOME/$WORKSPACE_NAME $DOCKER_IMAGE /bin/bash -c \
    "cd ${DOCKER_IMAGE_HOME}/${WORKSPACE_NAME} &&\
    rosdep ${args}"
    docker commit ${tmp_container} ${commit_image_name}
    docker rm ${tmp_container}
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