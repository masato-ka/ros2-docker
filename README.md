# ros2-docker

## 1. Overview

You can develop ros2 application without native ros2 environment. This is support script for easily ros2 with docker development.

## 2. Requirement

* docker
* macOS or Linux

## 3. Install

Do don't need install. only build docker image for ros2.

```
$git clone
$cd ros2-docker/app_build_container
$docker build -t <Own image name> .
$cd ../
$sudo chmod +x ros2-docker.sh
$exprot PATH=$PATH:$PWD/ros2-docker.sh
```

## 4. Usage

### Create ros2 package

```
$mkdir -p workspace/src
$ros2-docker.sh workspace create <pkg_name> --build-type ament_cmake --node-name own_node
```

### Build package

```
$ros2-docker.sh workspace build
```

### Run node

```
$ros2-docker.sh workspace run <pkg_name> <node_name>
```

### Launch

```
$ros2-docker.sh workspace launch <pkg_name> <node_name>
If you need gazebo GUI
$ros2-docker.sh workspace gazebo_launch <pkg_name> <node_name>
```

## Rviz

```
$ros2-docker.sh workspace rviz
```

## Gazebo
```
$ros2-docker.sh workspace gazebo
```

## vcs import
```
.rosinstall if it exists directly under the workspace
$ro2-docker.sh workspace vcs
```

## 5. Release note

* 2021/01/27
 First release shell script version. 


## 6. Contribution

If you find bug or want to new functions, please write issue.

If you fix your self, please fork and send pull request.

## 7. LICENSE

This software license under MIT licence.


## 8. Author 

[masato-ka]()