# Tutorial for running demo with Foxglove

## Isaac ROS Common

### Modify `dockerfile.ros2_humble` to build custom container

First, add the following in `isaac_ros_common/docker/Dockerfile.ros2_humble` before its `FROM ${BASE_IMAGE}` line.

```
# Install foxglove_bridge
RUN apt-get update && mkdir -p ${ROS_ROOT}/src && cd ${ROS_ROOT}/src \
    && apt-get install -y libwebsocketpp-dev \
    && git clone https://github.com/foxglove/ros-foxglove-bridge.git && cd ros-foxglove-bridge && git checkout main && cd .. \
    && source ${ROS_ROOT}/setup.bash && cd ${ROS_ROOT} \
    && rosdep install -y -r --ignore-src --from-paths src --rosdistro ${ROS_DISTRO} \
    && colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo --packages-up-to foxglove_bridge \
    && rm -Rf src build log \
&& rm -rf /var/lib/apt/lists/* \
&& apt-get cleand
```

Then remove `Dockerfile.aarch64.ros2_humble` to skip just pulling a pre-built container image from NGC.

```
cd $ISAAC_ROS_WS
rm src/isaac_ros_common/docker/Dockerfile.aarch64.ros2_humble
```

Also, when executing `docker run` command, mount the model directory for `peoplesemsegnet_shuffleseg` files to skip TensorRT engine build process.

```
vi $ISAAC_ROS_WS/src/isaac_ros_common/
```

Add following line.

```
DOCKER_ARGS+=("-v /home/jetson/workspaces/models:/tmp/models")
```



## Copy build TensorRT engine files

```
cd $ISAAC_ROS_WS
mkdir modles
cd models
pip3 install gdown
gdown --folder https://drive.google.com/drive/folders/1nmAg9OcOQ24R13fBxu8xdpDLCENM2y-a
```


## Procedure

### On desktop (Optional)

```
xhost +
isaac_ros_container
```

In the container started on desktop

```
rviz2
```

### On remote (or local) terminal

#### Terminal 1

```
isaac_ros_container
```

Inside the container

```
source ./install/setup.bash
ros2 launch isaac_ros_unet isaac_ros_argus_unet_triton.launch.py \
    model_name:=peoplesemsegnet_shuffleseg \
    model_repository_paths:=['/tmp/models'] \
    input_binding_names:=['input_2:0'] \
    output_binding_names:=['argmax_1'] \
    network_output_type:='argmax'
```

#### Terminal 2

```
isaac_ros_container
```

Inside the container

```
source ./install/setup.bash
ros2 launch foxglove_bridge foxglove_bridge.xml
```


### In case you need to rebuild

```
sudo apt-get install -y libwebsocketpp-dev 
cd /workspaces/isaac_ros-dev && \
  colcon build --symlink-install && \
  source install/setup.bash
```