# Tutorial for running demo with Foxglove

## Isaac ROS Common

### Modify `dockerfile.ros2_humble` to build custom container

First, add the following in `isaac_ros_common/docker/Dockerfile.aarch64.ros2_humble` after `FROM nvcr.io/nvidia/isaac/ros:aarch64-ros2_humble_33836e394da2d095a59afd2d151038f8` line.

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
&& apt-get clean

RUN git clone https://github.com/ros2/demos/ && \
    cd demos && git checkout humble && \ 
    mv image_tools /workspaces/isaac_ros-dev/src && \
    cd /workspaces/isaac_ros-dev/ && \
    colcon build --packages-up-to image_tools && \
    source install/setup.bash
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


## Demo 1 : USB webcam

### Terminal 1

```
isaac_ros_container
```

Once in container;

```
source ./install/setup.bash
ros2 run image_tools cam2image --ros-args -p device_id:=1 -p width:=640 -p height:=480
```

### Terminal 2

```
isaac_ros_container
```

Once in container;

```
source ./install/setup.bash
ros2 launch isaac_ros_unet isaac_ros_unet_triton_foxglove.launch.py \
    model_name:=peoplesemsegnet_shuffleseg \
    model_repository_paths:=['/tmp/models'] \
    input_binding_names:=['input_2:0'] \
    output_binding_names:=['argmax_1'] \
    network_output_type:='argmax'
```

### On Jetson Desktop UI

Launch Foxglove application.

Click on the logo at the left-top, select `File` > `ws://localhost:8765`.

Make sure the **LAYOUT** is set to `Default` or `Image Segmentation (Original top, Seg mask down)`.


## Demo 2 : CSI Camera

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
ros2 launch isaac_ros_unet isaac_ros_argus_unet_triton_foxglove.launch.py \
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
ros2 topic list
```


### In case you need to rebuild

```
sudo apt-get install -y libwebsocketpp-dev 
cd /workspaces/isaac_ros-dev && \
  colcon build --symlink-install && \
  source install/setup.bash
```