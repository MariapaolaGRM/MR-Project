xhost +
docker run -it --rm --net host --ipc host --privileged \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v ~/.Xauthority:/root/.Xauthority \
    --runtime nvidia --gpus all \
    -e DISPLAY=$DISPLAY \
    -e XAUTHORITY=$XAUTHORITY \
    -v ./ros_ws/:/root/ros_workspace \
    --name proj \
    ros:project bash
