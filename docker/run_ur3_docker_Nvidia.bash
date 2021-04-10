xhost + local:root

command="docker run \
    --rm \
    --gpus all -e NVIDIA_DRIVER_CAPABILITIES=all \
    --privileged \
    -it \
    -e DISPLAY \
    --env='QT_X11_NO_MITSHM=1' \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    ur3_erc"

$command

#    --network=host \
#    --env="XAUTHORITY=$XAUTH" \
#    --volume="$XAUTH:$XAUTH" \
#    --volume="/dev:/dev" \
