#!/bin/bash
#

docker run --rm -it \
    --gpus all \
    --network=host \
    --privileged \
    -e "TERM=xterm-256color" \
    -v "/dev:/dev" \
    -v "/tmp/.X11-unix:/tmp/.X11-unix" \
    -v "`pwd`/../flyer:/home/`whoami`/ws/src/flyer" \
    -v "/home/`whoami`/Data:/home/`whoami`/data" \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -e XAUTHORITY=$XAUTH \
    --name flyer-x86 \
    flyer:x86 \
    bash
