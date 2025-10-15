#!/bin/bash
#

docker run --rm -it \
    --runtime=nvidia \
    --network=host \
    --privileged \
    -e "TERM=xterm-256color" \
    -v "/dev:/dev" \
    -v "`pwd`/../symbiote-ag:/home/`whoami`/ws/src/symbiote-ag" \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -e XAUTHORITY=$XAUTH \
    --name lang-air-ground-team-l4t \
    lang-air-ground-team:l4t \
    bash
