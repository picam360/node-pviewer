#!/bin/bash

IMAGE_NAME=vslam
VERSION=latest
if sudo docker image inspect ${IMAGE_NAME}:${VERSION} 1> /dev/null 2>/dev/null; then
    echo "${IMAGE_NAME}:${VERSION} found"
else
    echo "${IMAGE_NAME}:${VERSION} not found"

    sudo docker build -t ${IMAGE_NAME}:${VERSION} \
        -f Dockerfile.vslam .
fi