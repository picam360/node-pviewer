#!/bin/bash

# USBメモリのパス（空の場合はマウントしない）
USB_PATH="/media/picam360/KINGSTON"

# イメージ名を変数として定義
IMAGE_NAME="auto_drive_wheel_odom"
CONT_NAME="auto_drive_wheel_odom"

ROS_DISTRO=noetic
WORKSPACE_HOST=/home/picam360/${CONT_NAME}/workspace
WORKSPACE_CONT=/home/picam360/workspace/

# 情報出力関数
print_info() {
    echo "[INFO] $1"
}

# コンテナが実行中かチェック
if [ "$(docker ps -q -f name=${CONT_NAME})" ]; then
    print_info "既存のコンテナに接続します: ${CONT_NAME}"
    docker exec -it ${CONT_NAME} bash
    exit 0
fi

# 停止中のコンテナがあるかチェック
if [ "$(docker ps -aq -f status=exited -f name=${CONT_NAME})" ]; then
    print_info "停止中のコンテナを削除します: ${CONT_NAME}"
    docker rm ${CONT_NAME}
fi

# Dockerの引数を初期化
DOCKER_ARGS=(
    "-it"
    "--rm"
    "--name" "${CONT_NAME}"
    "--privileged"
    "--network" "host"
    "--entrypoint" "/bin/bash"
    "-v" "${WORKSPACE_HOST}:${WORKSPACE_CONT}"
    "-v" "/tmp/.X11-unix:/tmp/.X11-unix"
    "-v" "/dev/*:/dev/*"
    "-e" "DISPLAY"
)

# USBメモリのマウント処理
if [ -n "$USB_PATH" ] && [ -d "$USB_PATH" ]; then
    print_info "USBメモリをマウントします: $USB_PATH"
    DOCKER_ARGS+=("-v" "$USB_PATH:/mnt/usb-in-container:rw")
else
    print_info "USBメモリをマウントせずに起動します"
fi

# Dockerコンテナを実行
sudo docker run "${DOCKER_ARGS[@]}" "$IMAGE_NAME"
