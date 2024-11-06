#!/bin/bash
#./bind_mount.sh /media/picam360/KINGSTON/vslam/roadside_mf/roadside_mf_0 /media/picam360/KINGSTON/vslam/_ref/video

# 使用方法をチェック
if [ "$#" -ne 2 ]; then
    echo "使用方法: $0 <元のディレクトリパス> <マウントポイントパス>"
    exit 1
fi

# パラメータを変数に格納
SOURCE_DIR="$1"
MOUNT_POINT="$2"

# ソースディレクトリの存在チェック
if [ ! -d "$SOURCE_DIR" ]; then
    echo "エラー: 元のディレクトリ '$SOURCE_DIR' が存在しません。"
    exit 1
fi

# マウントポイントの存在チェック、なければ作成
if [ ! -d "$MOUNT_POINT" ]; then
    echo "マウントポイント '$MOUNT_POINT' が存在しません。作成します..."
    sudo mkdir -p "$MOUNT_POINT"
    if [ $? -ne 0 ]; then
        echo "エラー: マウントポイントの作成に失敗しました。"
        exit 1
    fi
fi

# 既存のマウントをチェックし、存在する場合はアンマウント
if mountpoint -q "$MOUNT_POINT"; then
    echo "既存のマウントポイントを検出しました。アンマウントします..."
    sudo umount "$MOUNT_POINT"
    if [ $? -ne 0 ]; then
        echo "エラー: 既存のマウントポイントのアンマウントに失敗しました。"
        exit 1
    fi
fi

# バインドマウントの実行
sudo mount --bind "$SOURCE_DIR" "$MOUNT_POINT"

if [ $? -eq 0 ]; then
    echo "バインドマウントが成功しました。"
    echo "'$SOURCE_DIR' を '$MOUNT_POINT' にマウントしました。"
else
    echo "エラー: バインドマウントに失敗しました。"
    exit 1
fi