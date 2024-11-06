target_dir="${ISAAC_ROS_WS}/src/isaac_ros_common/scripts"
file_name="run_dev.sh"

# 現在のディレクトリを保存し、ターゲットディレクトリに移動
current_dir=$(pwd)
cd "$target_dir"

# バックアップファイルが存在しない場合、バックアップを作成
[ -f "$file_name" ] && [ ! -f "${file_name}.bak" ] && \
    sudo cp "$file_name" "${file_name}.bak"

# 元のディレクトリに戻る
cd "$current_dir"

# 新しいファイルをコピー
sudo cp -f \
    "./assets/$file_name" \
    "$target_dir/$file_name"

echo "${file_name}を置き換えました"

cd ${ISAAC_ROS_WS}/src/isaac_ros_common && ./scripts/run_dev.sh ${ISAAC_ROS_WS}