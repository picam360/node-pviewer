apt-get install -y nano
apt-get install -y ros-humble-isaac-ros-visual-slam

target_dir="/opt/ros/humble/share/isaac_ros_visual_slam/launch"
file_name="isaac_ros_visual_slam.launch.py"

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