mkdir -p ~/cmake
cd ~/cmake
wget https://github.com/Kitware/CMake/releases/download/v3.31.11/cmake-3.31.11-linux-aarch64.tar.gz
tar xvfz cmake-3.31.11-linux-aarch64.tar.gz
echo "export PATH=$HOME/cmake/cmake-3.31.11-linux-aarch64/bin:$PATH" >> ~/.bashrc