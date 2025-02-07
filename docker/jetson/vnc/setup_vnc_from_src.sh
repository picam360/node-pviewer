
sudo apt-get -y install libfltk1.3-dev libxtst-dev libxdamage-dev libgmp-dev gettext libpam0g-dev

git clone --depth=1 https://github.com/TigerVNC/tigervnc.git
cd tigervnc
mkdir build
cd build
cmake ..
make -j4
sudo make install

cd ../..
rm -rf tigervnc
