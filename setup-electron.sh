#!/bin/bash

cordova create pviewer-electron com.picam360.pviewer PViewer
cd pviewer-electron
cp ../config.xml ./
#cp ../icon.png ./
cp ../splash.png ./

npm install ../cordova-plugin-pstcore/src/electron/

rm -r www
git clone --depth 1 https://github.com/picam360/pviewer.git www
cp ../config.json ./www/

cp -r ../../plugins/cordova_binder_st/app/res ./

cordova platform add electron
cp ../icon.png ./platforms/electron/build-res/installer.png
cordova prepare electron
cordova run electron
