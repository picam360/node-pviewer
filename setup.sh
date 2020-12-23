#!/bin/bash

cordova create pviewer_electron com.picam360.pviewer PViewer
cd pviewer_electron
cp ../config.xml ./
#cp ../icon.png ./
cp ../splash.png ./

cordova plugin add ../pstcore-electron/
npm install pstcore

rm -r www
git clone --depth 1 https://github.com/picam360/pviewer.git www
cp ../config.json ./www/

cp -r ../res ./

cordova platform add electron@2.0.0
cp ./res/electron/icon.png ./platforms/electron/build-res/installer.png
cp ./res/electron/cdv-electron-main.js ./platforms/electron/platform_www/cdv-electron-main.js
cordova prepare electron --release
#cordova run electron --nobuild
