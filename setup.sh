#!/bin/bash

rm -r www
git clone --depth 1 https://github.com/picam360/pviewer.git www
cp -r ./res/www/* ./www/