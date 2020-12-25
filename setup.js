#! /usr/bin/env node
const fs = require("fs");
const { execSync } = require('child_process');

if (fs.existsSync('www')) {
	execSync('rm -r www', {cwd : __dirname});
}
execSync('git clone --depth 1 https://github.com/picam360/pviewer.git www', {cwd : __dirname});
execSync('cp -r ./res/www/* ./www/', {cwd : __dirname});