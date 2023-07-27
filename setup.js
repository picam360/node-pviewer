#! /usr/bin/env node
process.chdir(__dirname);

const fs = require("fs");
const rimraf = require("rimraf");
const { execSync } = require('child_process');

try{
	if (fs.existsSync('www')) {
		rimraf.sync('www');
	}
}catch(err){
	console.log("error on rm www:" + err);
}

try{
	execSync('git clone --depth 1 https://github.com/picam360/pviewer.git www -b v0.29', {cwd : __dirname});
}catch(err){
	console.log("error on git:" + err);
}

try{
	fs.copyFileSync('www/config.json.tmp', 'www/config.json');
}catch(err){
	console.log("error on copy config.json : " + err);
}