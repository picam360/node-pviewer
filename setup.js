#! /usr/bin/env node
process.chdir(__dirname);

const fs = require("fs");
const rimraf = require("rimraf");
const rcopy = require('recursive-copy');
const { execSync } = require('child_process');

try{
	if (fs.existsSync('www')) {
		rimraf.sync('www');
	}
}catch(err){
	console.log("error on rm www:" + err);
}

try{
	execSync('git clone --depth 1 https://github.com/picam360/pviewer.git www -b v0.22', {cwd : __dirname});
}catch(err){
	console.log("error on git:" + err);
}

try{
	rcopy('./res/www', './www', function (err) {
		if (err) {
			throw(err);
		}
	});
}catch(err){
	console.log("error on cp ./res/www/* ./www/:" + err);
}