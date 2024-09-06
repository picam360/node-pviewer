#! /usr/bin/env node
process.chdir(__dirname);

const fs = require("fs");
const { execSync } = require('child_process');

try{
	if (fs.existsSync('pviewer')) {
		fs.rmSync('pviewer', {recursive:true, force:true});
	}
}catch(err){
	console.log("error on rm pviewer:" + err);
}

try{
	execSync('git clone --depth 1 https://github.com/picam360/pviewer.git -b v0.32', {cwd : __dirname});
}catch(err){
	console.log("error on git:" + err);
}

try{
	fs.copyFileSync("pviewer/plugins/network/signaling.js", "pserver/plugins/network/signaling.js");
	fs.copyFileSync("pviewer/plugins/network/meeting.js", "pserver/plugins/network/meeting.js");
	fs.copyFileSync("pviewer/plugins/network/rtp.js", "pserver/plugins/network/rtp.js");
}catch(err){
	console.log("copy files:" + err);
}
