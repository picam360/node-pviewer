#! /usr/bin/env node
process.chdir(__dirname);

const path = require('path');
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
	let pviewer_version = "https://github.com/picam360/pviewer.git -b v0.32";
	if (process.env.PVIEWER_VERSION) {
		pviewer_version = process.env.PVIEWER_VERSION;
		if(pviewer_version == "github"){
			pviewer_version = "https://github.com/picam360/pviewer.git";
		}
	}
    console.log(`pviewer from "${pviewer_version}"`);
	execSync(`git clone --depth 1 ${pviewer_version}`, {cwd : __dirname});
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

if (process.env.NODE_PSTCORE_VERSION) {
	let node_pstcore_version = process.env.NODE_PSTCORE_VERSION;
	if(node_pstcore_version == "github"){
		node_pstcore_version = "https://github.com/picam360/node-pstcore.git";
	}
	if(node_pstcore_version.startsWith("github.com")){
		node_pstcore_version = `https://${node_pstcore_version}`;
	}
	const node_pstcore_path = path.dirname(require.resolve('node-pstcore'));
	console.log(node_pstcore_path, path.dirname(node_pstcore_path));

	fs.rmSync(node_pstcore_path, {recursive:true, force:true});
	if(node_pstcore_version.startsWith("https")){
		execSync(`git clone --depth 1 ${node_pstcore_version} node-pstcore`, {cwd : path.dirname(node_pstcore_path)});
		execSync(`npm install`, {cwd : node_pstcore_path});
	}else{
		execSync(`npm install node-pstcore@${node_pstcore_version}`);
	}
	
    console.log(`Updated node-pstcore version to ${node_pstcore_version}`);
}else{
	console.log(`Use node-pstcore original version in package.json`);
}