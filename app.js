#! /usr/bin/env node
const { spawn } = require('child_process');

process.chdir(__dirname);

spawn('npx', ['electron', 'www'].concat(process.argv.slice(2)));