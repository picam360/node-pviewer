#! /usr/bin/env node
const { execSync } = require('child_process');

const argv = ['npx', 'electron', 'www'].concat(process.argv.slice(2));

execSync(argv.join(' '), {cwd : __dirname});