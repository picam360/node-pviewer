#! /usr/bin/env node
process.chdir(__dirname);

const { execSync } = require('child_process');

const argv = ['npx', 'electron', '.'].concat(process.argv.slice(2));
execSync(argv.join(' '));