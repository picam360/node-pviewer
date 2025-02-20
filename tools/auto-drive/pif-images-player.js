
const fs = require("fs");
const path = require('path');
const nmea = require('nmea-simple');
const yargs = require('yargs');
const fxp = require('fast-xml-parser');
const pif_utils = require('./pif-utils');

function main() {
    const argv = yargs
        .option('fps', {
            alias: 'f',
            type: 'number',
			default: 15,
            description: 'fps',
        })
        .option('dir', {
            alias: 'd',
            type: 'string',
			default: "auto-drive-waypoints/waypoint_images",
            description: 'directory',
        })
        .option('host', {
            type: 'string',
            default: 'localhost',
            description: 'directory',
        })
        .option('port', {
            type: 'number',
            default: 6379,
            description: 'port',
        })
        .option('reverse', {
            type: 'boolean',
            default: false,
            description: 'reverse',
        })
        .option('start_cur', {
            type: 'number',
            default: 0,
            description: 'start cursol',
        })
        .option('end_cur', {
            type: 'number',
            default: 0,
            description: 'start cursol',
        })
        .help()
        .alias('help', 'h')
        .argv;

	const fps = argv.fps;
	const dir = argv.dir;
	const host = argv.host;
	const port = argv.port;
	const reverse = argv.reverse;
	let start_cur = argv.start_cur;
	let end_cur = argv.end_cur;

    const redis = require('redis');
    const client = redis.createClient({
        url: `redis://${host}:${port}`
    });

    client.on('error', (err) => {
        console.error('redis error:', err);
    });
    client.connect().then(() => {
        console.log('redis connected:');

		fs.readdir(dir, { withFileTypes: true }, (err, entries) => {
			if (err) {
				console.error('Error reading directory:', err);
				return;
			}
			const pifs = [];
			entries.forEach(entry => {
				if (entry.isFile()) {
					if(path.extname(entry.name) == ".pif"){
						const file_path = path.join(dir, entry.name);
						pifs.push(file_path);
						//console.log(`File: ${entry.name}`);
					}
				} else if (entry.isDirectory()) {
					//console.log(`Directory: ${entry.name}`);
				}
			});
			if(end_cur <= 0){
				end_cur = Object.keys(pifs).length + end_cur;
			}
			if(reverse){
				console.log("reverse");
				pifs.reverse();
				const start_cur_tmp = Object.keys(pifs).length - end_cur;
				const end_cur2_tmp = Object.keys(pifs).length - start_cur;
				start_cur = start_cur_tmp;
				end_cur = end_cur2_tmp;
			}
			let cur = start_cur;
			setInterval(() => {
				if(cur >= Math.min(Object.keys(pifs).length, end_cur)){
					console.log(cur, "done");
					process.exit(0);
				}
				const file_path = pifs[cur];
				pif_utils.read_pif(file_path, (file_path, result) => {
					client.publish('pserver-vslam-pst', Buffer.from(result[0]).toString('base64')).then((data) => {});
					client.publish('pserver-vslam-pst', Buffer.from(result[1]).toString('base64')).then((data) => {});
					client.publish('pserver-vslam-pst', Buffer.from(result[2]).toString('base64')).then((data) => {});
					client.publish('pserver-vslam-pst', "").then((data) => {});
				});
				console.log(cur, file_path);
				cur++;
			}, 1000/fps);
		});
	});
}

if (require.main === module) {
    main();
}
