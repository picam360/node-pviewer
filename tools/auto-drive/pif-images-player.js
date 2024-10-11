
const fs = require("fs");
const path = require('path');
const nmea = require('nmea-simple');
const xml2js = require('xml2js');
const yargs = require('yargs');

function read_pif(file_path, callback) {
	try{

		const data = fs.readFileSync(file_path);
		// 最初の2バイトを取得
		const header_head = data.slice(0, 2).toString('utf-8');
		if (header_head !== 'PI') {
			throw new Error('Invalid file format');
		}

		// 次の4バイトでサイズを取得（ビッグエンディアン）
		const header_size = data.readUInt16BE(2);
		const header = data.slice(0, header_size + 4);

		// XMLデータを取得してパース
		const xmlData = header.slice(4).toString('utf-8');
		xml2js.parseString(xmlData, (err, result) => {
			if (err) throw err;

			const meta_size = parseInt(result["picam360:image"].$.meta_size, 10);
			const meta = data.slice(4 + header_size, 4 + header_size + meta_size);
			const jpeg_filepath = file_path + ".0.0.jpeg";
			const jpeg_data = fs.readFileSync(jpeg_filepath);
			callback(file_path, [header, meta, jpeg_data]);
		});
	}catch(err){

	};
}
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
            description: 'directory',
        })
        .help()
        .alias('help', 'h')
        .argv;

	const fps = argv.fps;
	const dir = argv.dir;

    const redis = require('redis');
    const client = redis.createClient({
        host: 'localhost',
        port: 6379,
    });
    client.on('error', (err) => {
        console.error('redis error:', err);
    });
    client.connect().then(() => {
        console.log('redis connected:');
	});

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
		let cur = 0;
		setInterval(() => {
			if(cur >= Object.keys(pifs).length){
				console.log(cur, "done");
				process.exit(0);
			}
			const file_path = pifs[cur];
			read_pif(file_path, (file_path, result) => {
				client.publish('pserver-vslam-pst', Buffer.from(result[0]).toString('base64')).then((data) => {});
				client.publish('pserver-vslam-pst', Buffer.from(result[1]).toString('base64')).then((data) => {});
				client.publish('pserver-vslam-pst', Buffer.from(result[2]).toString('base64')).then((data) => {});
				client.publish('pserver-vslam-pst', "").then((data) => {});
			});
			console.log(cur, file_path);
			cur++;
		}, 1000/fps);
	});
}

if (require.main === module) {
    main();
}