console.log("pif-sbs-splitter");

const { J } = require('quaternion');
const fs = require("fs");
const path = require("path");
const nmea = require('nmea-simple');
const { execSync } = require('child_process');
const yargs = require('yargs');
const fxp = require('fast-xml-parser');
const xml_parser = new fxp.XMLParser({
	ignoreAttributes: false,
	attributeNamePrefix: "",
});
const pif_utils = require('./pif-utils');
const cv = require('opencv4nodejs');

let m_options = {
	"waypoint_threshold_m": 10,
	"data_filepath": "auto-drive-waypoints"
};
let m_vslam_frame_count = 0;

function toSec(timestampText) {
	const [seconds, microseconds] = timestampText.split(',');
	const secondsNumber = parseInt(seconds);
	const microToSeconds = parseInt(microseconds) / 1000000;
	const totalSeconds = secondsNumber + microToSeconds;
	return totalSeconds;
}

async function auto_drive_handler(tmp_img) {
	if (tmp_img.length != 3) {
		return;
	}
	
	const sideBySideImageBuffer = tmp_img[2];
	const image = cv.imdecode(sideBySideImageBuffer);

	const { cols: width, rows: height } = image;

	const halfWidth = Math.floor(width / 2);

	//camera image
	const leftROI = new cv.Rect(0, 0, halfWidth, height);
	if (leftROI.width <= 0 || leftROI.height <= 0) {
		throw new Error('Invalid left ROI dimensions.');
	}
	const leftImage = image.getRegion(leftROI).copy();
	const leftImage_filepath = `${m_options.data_filepath}/frames_left/frame_${String(m_vslam_frame_count + 1).padStart(6, '0')}.png`;
	cv.imwrite(leftImage_filepath, leftImage);
	console.log(leftImage_filepath);

	const rightROI = new cv.Rect(halfWidth, 0, halfWidth, height);
	if (rightROI.width <= 0 || rightROI.height <= 0) {
		throw new Error('Invalid right ROI dimensions.');
	}
	const rightImage = image.getRegion(rightROI).copy();
	const rightImage_filepath = `${m_options.data_filepath}/frames_right/frame_${String(m_vslam_frame_count + 1).padStart(6, '0')}.png`;
	cv.imwrite(rightImage_filepath, rightImage);
	console.log(rightImage_filepath);

	m_vslam_frame_count++;
}

function main() {

	const argv = yargs
		.option('host', {
			type: 'string',
			default: 'localhost',
			description: 'host',
		})
		.option('port', {
			type: 'number',
			default: 6379,
			description: 'port',
		})
        .option('dir', {
            alias: 'd',
            type: 'string',
            description: 'directory',
        })
		.help()
		.alias('help', 'h')
		.argv;
	const host = argv.host;
	const port = argv.port;

	if(argv.dir){
		m_options.data_filepath = argv.dir;
	}

	const redis = require('redis');
	const client = redis.createClient({
		url: `redis://${host}:${port}`
	});
	client.on('error', (err) => {
		console.error('redis error:', err);
		m_client = null;
	});
	client.connect().then(() => {
		console.log('redis connected:');
		m_client = client;
	});

	{
		const left_dirpath = `${m_options.data_filepath}/frames_left`;
		if (!fs.existsSync(left_dirpath)) {
			try {
				fs.mkdirSync(left_dirpath, { recursive: true });
			} catch (err) {
				console.log(err);
			}
		}
	}
	{
		const right_dirpath = `${m_options.data_filepath}/frames_right`;
		if (!fs.existsSync(right_dirpath)) {
			try {
				fs.mkdirSync(right_dirpath, { recursive: true });
			} catch (err) {
				console.log(err);
			}
		}
	}

	const subscriber = client.duplicate();
	subscriber.connect().then(async () => {
		console.log('redis connected:');

		let tmp_img = [];
		subscriber.subscribe('pserver-vslam-pst', (data, key) => {
			if (data.length == 0 && tmp_img.length != 0) {
				auto_drive_handler(tmp_img);
				tmp_img = [];
			} else {
				tmp_img.push(Buffer.from(data, 'base64'));
			}
		});
	});
}

if (require.main === module) {
	main();
}
