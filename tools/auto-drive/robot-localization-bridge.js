const { J } = require('quaternion');
console.log("auto-drive");
const fs = require("fs");
const path = require("path");
const nmea = require('nmea-simple');
const xml2js = require('xml2js');
const { execSync } = require('child_process');
const yargs = require('yargs');
const fxp = require('fast-xml-parser');
const xml_parser = new fxp.XMLParser({
	ignoreAttributes: false,
	attributeNamePrefix: "",
});
const pif_utils = require('./pif-utils');
const PifRosMessagePublisher = require('./pif-ros-message-publisher');

let m_options = {
	"waypoint_threshold_m": 10,
	"data_filepath": "auto-drive-waypoints"
};
let m_socket = null;
let m_drive_mode = "STANBY";
let m_averaging_nmea = null;
let m_averaging_count = 0;
let m_last_nmea = null;
let m_auto_drive_waypoints = null;
let m_auto_drive_cur = 0;
let m_ros_msg_pub = new PifRosMessagePublisher();

function toSec(timestampText) {
	const [seconds, microseconds] = timestampText.split(',');
	const secondsNumber = parseInt(seconds);
	const microToSeconds = parseInt(microseconds) / 1000000;
	const totalSeconds = secondsNumber + microToSeconds;
	return  totalSeconds;
}

function auto_drive_handler(tmp_img) {
	if (tmp_img.length != 3) {
		return;
	}

	const data = Buffer.concat([tmp_img[0], tmp_img[1]]);
	const header = data.slice(0, 2).toString('utf-8');
	if (header !== 'PI') {
		throw new Error('Invalid file format');
	}

	const header_size = data.readUInt16BE(2);
	const xml = data.slice(4, 4 + header_size).toString('utf-8');
	const img_dom = xml_parser.parse(xml);

	const timestamp = img_dom["picam360:image"].timestamp;
	const meta_size = parseInt(img_dom["picam360:image"].meta_size, 10);
	const meta = data.slice(4 + header_size, 4 + header_size + meta_size);
	const frame_dom = xml_parser.parse(meta);
	const nmea_str = frame_dom['picam360:frame']['passthrough:nmea'];
	const current_nmea = nmea.parseNmeaSentence(nmea_str);
	const current_imu = JSON.parse(frame_dom['picam360:frame']['passthrough:imu']);
	const current_encoder = JSON.parse(frame_dom['picam360:frame']['passthrough:encoder']);

	console.log(current_nmea, current_imu, current_encoder);

	if(!m_ros_msg_pub.isInitialized){
		m_ros_msg_pub.initialize();
	}
	const timestampSec = toSec(timestamp);
	m_ros_msg_pub.updateWheelCount(current_encoder.left, current_encoder.right, timestampSec);
	m_ros_msg_pub.updateGpsNmea(nmea_str, timestampSec);
	m_ros_msg_pub.publishMessages();
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
		.help()
		.alias('help', 'h')
		.argv;
	const host = argv.host;
	const port = argv.port;

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

	const subscriber = client.duplicate();
	subscriber.connect().then(() => {
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
