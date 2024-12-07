console.log("robot-localization-bridge");

//ros noetic
//process.env.CMAKE_PREFIX_PATH = "/opt/ros/noetic";
//process.env.ROS_MASTER_URI = "http://localhost:11311";
//process.env.ROS_PACKAGE_PATH = "/opt/ros/noetic/share";

//ros2 humble
//process.env.LD_LIBRARY_PATH = "/opt/ros/humble/lib/aarch64-linux-gnu:/opt/ros/humble/lib:/usr/local/cuda-11.4/lib64";

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
const PifRosMessagePublisher = require('./pif-ros-message-publisher-humble');

let m_options = {
	"waypoint_threshold_m": 10,
	"data_filepath": "auto-drive-waypoints"
};
let m_ros_msg_pub = new PifRosMessagePublisher();
let m_vslam_frame_skip = 0;
let m_vslam_frame_count = 0;
const m_ros_timestamp_base_ms = Date.now();

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

	if (m_options.debug) {
		console.log(current_nmea, current_imu, current_encoder);
	}

	//const timestampSec = toSec(timestamp);

	const rosTimestamp = (() => {
		const { seconds, nanoseconds } = { seconds : Math.floor(m_ros_timestamp_base_ms / 1e3), nanoseconds : 0 };
		const additionalNanoseconds = Math.floor(m_vslam_frame_count * (1 / 30) * 1e9);
		const totalNanoseconds = nanoseconds + additionalNanoseconds;
		return {
			sec: seconds + Math.floor(totalNanoseconds / 1e9),
			nanosec: totalNanoseconds % 1e9,
		};
	})();

	m_ros_msg_pub.publishWheelCount([current_encoder.left, current_encoder.right], current_imu.heading, rosTimestamp);
	m_ros_msg_pub.publishGpsNmea(nmea_str, rosTimestamp);
	m_ros_msg_pub.publishImu(current_imu, rosTimestamp);
	if((m_vslam_frame_count % (m_vslam_frame_skip + 1)) == 0){
		m_ros_msg_pub.publishVslam(tmp_img[2], rosTimestamp);
	}
	m_vslam_frame_count++;
	//console.log(m_vslam_frame_count);
    //console.log(process.memoryUsage());
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
		.option('vslam-frame-skip', {
			type: 'number',
			default: 0,
			description: 'vslam-frame-skip',
		})
		.help()
		.alias('help', 'h')
		.argv;
	const host = argv.host;
	const port = argv.port;
	m_vslam_frame_skip = argv['vslam-frame-skip'];

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
	subscriber.connect().then(async () => {
		console.log('redis connected:');

		await m_ros_msg_pub.initialize();

		m_ros_msg_pub.subscribeOdometry((odometry) => {
			client.publish('pserver-odometry', JSON.stringify(odometry), (err, reply) => {
				if (err) {
					console.error('Error publishing message:', err);
				} else {
					//console.log(`Message published to ${reply} subscribers.`);
				}
			});
		});

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
	process.on('SIGINT', () => {
		console.log('Ctrl+C detected! Gracefully exiting...');
		process.exit(0);
	});
	main();
}
