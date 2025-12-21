//snipet:
// source /opt/ros/humble/setup.bash

console.log("vio");

const { J } = require('quaternion');
const fs = require("fs");
const path = require("path");
const { execSync } = require('child_process');
const yargs = require('yargs');
const fxp = require('fast-xml-parser');
const xml_parser = new fxp.XMLParser({
	ignoreAttributes: false,
	attributeNamePrefix: "",
});
const pif_utils = require('./pif-utils');
const jsonc = require('jsonc-parser');

const rclnodejs = require('rclnodejs');
const cv = require('opencv4nodejs');

let m_options = {
	"data_filepath": "data",
};

async function main() {
	const argv = yargs
		.option('config', {
			alias: 'c',
			type: 'string',
			description: 'config path',
		})
		.option('dir', {
			alias: 'd',
			type: 'string',
			description: 'directory',
		})
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
	m_argv = argv;

	if (argv.config !== undefined) {
		const json_str = fs.readFileSync(argv.config, 'utf-8');
		Object.assign(m_options, jsonc.parse(json_str));
	}
	if (argv.dir !== undefined) {
		m_options.data_filepath = argv.dir;
	}

	const redis = require('redis');
	const client = redis.createClient({
		url: `redis://${host}:${port}`
	});
	client.on('error', (err) => {
		console.error('redis error:', err);
		m_client = null;
		process.exit(1);
	});
	client.connect().then(() => {
		console.log('redis connected:');
		m_client = client;
	});

	const subscriber = client.duplicate();
	subscriber.connect().then(async () => {
		console.log('redis subscriber connected:');

		await rclnodejs.init();
		const ros_node = new rclnodejs.Node('openvins_bridge');

		const imu_publisher = ros_node.createPublisher(
			'sensor_msgs/msg/Imu',
			'/imu0'
		);
		const cam0_publisher = ros_node.createPublisher(
			'sensor_msgs/msg/Image',
			'/cam0/image_raw'
		);
		const cam1_publisher = ros_node.createPublisher(
			'sensor_msgs/msg/Image',
			'/cam1/image_raw'
		);

		let imu_count = 0;
		subscriber.subscribe('pserver-imu', (json_str, key) => {
			try {
				const data = JSON.parse(json_str);
				if (!data || !data.timestamp || !data.gyro || !data.accel) {
					return;
				}

				imu_count++;

				const msg = {
					header: {
						stamp: data.timestamp,
						frame_id: 'imu'
					},
					orientation: {
						x: 0.0, y: 0.0, z: 0.0, w: 1.0
					},
					orientation_covariance: [
						-1, 0, 0,
						0, 0, 0,
						0, 0, 0
					],
					angular_velocity: data.gyro,
					angular_velocity_covariance: [
						0.01, 0, 0,
						0, 0.01, 0,
						0, 0, 0.01
					],
					linear_acceleration: data.accel,
					linear_acceleration_covariance: [
						0.1, 0, 0,
						0, 0.1, 0,
						0, 0, 0.1
					]
				};

				imu_publisher.publish(msg);
			} catch (err) {
				console.log(err);
			}
		}).catch((err) => {
			console.error(err);
		});

		let tmp_img = [];
		let last_ts = Date.now();
		let frame_count = 0;
		subscriber.subscribe('pserver-forward-pst', (data, key) => {
			last_ts = Date.now();
			if (data.length == 0 && tmp_img.length != 0) {
				const img_buffs = tmp_img;
				tmp_img = [];

				if (img_buffs.length != 3) {
					return;
				}

				frame_count++;

				// XMLデータを取得してパース
				const xmlData = img_buffs[0].slice(4).toString('utf-8');
				const parser = new fxp.XMLParser({
					ignoreAttributes: false,
					attributeNamePrefix: "",
				});
				const img_dom = parser.parse(xmlData);
				const ts_nodes = img_dom["picam360:image"].timestamp.split(',');
				const timestamp = {
					sec: parseInt(ts_nodes[0]),
					nanosec: parseInt(ts_nodes[1])*1000
				};

				function matToRosImage(mat, frame_id, stamp) {
					return {
						header: {
							stamp,
							frame_id
						},
						height: mat.rows,
						width: mat.cols,
						encoding: 'bgr8',
						is_bigendian: 0,
						step: mat.cols * mat.channels,
						data: Buffer.from(mat.getData())
					};
				}

				const img = cv.imdecode(img_buffs[2]); // BGR
				const height = img.rows;
				const width = img.cols;
				const singleWidth = Math.floor(width / 2);

				const leftMatROI = img.getRegion(new cv.Rect(0, 0, singleWidth, height));
				const rightMatROI = img.getRegion(new cv.Rect(singleWidth, 0, singleWidth, height));
				const leftMat = leftMatROI.copy();
				const rightMat = rightMatROI.copy();
				const cam0_msg = matToRosImage(leftMat, 'cam0', timestamp);
				const cam1_msg = matToRosImage(rightMat, 'cam1', timestamp);

				// ---- publish ----
				cam0_publisher.publish(cam0_msg);
				cam1_publisher.publish(cam1_msg);
			} else {
				tmp_img.push(Buffer.from(data, 'base64'));
			}
		});

		const fps_interval_sec = 5;
		let last_imu_count = 0;
		let last_frame_count = 0;
		let imu_fps = 0;
		let frame_fps = 0;
		setInterval(() => {
			imu_fps = (imu_count - last_imu_count) / fps_interval_sec;
			frame_fps = (frame_count - last_frame_count) / fps_interval_sec;
			last_imu_count = imu_count;
			last_frame_count = frame_count;
			console.log(`imu freq ${imu_fps.toFixed(3)}, frame freq ${frame_fps.toFixed(3)}`);
		}, fps_interval_sec * 1000);

		setInterval(() => {
			const elapsed = Date.now() - last_ts;
			if (elapsed > 1000) {
				m_client.publish('pserver-vio-info', JSON.stringify({
					"state": "WAITING_PST",
				}));
			} else {
				m_client.publish('pserver-vio-info', JSON.stringify({
					"state": "RECEIVING_PST",
				}));
			}
		}, 1000);
	});
}

if (require.main === module) {
	main();
	process.on('SIGINT', () => {
		console.log('Ctrl+C detected! Gracefully exiting...');
		process.exit(0);
	});
}
