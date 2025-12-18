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

let m_options = {
	"data_filepath" : "data",
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

	if(argv.config !== undefined){
		const json_str = fs.readFileSync(argv.config, 'utf-8');
		Object.assign(m_options, jsonc.parse(json_str));
	}
	if(argv.dir !== undefined){
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

		subscriber.subscribe('pserver-imu', (json_str, key) => {
			try{
				const data = JSON.parse(json_str);
				if(!data || !data.byro || !data.accel){
					return;
				}
				const msg = {
				  header: {
					stamp: now,
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
				  angular_velocity: imu_data.gyro,
				  angular_velocity_covariance: [
					0.01, 0,    0,
					0,    0.01, 0,
					0,    0,    0.01
				  ],
				  linear_acceleration: imu_data.accel,
				  linear_acceleration_covariance: [
					0.1, 0,   0,
					0,   0.1, 0,
					0,   0,   0.1
				  ]
				};
			
				imu_publisher.publish(msg);
			}catch(err){
				console.log(err);
			}
		}).catch((err) => {
			console.error(err);
		});

		let tmp_img = [];
		let last_ts = Date.now();
		subscriber.subscribe('pserver-forward-pst', (data, key) => {
			last_ts = Date.now();
			if(data.length == 0 && tmp_img.length != 0){
				tmp_img = [];
			}else{
				tmp_img.push(Buffer.from(data, 'base64'));
			}
		});
		
		setInterval(() => {
			const elapsed = Date.now() - last_ts;
			if(elapsed > 1000){
				m_client.publish('pserver-vio-info', JSON.stringify({
					"state" : "WAITING_PST",
				}));
			}else{
				m_client.publish('pserver-vio-info', JSON.stringify({
					"state" : "RECEIVING_PST",
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
