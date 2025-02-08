
console.log("odometry");

const { J } = require('quaternion');
const fs = require("fs");
const path = require("path");
const nmea = require('nmea-simple');
const utm = require('utm');
const { execSync } = require('child_process');
const yargs = require('yargs');
const fxp = require('fast-xml-parser');
const xml_parser = new fxp.XMLParser({
	ignoreAttributes: false,
	attributeNamePrefix: "",
});
const pif_utils = require('./pif-utils');
const jsonc = require('jsonc-parser');

const { GpsOdometry } = require('./odometry-handlers/gps-odometry');
const { EncoderOdometry } = require('./odometry-handlers/encoder-odometry');
const { VslamOdometry } = require('./odometry-handlers/vslam-odometry');

let m_options = {
	"data_filepath" : "data",
};

let m_argv = {};
let m_drive_mode = "LOCALIZATION";//LOCALIZATION or MAPPING
let m_averaging_nmea = null;
let m_averaging_count = 0;
let m_last_nmea = null;

const ODOMETRY_TYPE = {
	GPS : "GPS",
	ENCODER : "ENCODER",
	VSLAM : "VSLAM",
};
let m_odometry_conf = {
//	odom_type : ODOMETRY_TYPE.VSLAM,
	odom_type : ODOMETRY_TYPE.ENCODER,
	GPS : {
		enabled : false
	},
	ENCODER : {
		enabled : true
	},
	VSLAM : {
		enabled : false
	},
};

function init_odometory_handlers(){
	const keys = Object.keys(m_odometry_conf);
	function build_odometry_handler(cur){
		const key = keys[cur];
		const next_cb = () => {
			if(cur == keys.length - 1){
				console.log("mode", m_drive_mode);

				m_client.publish('pserver-odometry-info', JSON.stringify({
					"mode" : "LOCALIZATION",
					"state" : "READY_LOCALIZATION",
				}));

			}else{
				build_odometry_handler(cur + 1);
			}
		};
		if(m_odometry_conf[key].handler){
			m_odometry_conf[key].handler.deinit();
			m_odometry_conf[key].handler = null;
		}
		if(!m_odometry_conf[key].enabled){
			next_cb();
			return;
		}
		switch(key){
			case ODOMETRY_TYPE.GPS:
				m_odometry_conf[key].handler = new GpsOdometry();
				break;
			case ODOMETRY_TYPE.ENCODER:
				m_odometry_conf[key].handler = new EncoderOdometry();
				break;
			case ODOMETRY_TYPE.VSLAM:
				m_odometry_conf[key].handler = new VslamOdometry({
					//host : m_argv.host,
					transforms_callback : (vslam_waypoints, active_points) => {
						msg["VSLAM"] = vslam_waypoints;
						msg["VSLAM_ACTIVE"] = active_points;
						update_auto_drive_waypoints(msg);
					},
				});
				break;
		}
		m_odometry_conf[key].handler.init(next_cb);
	}
	build_odometry_handler(0);
}
function localization_handler(tmp_img){
	if(tmp_img.length != 3){
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

	const meta_size = parseInt(img_dom["picam360:image"].meta_size, 10);
	const meta = data.slice(4 + header_size, 4 + header_size + meta_size);

	const conf_keys = Object.keys(m_odometry_conf)
	for(const key of conf_keys){
		const conf = m_odometry_conf[key];
		if(conf.handler){
			conf.handler.push(header, meta, tmp_img[2]);
		}
	}

	if(m_odometry_conf[m_odometry_conf.odom_type].handler.is_ready() == false){
		return;
	}
	
	const odom = m_odometry_conf[m_odometry_conf.odom_type].handler.getPosition();
	m_client.publish('pserver-odometry-info', JSON.stringify({
		"mode" : m_drive_mode,
		"state" : "UPDATE_ODOMETRY",
		"odom" : odom,
	}));
}

function main() {
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

	if(m_options.gps_odom){
		Object.assign(GpsOdometry.settings, m_options.gps_odom);
	}
	if(m_options.encoder_odom){
		Object.assign(EncoderOdometry.settings, m_options.encoder_odom);
	}
	if(m_options.vslam_odom){
		Object.assign(VslamOdometry.settings, m_options.vslam_odom);
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

	const subscriber = client.duplicate();
	subscriber.connect().then(() => {
		console.log('redis subscriber connected:');

		init_odometory_handlers();

		subscriber.subscribe('pserver-odometry', (data, key) => {
			const params = data.trim().split(' ');
			switch (params[0]) {
				case "CMD":
					console.log(`"${data}" subscribed.`);
					command_handler(data.substr(params[0].length + 1));
					break;
			}
		});

		let tmp_img = [];
		let last_ts = Date.now();
		subscriber.subscribe('pserver-vslam-pst', (data, key) => {
			last_ts = Date.now();
			if(data.length == 0 && tmp_img.length != 0){
				localization_handler(tmp_img);

				tmp_img = [];
			}else{
				tmp_img.push(Buffer.from(data, 'base64'));
			}
		});
		setInterval(() => {
			const elapsed = Date.now() - last_ts;
			if(elapsed > 1000){
				m_client.publish('pserver-odometry-info', JSON.stringify({
					"mode" : m_drive_mode,
					"state" : "WAITING_PST",
				}));
			}else{
				m_client.publish('pserver-odometry-info', JSON.stringify({
					"mode" : m_drive_mode,
					"state" : "RECEIVING_PST",
				}));
			}
		}, 1000);
	});
}

function command_handler(cmd) {
	let split = cmd.split(' ');
	switch(split[0]){
		case "START_MAPPING":
			break;
		case "STOP_MAPPING":
			break;
	}
}

if (require.main === module) {
    main();
	process.on('SIGINT', () => {
		console.log('Ctrl+C detected! Gracefully exiting...');
		process.exit(0);
	});
}
