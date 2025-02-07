
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
	odom_type : ODOMETRY_TYPE.VSLAM,
//	odom_type : ODOMETRY_TYPE.ENCODER,
	GPS : {
		enabled : true
	},
	ENCODER : {
		enabled : true
	},
	VSLAM : {
		enabled : true
	},
};

function init_odometory_handlers(){
	const keys = Object.keys(m_odometry_conf);
	function build_odometry_handler(cur){
		const key = keys[cur];
		const next_cb = () => {
			if(cur == keys.length - 1){
				console.log("drive mode", m_drive_mode);

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
					reverse : m_options.reverse,
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

		stop_robot();
		
		const pif_dirpath = `${m_options.data_filepath}/waypoint_images`;
		load_auto_drive_waypoints_ext(pif_dirpath, 0, null, (waypoints) => {
			waypoints = reindex_waypoints(waypoints, m_options.reverse);

			update_auto_drive_waypoints({
				src : waypoints
			});
			update_auto_drive_cur(0);
		});
	});

	const subscriber = client.duplicate();
	subscriber.connect().then(() => {
		console.log('redis subscriber connected:');

		subscriber.subscribe('pserver-odometry', (data, key) => {
			const params = data.trim().split(' ');
			switch (params[0]) {
				case "CMD":
					console.log(`"${data}" subscribed.`);
					command_handler(data.substr(params[0].length + 1));
					break;
			}
		});
		
		subscriber.subscribe('pserver-vslam-pst', (data, key) => {
			last_ts = Date.now();
			if(data.length == 0 && tmp_img.length != 0){
				//TODO:vslam
				tmp_img = [];
			}else{
				tmp_img.push(Buffer.from(data, 'base64'));
			}
		});

		let tmp_img = [];
		let last_ts = Date.now();
		subscriber.subscribe('pserver-vslam-pst', (data, key) => {
			last_ts = Date.now();
			if(data.length == 0 && tmp_img.length != 0){
				//TODO:vslam
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
		case "START_AUTO":
			stop_robot();
			if(m_drive_mode == "STANBY") {
				m_options.reverse = (split[1] == "REVERSE");

				m_drive_mode = "AUTO";
				m_client.publish('pserver-odometry-info', JSON.stringify({
					"mode" : "AUTO",
					"state" : "START_AUTO",
				}));

				const pif_dirpath = `${m_options.data_filepath}/waypoint_images`;
				load_auto_drive_waypoints_ext(pif_dirpath, 0, null, (waypoints) => {
					waypoints = reindex_waypoints(waypoints, m_options.reverse);

					const msg = {
						src : waypoints
					};
					const keys = Object.keys(m_odometry_conf);
					function build_odometry_handler(cur){
						const key = keys[cur];
						const next_cb = (converted_waypoints) => {
							m_odometry_conf[key].converted_waypoints = converted_waypoints;
							msg[key] = converted_waypoints;
							if(cur == keys.length - 1){
								update_auto_drive_waypoints(msg);
								update_auto_drive_cur(0);
								console.log("drive mode", m_drive_mode);

								m_client.publish('pserver-odometry-info', JSON.stringify({
									"mode" : "AUTO",
									"state" : "READY_AUTO",
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
									reverse : m_options.reverse,
									//host : m_argv.host,
									transforms_callback : (vslam_waypoints, active_points) => {
										msg["VSLAM"] = vslam_waypoints;
										msg["VSLAM_ACTIVE"] = active_points;
										update_auto_drive_waypoints(msg);
									},
								});
								break;
						}
						m_odometry_conf[key].handler.init(waypoints, next_cb);
					}
					build_odometry_handler(0);
				});
			}else{
				console.log("drive mode", m_drive_mode);
			}
			break;
		case "STOP_AUTO":
			stop_robot();
			setTimeout(() => {
				stop_robot();
			}, 1000)
			m_drive_mode = "STANBY";

			m_client.publish('pserver-odometry-info', JSON.stringify({
				"mode" : "AUTO",
				"state" : "STOP_AUTO",
			}));

			console.log("drive mode", m_drive_mode);
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
