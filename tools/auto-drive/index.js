const { J } = require('quaternion');
console.log("auto-drive");
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

let m_options = {
	"waypoint_threshold_m" : 10,
	"data_filepath" : "auto-drive-waypoints"
};
let m_socket = null;
let m_drive_mode = "STANBY";
let m_averaging_nmea = null;
let m_averaging_count = 0;
let m_last_nmea = null;
let m_auto_drive_waypoints = null;
let m_auto_drive_cur = 0;

function latLonToXY(lat1, lon1, lat2, lon2) {
	const R = 6378137; // Earth's radius in meters
	
	// Convert latitude and longitude to radians
	const φ1 = lat1 * Math.PI / 180;
	const φ2 = lat2 * Math.PI / 180;
	const Δλ = (lon2 - lon1) * Math.PI / 180;
	
	// Calculate XY coordinates
	const x = R * Δλ * Math.cos((φ1 + φ2) / 2);
	const y = R * (φ2 - φ1);
	
	return { x, y };
}

function load_auto_drive_waypoints(dirpath, callback){
	if (!fs.existsSync(dirpath)) {
		const drive_waypoints = {};
		if(callback){
			callback(drive_waypoints);
		}
	}else{
		fs.readdir(dirpath, { withFileTypes: true }, (err, entries) => {
			if (err) {
				console.error('Error reading directory:', err);
				return;
			}
			const drive_waypoints = {};
			entries.forEach(entry => {
				if (entry.isFile()) {
					if(path.extname(entry.name) == ".pif"){
						const fullPath = path.join(dirpath, entry.name);
						pif_utils.read_pif(fullPath, (file_path, result) => {
							const meta = result[1].toString('utf-8');
							const frame_dom = xml_parser.parse(meta);
	
							const fileNameWithoutExt = path.basename(file_path, path.extname(file_path));
							drive_waypoints[fileNameWithoutExt] = {
								nmea : frame_dom['picam360:frame']['passthrough:nmea'],
								encoder : frame_dom['picam360:frame']['passthrough:encoder'],
								imu : frame_dom['picam360:frame']['passthrough:imu'],
								image : `/auto_drive_waypoints/${entry.name}.0.0.jpeg`,
							};
						});
						//console.log(`File: ${entry.name}`);
					}
				} else if (entry.isDirectory()) {
					//console.log(`Directory: ${entry.name}`);
				}
			});
			if(callback){
				callback(drive_waypoints);
			}
		});
	}
}

function record_waypoints_handler(tmp_img){
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
	let timestamp = img_dom["picam360:image"]['timestamp'];
	if(timestamp.includes(',')){
		const ary = timestamp.split(',');
		timestamp = `${ary[0]}.${ary[1].padStart(6, '0')}`;
	}
	const pif_filepath = `${m_options.data_filepath}/waypoint_images/${timestamp}.pif`;
	fs.writeFile(pif_filepath, data, (err) => {
		if (err) {
			console.error('pif file dump faild', err);
		}
	});
	const jpeg_filepath = pif_filepath + ".0.0.JPEG";
	fs.writeFile(jpeg_filepath, tmp_img[2], (err) => {
		if (err) {
			console.error('jpeg file dump faild', err);
		}
	});

	if(m_options.debug){
		console.log(`${pif_filepath} recorded.`);
	}
}

// Calculate the distance using the Haversine formula
function calculateDistance(lat1, lon1, lat2, lon2) {
    const R = 6371000; // Radius of the Earth in meters
    const φ1 = degreesToRadians(lat1);
    const φ2 = degreesToRadians(lat2);
    const Δφ = degreesToRadians(lat2 - lat1);
    const Δλ = degreesToRadians(lon2 - lon1);

    const a = Math.sin(Δφ / 2) ** 2 +
              Math.cos(φ1) * Math.cos(φ2) * Math.sin(Δλ / 2) ** 2;
    const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));

    return R * c; // Distance in meters
}

// Calculate the bearing from the current location to the target
function calculateBearing(lat1, lon1, lat2, lon2) {
    const φ1 = degreesToRadians(lat1);
    const φ2 = degreesToRadians(lat2);
    const Δλ = degreesToRadians(lon2 - lon1);

    const y = Math.sin(Δλ) * Math.cos(φ2);
    const x = Math.cos(φ1) * Math.sin(φ2) -
              Math.sin(φ1) * Math.cos(φ2) * Math.cos(Δλ);
    const θ = Math.atan2(y, x);

    return (radiansToDegrees(θ) + 360) % 360; // Bearing in degrees
}

// Convert degrees to radians
function degreesToRadians(degrees) {
    return degrees * (Math.PI / 180);
}

// Convert radians to degrees
function radiansToDegrees(radians) {
    return radians * (180 / Math.PI);
}

function move_forward(distance) {
    console.log(`Moving forward ${distance.toFixed(2)} meters`);
	if(distance > 0){
		m_client.publish('pserver-vehicle-wheel', 'CMD move_forward');
	}else{
		m_client.publish('pserver-vehicle-wheel', 'CMD move_backward');
	}
}

function rotate_robot(angle) {
    console.log(`Rotating ${angle.toFixed(2)} degrees`);

	if(angle > 0){
		m_client.publish('pserver-vehicle-wheel', 'CMD turn_right');
	}else{
		m_client.publish('pserver-vehicle-wheel', 'CMD turn_left');
	}
}
function update_auto_drive_cur(cur) {
	m_auto_drive_cur = cur;
	m_client.set('pserver-auto-drive-cur', m_auto_drive_cur).then((data) => {
		console.log('set auto drive cur', data);
	});
}
function update_auto_drive_waypoints(waypoints) {
	m_auto_drive_waypoints = waypoints;
	m_client.set('pserver-auto-drive-waypoints', JSON.stringify(waypoints)).then((data) => {
		console.log('set auto drive waypoints', data);
	});
}

function auto_drive_handler(tmp_img){
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
	const frame_dom = xml_parser.parse(meta);
	const current_nmea = nmea.parseNmeaSentence(frame_dom['picam360:frame']['passthrough:nmea']);
	const current_imu = JSON.parse(frame_dom['picam360:frame']['passthrough:imu']);

	const keys = Object.keys(m_auto_drive_waypoints);
    if (m_auto_drive_cur >= keys.length) {
        console.log('All waypoints reached!');

		m_client.publish('pserver-auto-drive-info', JSON.stringify({
			"mode" : "AUTO",
			"state" : "DONE",
			"waypoint_distance" : "-",
			"heading_error" : "-",
		}));
        return;
    }

	let cur = m_auto_drive_cur;
	while(cur < keys.length){
		const key = keys[cur];
		const target_waypoint = m_auto_drive_waypoints[key];
		const target_nmea = nmea.parseNmeaSentence(target_waypoint['nmea']);
		const distanceToTarget = calculateDistance(
			current_nmea.latitude, current_nmea.longitude,
			target_nmea.latitude, target_nmea.longitude);
		const targetHeading = calculateBearing(
			current_nmea.latitude, current_nmea.longitude,
			target_nmea.latitude, target_nmea.longitude);
		let headingError = targetHeading - current_imu.heading;
		if(headingError <= -180){
			headingError += 360;
		}else if(headingError > 180){
			headingError -= 360;
		}
	
		if (distanceToTarget > 0.5) {
			// Control logic: move forward/backward or rotate
			if (Math.abs(headingError) > 10) { // 10 degrees threshold
				rotate_robot(headingError);
			} else {
				move_forward(distanceToTarget);
			}
	
			m_client.publish('pserver-auto-drive-info', JSON.stringify({
				"mode" : "AUTO",
				"state" : "DRIVING",
				"waypoint_distance" : distanceToTarget,
				"heading_error" : headingError,
			}));
			break;
		}

		cur++;
	}

	console.log(`Reached waypoint ${cur}`);
	update_auto_drive_cur(cur);
}

function main() {
    const argv = yargs
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
		
		const pif_dirpath = `${m_options.data_filepath}/waypoint_images`;
		load_auto_drive_waypoints(pif_dirpath, (waypoints) => {
			update_auto_drive_waypoints(waypoints);
			update_auto_drive_cur(0);
		});
	});

	const subscriber = client.duplicate();
	subscriber.connect().then(() => {
		console.log('redis connected:');

		subscriber.subscribe('pserver-auto-drive', (data, key) => {
			const params = data.trim().split(' ');
			switch (params[0]) {
				case "CMD":
					console.log(`"${data}" subscribed.`);
					command_handler(data.substr(params[0].length + 1));
					break;
			}
		});

		subscriber.subscribe('pserver-nmea', (data, key) => {
			const parsedData = nmea.parseNmeaSentence(data);

			if(m_averaging_count == 0){
				m_averaging_nmea = parsedData;
			}else{
				m_averaging_nmea.latitude += parsedData.latitude;
				m_averaging_nmea.longitude += parsedData.longitude;
			}
			m_averaging_count++;
			
			if(m_averaging_count == 10){
				m_averaging_nmea.latitude /= m_averaging_count;
				m_averaging_nmea.longitude /= m_averaging_count;
				m_averaging_count = 0;

				push_nmea(m_averaging_nmea);
			}
		});

		let tmp_img = [];
		subscriber.subscribe('pserver-vslam-pst', (data, key) => {
			if(data.length == 0 && tmp_img.length != 0){
				switch(m_drive_mode){
				case "RECORD":
					record_waypoints_handler(tmp_img);
					break;
				case "AUTO":
					auto_drive_handler(tmp_img);
					break;
				}
				tmp_img = [];
			}else{
				tmp_img.push(Buffer.from(data, 'base64'));
			}
		});
	});
}
function command_handler(cmd) {
	let split = cmd.split(' ');
	switch(split[0]){
		case "START_RECORD":
			if(m_drive_mode == "STANBY") {
				const pif_dirpath = `${m_options.data_filepath}/waypoint_images`;
				let succeeded = false;
				if (!fs.existsSync(pif_dirpath)) {
					try {
						fs.mkdirSync(pif_dirpath, { recursive: true });
						succeeded = true;
					} catch (err) {
						console.log(err);
					}
				}else{
					succeeded = true;
				}
				if(succeeded){
					m_drive_mode = "RECORD";
				}
			}
			console.log("drive mode", m_drive_mode);
			break;
		case "STOP_RECORD":
			m_drive_mode = "STANBY";
			execSync('sync');
			console.log("drive mode", m_drive_mode);
			break;
		case "START_AUTO":
			if(m_drive_mode == "STANBY") {
				const pif_dirpath = `${m_options.data_filepath}/waypoint_images`;
				load_auto_drive_waypoints(pif_dirpath, (waypoints) => {
					m_drive_mode = "AUTO";
					update_auto_drive_waypoints(waypoints);
					update_auto_drive_cur(0);
					console.log("drive mode", m_drive_mode);
				});
			}else{
				console.log("drive mode", m_drive_mode);
			}
			break;
		case "STOP_AUTO":
			m_drive_mode = "STANBY";
			console.log("drive mode", m_drive_mode);
			break;
	}
}
function push_nmea(nmea){
	if(!m_last_nmea){
		m_last_nmea = m_averaging_nmea;
		return;
	}

	const { x, y } = latLonToXY(
		m_last_nmea.latitude, m_last_nmea.longitude,
		m_averaging_nmea.latitude, m_averaging_nmea.longitude);
	console.log(`X: ${x} meters, Y: ${y} meters`);

	m_last_nmea = m_averaging_nmea;
}

if (require.main === module) {
    main();
}
