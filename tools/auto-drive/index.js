const { J } = require('quaternion');
console.log("auto-drive");
const fs = require("fs");
const path = require("path");
const nmea = require('nmea-simple');
const utm = require('utm');
const { execSync } = require('child_process');
const { spawn } = require('child_process');
const yargs = require('yargs');
const fxp = require('fast-xml-parser');
const xml_parser = new fxp.XMLParser({
	ignoreAttributes: false,
	attributeNamePrefix: "",
});
const pif_utils = require('./pif-utils');
const jsonc = require('jsonc-parser');
const cv = require('opencv4nodejs');

const { GpsOdometry } = require('./odometry-handlers/gps-odometry');
const { EncoderOdometry } = require('./odometry-handlers/encoder-odometry');
const { VslamOdometry } = require('./odometry-handlers/vslam-odometry');

let m_options = {
	"waypoint_threshold_m": 10,
	"data_filepath": "/home/picam360/.auto-drive/auto-drive-waypoints",
	"reverse": false,
	"vord_enabled": true,
	"vord_debug": false,
	"depth_debug": true,
	"depth_binning": 2,

	//juki
	"tolerance_distance": 2.0,
	"tolerance_shift": 0.5,
	"forward_pwm_base": 50,
	"backward_pwm_base": 46,
	"lr_ratio_forward": 1.0,
	"lr_ratio_backward": 1.0,
	"pwm_range": 10,
	"pwm_control_gain": 0.06,
	"move_forward_camera": "backward",
	"move_backward_camera": "backward",
	"cameras": {
		"forward": {
			"cam_offset" : {
				"x" : 0.0,
				"y" : 2.2,
				//"y" : 2.228590172834311,
				"heading" : 0,
				//"heading" : -8.5,
			},
			"cam_heading" : 0,//physical
			"cam_height" : 1.0,
			"pst_channel": "pserver-forward-pst",
			"check_person_detected": true,
			"stereo_distance": 0.06,
		},
		"backward": {
			"cam_offset" : {
				"x" : 0.0,
				"y" : 2.2,
				//"y" : 2.228590172834311,
				"heading" : 0,
				//"heading" : -8.5,
			},
			"cam_heading" : 180,//physical
			"cam_height" : 1.0,
			"pst_channel": "pserver-backward-pst",
			"check_person_detected": true,
			"stereo_distance": 0.06,
		}
	},
	"tracking": {
		"tolerance_depth": 2.0,
		"min_object_height": 1.0,
	},

	// //for jetchariot
	// "tolerance_distance" : 0.1,
	// "tolerance_shift": 0.1,
	// "forward_pwm_base" : 75,
	// "backward_pwm_base" : 75,
	// "lr_ratio_forward" : 0.999,
	// "lr_ratio_backward" : 1.050,
	// "pwm_range" : 75,
	// "pwm_control_gain" : 0.06,
	// "move_forward_camera": "forward",
	// "move_backward_camera": "forward",
	// "cameras": {
	// 	"forward": {
	// 		"cam_offset" : {
	// 			"x" : 0.0,
	// 			"y" : 2.2,
	// 			//"y" : 2.228590172834311,
	// 			"heading" : 0,
	// 			//"heading" : -8.5,
	// 		},
	// 		"cam_heading" : 0,//physical
	//		"cam_height" : 0.13,
	// 		"pst_channel": "pserver-forward-pst",
	//		"check_person_detected": true,
	//		"stereo_distance": 0.03,
	// 	},
	// 	"backward": {
	// 		"cam_offset" : {
	// 			"x" : 0.0,
	// 			"y" : 2.2,
	// 			//"y" : 2.228590172834311,
	// 			"heading" : 0,
	// 			//"heading" : -8.5,
	// 		},
	// 		"cam_heading" : 180,//physical
	//		"cam_height" : 0.13,
	// 		"pst_channel": "pserver-backward-pst",
	//		"check_person_detected": false,
	//		"stereo_distance": 0.03,
	// 	}
	// },
};
let m_argv = null;
let m_socket = null;
let m_drive_mode = "STANBY";
let m_drive_submode = "";
let m_averaging_nmea = null;
let m_averaging_count = 0;
let m_last_nmea = null;
let m_record_pif_dirpath = "";
let m_auto_drive_ready_first_launch = true;
let m_auto_drive_ready = false;
let m_auto_drive_last_reverse = false;
let m_auto_drive_waypoints = null;
let m_auto_drive_cur = -1;//negative value means init state
let m_auto_drive_direction = "forward";//forward or backward
let m_auto_drive_heading_tuning = false;
let m_auto_drive_last_state = 0;
let m_auto_drive_last_lastdistance = 0;
let m_vord_person = {
	"state": 0,
	"forward": {
		"objects": [],
		"st": 0,
		"et": 0,
	},
	"backward": {
		"objects": [],
		"st": 0,
		"et": 0,
	},
};
let m_vord_tree = {
	"state": 0,
	"tree": {
		"objects": [],
		"st": 0,
		"et": 0,
	},
};
let m_depth = {
	"state": 0,
	"forward": {
		"disparity": null,
		"st": 0,
		"et": 0,
	},
	"backward": {
		"disparity": null,
		"st": 0,
		"et": 0,
	},
};
const ODOMETRY_TYPE = {
	GPS: "GPS",
	ENCODER: "ENCODER",
	VSLAM: "VSLAM",
};
let m_odometry_conf = {
	odom_type: ODOMETRY_TYPE.VSLAM,
	//odom_type : ODOMETRY_TYPE.ENCODER,
	GPS: {
		enabled: true
	},
	ENCODER: {
		enabled: true
	},
	VSLAM: {
		enabled: true
	},
};

function launchVordTree() {
	const vord_path = "/home/picam360/github/picam360-vord";
	const vord_options = "";
	const command = `
		source /home/picam360/miniconda3/etc/profile.d/conda.sh && \
		conda activate perceptree && \
		python ${vord_path}/vord-tree.py ${vord_options}
	`;
	const vslam_process = spawn(command, { shell: '/bin/bash', cwd: path.resolve(vord_path) });
	vslam_process.stdout.on('data', (data) => {
		//console.log(`PICAM360_VORD STDOUT: ${data}`);
	});

	vslam_process.stderr.on('data', (data) => {
		console.error(`PICAM360_VORD STDERR: ${data}`);
	});

	vslam_process.on('close', (code) => {
		console.log(`PICAM360_VORD STDOUT CLOSED(: ${code})`);
	});
}
function killVordTree() {
	const processName = "picam360-vord-tree";

	try {
		const output = execSync(`ps -eo pid,comm,args`, { encoding: "utf-8" });

		const lines = output.trim().split("\n").slice(1);

		const matchingPids = lines
			.map(line => line.trim().split(/\s+/, 3)) // [pid, comm, args]
			.filter(([pid, comm, args]) =>
				comm === processName || args.split(" ")[0] === processName
			)
			.map(([pid]) => parseInt(pid));

		// kill
		for (const pid of matchingPids) {
			console.log(`Killing PID ${pid} (${processName})`);
			process.kill(pid);
		}

		if (matchingPids.length === 0) {
			console.log(`No matching process found for "${processName}"`);
		}

	} catch (err) {
		console.error("Error while killing process:", err.message);
	}
}

function launchVordPerson() {
	const vord_path = "/home/picam360/github/picam360-vord";
	const vord_options = "";
	const command = `
		source /home/picam360/miniconda3/etc/profile.d/conda.sh && \
		conda activate yolo_v8_py310 && \
		PYTHONPATH=/usr/lib/python3.10/dist-packages:$PYTHONPATH python ${vord_path}/vord-person.py ${vord_options}
	`;
	const vslam_process = spawn(command, { shell: '/bin/bash', cwd: path.resolve(vord_path) });
	vslam_process.stdout.on('data', (data) => {
		//console.log(`PICAM360_VORD STDOUT: ${data}`);
	});

	vslam_process.stderr.on('data', (data) => {
		console.error(`PICAM360_VORD STDERR: ${data}`);
	});

	vslam_process.on('close', (code) => {
		console.log(`PICAM360_VORD STDOUT CLOSED(: ${code})`);
	});
}
function killVordPerson() {
	const processName = "picam360-vord-person";

	try {
		const output = execSync(`ps -eo pid,comm,args`, { encoding: "utf-8" });

		const lines = output.trim().split("\n").slice(1);

		const matchingPids = lines
			.map(line => line.trim().split(/\s+/, 3)) // [pid, comm, args]
			.filter(([pid, comm, args]) =>
				comm === processName || args.split(" ")[0] === processName
			)
			.map(([pid]) => parseInt(pid));

		// kill
		for (const pid of matchingPids) {
			console.log(`Killing PID ${pid} (${processName})`);
			process.kill(pid);
		}

		if (matchingPids.length === 0) {
			console.log(`No matching process found for "${processName}"`);
		}

	} catch (err) {
		console.error("Error while killing process:", err.message);
	}
}

function launchDepth() {
	const vord_path = "/home/picam360/github/picam360-depth";
	const vord_options = "";
	const command = `
		source /home/picam360/miniconda3/etc/profile.d/conda.sh && \
		conda activate raft_stereo_py310 && \
		python ${vord_path}/raft-stereo.py ${vord_options}
	`;
	// const command = `
	// 	source /home/picam360/miniconda3/etc/profile.d/conda.sh && \
	// 	conda activate raft_stereo_py310 && \
	// 	python ${vord_path}/stereo-sgbm.py ${vord_options}
	// `;
	const vslam_process = spawn(command, { shell: '/bin/bash', cwd: path.resolve(vord_path) });
	vslam_process.stdout.on('data', (data) => {
		console.log(`PICAM360_DEPTH STDOUT: ${data}`);
	});

	vslam_process.stderr.on('data', (data) => {
		console.error(`PICAM360_DEPTH STDERR: ${data}`);
	});

	vslam_process.on('close', (code) => {
		console.log(`PICAM360_DEPTH STDOUT CLOSED(: ${code})`);
	});
}
function killDepth() {
	const processName = "picam360-depth";

	try {
		const output = execSync(`ps -eo pid,comm,args`, { encoding: "utf-8" });

		const lines = output.trim().split("\n").slice(1);

		const matchingPids = lines
			.map(line => line.trim().split(/\s+/, 3)) // [pid, comm, args]
			.filter(([pid, comm, args]) =>
				comm === processName || args.split(" ")[0] === processName
			)
			.map(([pid]) => parseInt(pid));

		// kill
		for (const pid of matchingPids) {
			console.log(`Killing PID ${pid} (${processName})`);
			process.kill(pid);
		}

		if (matchingPids.length === 0) {
			console.log(`No matching process found for "${processName}"`);
		}

	} catch (err) {
		console.error("Error while killing process:", err.message);
	}
}

function latLonToXY(lat1, lon1, lat2, lon2) {
	const R = 6378137; // Earth's radius in meters

	// Convert latitude and longitude to radians
	const phi1 = lat1 * Math.PI / 180;
	const phi2 = lat2 * Math.PI / 180;
	const dlamda = (lon2 - lon1) * Math.PI / 180;

	// Calculate XY coordinates
	const x = R * dlamda * Math.cos((phi1 + phi2) / 2);
	const y = R * (phi2 - phi1);

	return { x, y };
}

function load_auto_drive_waypoints_ext(dirpath, idx, drive_waypoints, callback) {
	drive_waypoints = drive_waypoints || {};
	const dirpath_ext = dirpath + (idx ? `_ext_${idx}` : "");
	if (!fs.existsSync(dirpath_ext)) {
		if (callback) {
			callback(drive_waypoints);
		}
	} else {
		load_auto_drive_waypoints(dirpath_ext, (waypoints) => {
			drive_waypoints = Object.assign(drive_waypoints, waypoints);
			load_auto_drive_waypoints_ext(dirpath, idx + 1, drive_waypoints, callback);
		});
	}
}

function load_auto_drive_waypoints(dirpath, callback) {
	if (!fs.existsSync(dirpath)) {
		const drive_waypoints = {};
		if (callback) {
			callback(drive_waypoints);
		}
	} else {
		fs.readdir(dirpath, { withFileTypes: true }, (err, entries) => {
			if (err) {
				console.error('Error reading directory:', err);
				return;
			}
			const drive_waypoints = {};
			entries.forEach(entry => {
				if (entry.isFile()) {
					if (path.extname(entry.name) == ".pif") {
						const fullPath = path.join(dirpath, entry.name);
						pif_utils.read_pif(fullPath, (file_path, result) => {
							const meta = result[1].toString('utf-8');
							const frame_dom = xml_parser.parse(meta);

							const fileNameWithoutExt = path.basename(file_path, path.extname(file_path));
							drive_waypoints[fileNameWithoutExt] = {
								meta,
								nmea: frame_dom['picam360:frame']['passthrough:nmea'],
								encoder: frame_dom['picam360:frame']['passthrough:encoder'],
								imu: frame_dom['picam360:frame']['passthrough:imu'],
								jpeg_filepath: `${dirpath}/${entry.name}.0.0.JPEG`,
							};
						});
						//console.log(`File: ${entry.name}`);
					}
				} else if (entry.isDirectory()) {
					//console.log(`Directory: ${entry.name}`);
				}
			});
			if (callback) {
				callback(drive_waypoints);
			}
		});
	}
}

function reindex_waypoints(drive_waypoints, reverse) {
	const waypoints = [];
	for (const key of Object.keys(drive_waypoints)) {
		waypoints.push(drive_waypoints[key]);
	}
	if (reverse) {
		waypoints.reverse();
	}
	return waypoints;
}

function record_waypoints_handler(tmp_img) {
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

	const meta_size = parseInt(img_dom["picam360:image"].meta_size, 10);
	const meta = data.slice(4 + header_size, 4 + header_size + meta_size);

	let timestamp = img_dom["picam360:image"]['timestamp'];
	if (timestamp.includes(',')) {
		const ary = timestamp.split(',');
		timestamp = `${ary[0]}.${ary[1].padStart(6, '0')}`;
	}
	const pif_filepath = `${m_record_pif_dirpath}/${timestamp}.pif`;
	try {
		fs.writeFileSync(pif_filepath, data);
	} catch (err) {
		console.error('pif file dump failed', err);
		return;
	}
	const jpeg_data = tmp_img[2];
	const jpeg_filepath = pif_filepath + ".0.0.JPEG";

	m_client.publish('pserver-auto-drive-info', JSON.stringify({
		"mode": "RECORD",
		"state": "RECORDING",
		"pif_filepath": pif_filepath,
	}));

	try {
		fs.writeFileSync(jpeg_filepath, jpeg_data);
	} catch (err) {
		console.error('jpeg file dump faild', err);
		return;
	}

	if (m_options.debug) {
		console.log(`${pif_filepath} recorded.`);
	}
}

//START OF TRACKING CODE
function countNonZero(arr) {
  return arr.filter(x => x !== 0).length;
}
function medianIndex(arr) {
	const filtered = arr
		.map((v, i) => ({ v, i }))
		.filter(obj => obj.v !== 0);

	if (filtered.length === 0) return -1;

	filtered.sort((a, b) => a.v - b.v);

	const mid = Math.floor(filtered.length / 2);
	if (filtered.length % 2 === 0) {
		return Math.round((filtered[mid - 1].i + filtered[mid].i) / 2);
	} else {
		return filtered[mid].i;
	}
}

function getBest(objects, minCount) {
  // score
  const scores = objects.map(o => o.score);
  const nonZeroCount = countNonZero(scores);

  if (nonZeroCount <= minCount) {
	return null;
  }

  // return maximum
  return objects.reduce((max, obj) =>
	obj.score > max.score ? obj : max
  );
}

function pixelToAngle(dx, width = 512, fovDeg = 120) {
	const center = width / 2;
	const halfFovRad = (fovDeg / 2) * Math.PI / 180;

	const f = center / Math.tan(halfFovRad);

	const angleRad = Math.atan(dx / f);
	return angleRad * 180 / Math.PI;
}

function check_person_detected(direction) {
	for(const obj of m_vord_person[direction].objects){
		if(obj.label == "person"){
			return true;
		}
	}
	return false;
}

function gen_tracking_waypoints(obj_distance, obj_heading){

	const settings = EncoderOdometry.settings;
	const dtheta = obj_heading * Math.PI / 180.0;
	const waypoints = {};
	const step = 100;
	waypoints[0] = {
		encoder : JSON.stringify({
			left : 0,
			right : 0,
		})
	};
	for(let i=0;i<=step;i++){
		const d = i * obj_distance / step;
		const distance_left = d - dtheta * settings.wheel_separation / 2;
		const distance_right = d + dtheta * settings.wheel_separation / 2;
		waypoints[i+1] = {
			encoder : JSON.stringify({
				left : distance_left / settings.meter_per_pulse * settings.left_direction / settings.lr_ratio_forward,
				right : distance_right / settings.meter_per_pulse * settings.right_direction,
			})
		};
	}
	return waypoints;
}

function gen_tracking_roi(disparity_map, f, cam_height, wheel_separation, tolerance_depth, min_object_height){

	const ws_pix = f * wheel_separation / tolerance_depth;
	const ch_pix = f * (cam_height - min_object_height) / tolerance_depth;

	let x = (disparity_map.cols - ws_pix) / 2;
	let w = ws_pix;
	//above the horizon
	let y = 0;
	let h = disparity_map.rows / 2 + ch_pix;

	const cutoff = 10;
	x = Math.max(x, cutoff);
	w = Math.min(w, disparity_map.cols - cutoff*2);
	y = Math.max(y, 0);
	h = Math.min(h, disparity_map.rows);
	
	const roi_orig = disparity_map.getRegion(new cv.Rect(x, y, w, h));
	const roi = roi_orig.gaussianBlur(
		new cv.Size(1, 15),
		0,
		1.0   // sigmaY
	  );

	return roi;
}

function tracking_handler(direction) {
	if (!direction) {
		stop_robot();
		return;
	}

	const finalize_fnc = () => {
		console.log("tracking : done");
		stop_robot();
		setTimeout(() => {
			stop_robot();
		}, 200);
		m_drive_submode = "";
		return;
	};

	if (m_odometry_conf[ODOMETRY_TYPE.ENCODER].handler == null) {
		return;//fail safe : not ready or finished or something wrong
	}

	const tolerance_depth = m_options["tracking"]["tolerance_depth"];
	//const wheel_separation = m_options["encoder_odom"]["wheel_separation"];
	const wheel_separation = 2.0;
	const cam_height = m_options.cameras[direction].cam_height;
	const min_object_height = m_options["tracking"]["min_object_height"];

	const sd = m_options.cameras[direction].stereo_distance;
	const disparity_map = m_depth[direction].disparity;
	const min_val = m_depth[direction].min_val;
	const max_val = m_depth[direction].max_val;
	const uint16_max = ((1 << 16) - 1);
	const fov_deg = 90;
	const fov_rad = fov_deg * Math.PI / 180;
	const f  = (disparity_map.cols / 2) / Math.tan(fov_rad / 2);

	{
		const roi = gen_tracking_roi(
			disparity_map, f, 
			cam_height, wheel_separation,
			tolerance_depth, min_object_height);
		const { maxVal, maxLoc, minVal, minLoc } = roi.minMaxLoc();
		const disparity = maxVal / uint16_max * (max_val - min_val) + min_val;

		if(disparity < 0){
			stop_robot();
			console.log("disparity is invalid");
			return;
		}
		const depth = sd * f / disparity;
		console.log(`nearest object is ${depth}m : ${roi.cols}pix,${roi.rows}pix, xy=${maxLoc.x},${maxLoc.y}`);
		if(depth < tolerance_depth){
			m_odometry_conf[ODOMETRY_TYPE.ENCODER].handler.in_tolerance_count++;

			let norm = roi.normalize(0, 255,cv.NORM_MINMAX,cv.CV_8U);
			const color = cv.applyColorMap(norm, cv.COLORMAP_JET);
			color.drawCircle(new cv.Point(maxLoc.x, maxLoc.y), 3,   new cv.Vec(255, 255, 255), -1);
			color.putText(
				`${depth}m`,
				new cv.Point(maxLoc.x, Math.max(0, maxLoc.y - 10)),
				cv.FONT_HERSHEY_SIMPLEX,
				0.4,
				new cv.Vec(255, 255, 255),
				1,
				cv.LINE_AA);
			cv.imwrite(`tracking_in_tolerance${m_odometry_conf[ODOMETRY_TYPE.ENCODER].handler.in_tolerance_count}.png`, color);

			if(m_odometry_conf[ODOMETRY_TYPE.ENCODER].handler.in_tolerance_count >= 10){
				finalize_fnc();
				return;
			}
		}else{
			m_odometry_conf[ODOMETRY_TYPE.ENCODER].handler.in_tolerance_count = 0;
		}
	}

	{
		const centering_depth = 3;
		const roi = gen_tracking_roi(
			disparity_map, f, 
			cam_height, wheel_separation,
			centering_depth, min_object_height);
		const { maxVal, maxLoc, minVal, minLoc } = roi.minMaxLoc();
		const disparity = maxVal / uint16_max * (max_val - min_val) + min_val;

		if(disparity < 0){
			stop_robot();
			console.log("disparity is invalid");
			return;
		}
		const depth = sd * f / disparity;

		const elapsed_from_last_wapoints_updated = Date.now() - (m_odometry_conf[ODOMETRY_TYPE.ENCODER].handler.last_waypoints_updated_ms || 0);
		if(elapsed_from_last_wapoints_updated > 250 && depth < centering_depth && m_auto_drive_cur > 0){//tune
			const shift_pix = maxLoc.x - w/2;
			const gain = 0.01;
			const tune = shift_pix * gain;
			m_odometry_conf[ODOMETRY_TYPE.ENCODER].handler.encoder_params.heading -= tune;
			console.log("TRACKING_DEBUG", m_odometry_conf[ODOMETRY_TYPE.ENCODER].handler.encoder_params.heading, tune, shift_pix, depth);

			m_odometry_conf[ODOMETRY_TYPE.ENCODER].handler.last_waypoints_updated_ms = Date.now();
		}
	}

	auto_drive_handler(ODOMETRY_TYPE.ENCODER, finalize_fnc);
}
//END OF TRACKING CODE

function move_robot(distance) {
	console.log(`Moving forward ${distance.toFixed(2)} meters`);
	if (distance > 0) {
		m_client.publish('pserver-vehicle-wheel', 'CMD move_forward');
	} else {
		m_client.publish('pserver-vehicle-wheel', 'CMD move_backward');
	}
}

function move_pwm_robot(distance, angle, minus=0) {

	if (distance > 0) {
		m_auto_drive_direction = "forward";
	}else{
		m_auto_drive_direction = "backward";
	}

	let now = Date.now();
	
	if(m_options.cameras[m_auto_drive_direction].check_person_detected !== false){
		if(now - m_vord_person[m_auto_drive_direction].et > 3000){
			console.log("Valid object tracking is required for safety.");
			stop_robot();
			return;
		}else if(check_person_detected(m_auto_drive_direction)){
			console.log("Person detected. System stop vehicle for safety");
			stop_robot();
			return;
		}
	}

	if (m_auto_drive_direction == "forward") {
		const left_minus = Math.min(m_options.pwm_range, angle < 0 ? m_options.pwm_range * Math.abs(angle) * m_options.pwm_control_gain : 0);
		const right_minus = Math.min(m_options.pwm_range, angle > 0 ? m_options.pwm_range * Math.abs(angle) * m_options.pwm_control_gain : 0);
		const left_pwd = m_options.forward_pwm_base - Math.round(left_minus) - minus;
		const right_pwd = m_options.forward_pwm_base - Math.round(right_minus) - minus;
		console.log("move_forward_pwm", distance, angle, left_minus, right_minus, left_pwd, right_pwd);
		m_client.publish('pserver-vehicle-wheel', `CMD move_forward_pwm ${left_pwd * m_options.lr_ratio_forward} ${right_pwd} ${now}`);
	} else if (m_auto_drive_direction == "backward") {
		const left_minus = Math.min(m_options.pwm_range, angle > 0 ? m_options.pwm_range * Math.abs(angle) * m_options.pwm_control_gain : 0);
		const right_minus = Math.min(m_options.pwm_range, angle < 0 ? m_options.pwm_range * Math.abs(angle) * m_options.pwm_control_gain : 0);
		const left_pwd = m_options.backward_pwm_base - Math.round(left_minus) - minus;
		const right_pwd = m_options.backward_pwm_base - Math.round(right_minus) - minus;
		console.log("move_backward_pwm", distance, angle, left_minus, right_minus, left_pwd, right_pwd);
		m_client.publish('pserver-vehicle-wheel', `CMD move_backward_pwm ${left_pwd * m_options.lr_ratio_backward} ${right_pwd} ${now}`);
	}
}

function rotate_robot(angle) {
	console.log(`Rotating ${angle.toFixed(2)} degrees`);

	if (angle > 0) {
		m_client.publish('pserver-vehicle-wheel', 'CMD turn_right');
	} else {
		m_client.publish('pserver-vehicle-wheel', 'CMD turn_left');
	}
}

function stop_robot() {
	console.log(`Stop vihecle!!`);

	m_client.publish('pserver-vehicle-wheel', 'CMD stop');
}

function update_auto_drive_cur(cur) {
	m_auto_drive_cur = cur;
	if (cur < 0) {
		m_auto_drive_last_state = 0;
		m_auto_drive_last_lastdistance = 0;
	}
	m_client.set('pserver-auto-drive-cur', m_auto_drive_cur).then((data) => {
		console.log('set auto drive cur ' + cur, data);
	});
}
function update_auto_drive_waypoints(waypoints) {
	m_auto_drive_waypoints = waypoints;
	m_client.set('pserver-auto-drive-waypoints', JSON.stringify(waypoints)).then((data) => {
		console.log('set auto drive waypoints', data);
	});

	m_client.publish('pserver-auto-drive-info', JSON.stringify({
		"mode": "INFO",
		"state": "WAYPOINT_UPDATED",
	}));
}

function push_handler(tmp_img) {
	const rets = {};
	if (tmp_img.length != 3) {
		throw new Error('Invalid pif format');
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
	for (const key of conf_keys) {
		let ret = false;
		const conf = m_odometry_conf[key];
		if (conf.handler) {
			ret = conf.handler.push(header, meta, tmp_img[2]);
		}
		rets[key] = ret;
	}
	return rets;
}

function auto_drive_handler(odom_type, arrived_fnc) {

	if (m_odometry_conf[odom_type].handler == null) {
		return;//fail safe : not ready or finished or something wrong
	}

	const conf_keys = Object.keys(m_odometry_conf);

	const keys = Object.keys(m_auto_drive_waypoints.src);
	if (m_auto_drive_cur >= keys.length) {
		return;//fail safe : finished
	}

	if (keys.length > 0){//need to check ready
		if(m_odometry_conf[odom_type].handler.is_ready() == false) {
			return;
		}
	}

	let cur = m_auto_drive_cur;
	if(cur < 0){//init state
		cur = 0;
	}
	while (cur < keys.length) {
		for (const key of conf_keys) {
			const conf = m_odometry_conf[key];
			if (conf.handler && conf.handler.is_ready() && conf == m_odometry_conf[odom_type]) {
				const { x, y, heading, confidence } = conf.handler.getPosition();
				conf.x = x;
				conf.y = y;
				conf.heading = heading;
				conf.confidence = confidence;

				let [distanceToTarget, shiftToTarget] = conf.handler.calculateDistance(cur);
				let headingError = conf.handler.calculateHeadingError(cur);

				// if(Math.abs(headingError) > 90){//backward
				// 	distanceToTarget *= -1;
				// 	headingError = 180 - headingError;
				// 	if(headingError <= -180){
				// 		headingError += 360;
				// 	}else if(headingError > 180){
				// 		headingError -= 360;
				// 	}
				// 	headingError *= -1;
				// }

				conf.distanceToTarget = distanceToTarget;
				conf.shiftToTarget = shiftToTarget;
				conf.headingError = headingError;
			}
		}

		let distanceToTarget = m_odometry_conf[odom_type].distanceToTarget;
		let shiftToTarget = m_odometry_conf[odom_type].shiftToTarget;
		let headingError = m_odometry_conf[odom_type].headingError;

		let tolerance_distance = m_options.tolerance_distance;
		let tolerance_heading = (m_auto_drive_heading_tuning ? 999 : 999);
		if (cur == keys.length - 1) {
			tolerance_distance = m_options.tolerance_distance / 10;
			// switch(m_auto_drive_last_state){
			// case 0:
			// 	tolerance_heading = 1.0;
			// 	if(Math.abs(headingError) < 10.0){
			// 		m_auto_drive_last_state++;
			// 	}
			// 	break;
			// case 1:
			// 	if(Math.abs(distanceToTarget) < 0.3){
			// 		m_auto_drive_last_state++;
			// 		m_auto_drive_last_lastdistance = distanceToTarget;
			// 	}
			// 	break;
			// case 2:
			// 	if(Math.abs(distanceToTarget) > Math.abs(m_auto_drive_last_lastdistance)){//forcibly done
			// 		tolerance_distance = Math.abs(distanceToTarget) + 1.0;
			// 	}
			// 	m_auto_drive_last_lastdistance = distanceToTarget;
			// 	break;
			// }
		}
		if (Math.abs(distanceToTarget) > tolerance_distance) {

			// Control logic: move forward/backward or rotate
			if (Math.abs(headingError) > tolerance_heading) {
				rotate_robot(headingError);
				m_auto_drive_heading_tuning = true;
			} else {
				let tune = (distanceToTarget > 0 ? -1 : 1) * shiftToTarget / m_options.tolerance_shift * 60;
				tune = Math.max(-90, Math.min(tune, 90));
				console.log("DEBUG pwm", headingError, tune);
				move_pwm_robot(distanceToTarget, headingError + tune);
				//move_robot(distanceToTarget);
				m_auto_drive_heading_tuning = false;
			}

			const handlers = {};
			for (const key of conf_keys) {
				const conf = m_odometry_conf[key];
				handlers[key] = {
					"x": conf.x,
					"y": conf.y,
					"heading": conf.heading,
					"confidence": conf.confidence,
					"waypoint_distance": conf.distanceToTarget,
					"waypoint_shift": conf.shiftToTarget,
					"heading_error": conf.headingError,
				}
			}
			m_client.publish('pserver-auto-drive-info', JSON.stringify({
				"mode": "AUTO",
				"state": "DRIVING",
				"waypoint_distance": distanceToTarget,
				"heading_error": headingError,
				handlers,
			}));
			break;
		}

		cur++;
	}

	if (cur != m_auto_drive_cur) {
		console.log(`Reached waypoint ${cur}`);
		update_auto_drive_cur(cur);
	}
	if (m_auto_drive_cur >= keys.length) {
		console.log('All waypoints reached!');
		stop_robot();

		m_client.publish('pserver-auto-drive-info', JSON.stringify({
			"mode": "AUTO",
			"state": "DONE",
			"waypoint_distance": "-",
			"heading_error": "-",
		}));

		arrived_fnc();
	}
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
		.option('reverse', {
			type: 'boolean',
			description: 'reverse',
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
	if (argv.reverse !== undefined) {
		m_options.reverse = argv.reverse;
	}

	if (m_options.gps_odom) {
		Object.assign(GpsOdometry.settings, m_options.gps_odom);
	}
	if (m_options.encoder_odom) {
		Object.assign(EncoderOdometry.settings, m_options.encoder_odom);
	}
	if (m_options.vslam_odom) {
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
				src: waypoints
			});
			update_auto_drive_cur(-1);
		});
	});

	const subscriber = client.duplicate();
	subscriber.connect().then(() => {
		console.log('redis subscriber connected:');

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

			if (m_averaging_count == 0) {
				m_averaging_nmea = parsedData;
			} else {
				m_averaging_nmea.latitude += parsedData.latitude;
				m_averaging_nmea.longitude += parsedData.longitude;
			}
			m_averaging_count++;

			if (m_averaging_count == 10) {
				m_averaging_nmea.latitude /= m_averaging_count;
				m_averaging_nmea.longitude /= m_averaging_count;
				m_averaging_count = 0;

				push_nmea(m_averaging_nmea);
			}
		});

		let last_vslam_pst_ts = Date.now();
		{

			let tmp_img = [];
			subscriber.subscribe(m_options.cameras.forward.pst_channel, (data, key) => {
				const direction = "forward";
				const now = Date.now();
				m_options.cameras[direction].last_pst_ts = now;
				if (data.length == 0 && tmp_img.length != 0) {
					if (tmp_img.length == 3) {
						const jpeg_data = tmp_img[2];

						if(m_options["vord_enabled"] 
							&& m_drive_mode == "STANBY"
							&& now - m_vord_tree["tree"].st > 1000){//1fps

							if (m_vord_tree.state == 1) {
								m_vord_tree.state = 2;
								m_vord_tree["tree"].st = now;

								m_client.publish('picam360-vord-tree', JSON.stringify({
									"cmd": "detect",
									"test": false,
									"show": m_options["vord_debug"],
									"jpeg_data": jpeg_data.toString("base64"),
								}));
							}
						}

						if(m_options["vord_enabled"] && m_auto_drive_direction == direction){
							if (m_vord_person.state == 1 && now - m_vord_person[direction].st > 100) {//10fps
								m_vord_person.state = 2;
								m_vord_person[direction].st = now;

								m_client.publish('picam360-vord-person', JSON.stringify({
									"cmd": "detect",
									"test": false,
									"show": m_options["vord_debug"],
									"jpeg_data": jpeg_data.toString("base64"),
									"user_data": { direction },
								}));
							}
							if (m_depth.state == 1 && now - m_depth[direction].st > 500) {//2fps
								m_depth.state = 2;
								m_depth[direction].st = now;

								m_client.publish('picam360-depth', JSON.stringify({
									"cmd": "disparity",
									"test": false,
									"show": m_options["depth_debug"],
									"binning": m_options["depth_binning"],
									"jpeg_data": jpeg_data.toString("base64"),
									"user_data": { direction },
								}));
							}
						}
					}

					if (m_drive_mode == "RECORD" && m_drive_submode == "TRACKING") {
						if(now - m_depth["forward"].et > 1000){
							tracking_handler(null);//stop_robot
						}else{
							tracking_handler("forward");
						}
					}

					if((m_auto_drive_direction == "forward" && m_options["move_forward_camera"] == direction)
						|| (m_auto_drive_direction == "backward" && m_options["move_backward_camera"] == direction)){
					
						last_vslam_pst_ts = now;
						switch (m_drive_mode) {
							case "RECORD":
								const ret = push_handler(tmp_img);
								if(ret[ODOMETRY_TYPE.ENCODER]){
									record_waypoints_handler(tmp_img);
								}else{
									m_client.publish('pserver-auto-drive-info', JSON.stringify({
										"mode": "RECORD",
										"state": "RECORDING",
										"pif_filepath": "",
									}));
								}
								break;
							case "AUTO":
								if (m_auto_drive_ready) {
									push_handler(tmp_img);
									auto_drive_handler(m_odometry_conf.odom_type, () => {
										command_handler("STOP_AUTO");
									});
								}
								break;
						}
					}

					tmp_img = [];
				} else {
					tmp_img.push(Buffer.from(data, 'base64'));
				}
			});
		}
		{
			let tmp_img = [];
			subscriber.subscribe(m_options.cameras.backward.pst_channel, (data, key) => {
				const direction = "backward";
				const now = Date.now();
				m_options.cameras[direction].last_pst_ts = now;
				if (data.length == 0 && tmp_img.length != 0) {
					if (tmp_img.length == 3) {
						const jpeg_data = tmp_img[2];

						if(m_options["vord_enabled"] && m_auto_drive_direction == direction){
							if (m_vord_person.state == 1 && now - m_vord_person[direction].st > 100) {//10fps
								m_vord_person.state = 2;
								m_vord_person[direction].st = now;
	
								m_client.publish('picam360-vord-person', JSON.stringify({
									"cmd": "detect",
									"test": false,
									"show": m_options["vord_debug"],
									"jpeg_data": jpeg_data.toString("base64"),
									"user_data": { direction },
								}));
							}
							if (m_depth.state == 1 && now - m_depth[direction].st > 500) {//2fps
								m_depth.state = 2;
								m_depth[direction].st = now;

								m_client.publish('picam360-depth', JSON.stringify({
									"cmd": "disparity",
									"test": false,
									"show": m_options["depth_debug"],
									"binning": m_options["depth_binning"],
									"jpeg_data": jpeg_data.toString("base64"),
									"user_data": { direction },
								}));
							}
						}
					}

					if((m_auto_drive_direction == "forward" && m_options["move_forward_camera"] == direction)
						|| (m_auto_drive_direction == "backward" && m_options["move_backward_camera"] == direction)){

						last_vslam_pst_ts = now;
						switch (m_drive_mode) {
							case "RECORD":
								const ret = push_handler(tmp_img);
								if(ret[ODOMETRY_TYPE.ENCODER]){
									record_waypoints_handler(tmp_img);
								}else{
									m_client.publish('pserver-auto-drive-info', JSON.stringify({
										"mode": "RECORD",
										"state": "RECORDING",
										"pif_filepath": "",
									}));
								}
								break;
							case "AUTO":
								if (m_auto_drive_ready) {
									push_handler(tmp_img);
									auto_drive_handler(m_odometry_conf.odom_type, () => {
										command_handler("STOP_AUTO");
									});
								}
								break;
						}
					}

					tmp_img = [];
				} else {
					tmp_img.push(Buffer.from(data, 'base64'));
				}
			});
		}

		subscriber.subscribe('picam360-vord-tree-output', (data, key) => {
			//console.log(data);
			const params = JSON.parse(data);

			if (params['type'] == 'info') {
				if (params['msg'] == 'startup') {
					m_vord_tree.state = 1;
				}
			} else if (params['type'] == 'detect') {
				const direction = "tree";
				const now = Date.now();
				m_vord_tree.state = 1;
				m_vord_tree[direction].et = now;
				m_vord_tree[direction].objects = params['objects'];
				const elapsed = m_vord_tree[direction].et - m_vord_tree[direction].st;
				console.log(`object_tracking@tree (${direction}) updated in ${elapsed}ms`);

				console.log(params['objects']);
			}
		});

		subscriber.subscribe('picam360-vord-person-output', (data, key) => {
			//console.log(data);
			const params = JSON.parse(data);

			if (params['type'] == 'info') {
				if (params['msg'] == 'startup') {
					m_vord_person.state = 1;
				}
			} else if (params['type'] == 'detect') {
				const direction = params['user_data'].direction;
				const now = Date.now();
				m_vord_person.state = 1;
				m_vord_person[direction].et = now;
				m_vord_person[direction].objects = params['objects'];
				const elapsed = m_vord_person[direction].et - m_vord_person[direction].st;
				//console.log(`object_tracking@person (${direction}) updated in ${elapsed}ms`);

				if(m_depth[direction].disparity != null){
					for(const obj of params['objects']){
						try{
							const sd = m_options.cameras[direction].stereo_distance;
							const binning = m_options["depth_binning"];
							const disparity_map = m_depth[direction].disparity;
							const fov_deg = 90;
							const fov_rad = fov_deg * Math.PI / 180;
							const f  = (disparity_map.cols / 2) / Math.tan(fov_rad / 2)
							
                            const b = Array.isArray(obj.bbox) ? obj.bbox : [0, 0, 0, 0];

                            let x = b[0] ?? 0;
                            let y = b[1] ?? 0;
                            let w = 0;
                            let h = 0;

                            if (b.length >= 4) {
                                // If b looks like [xmin, ymin, xmax, ymax]
                                const looksLikeXYXY = b[2] > x && b[3] > y;
                                if (looksLikeXYXY) {
                                    w = b[2] - x;
                                    h = b[3] - y;
                                } else {
                                    // Otherwise assume [x, y, w, h]
                                    w = b[2];
                                    h = b[3];
                                }
                            }

							x = x / binning;
							y = y / binning;
							w = w / binning;
							h = h / binning;

							if (x < 0 || y < 0 || w <= 0 || h <= 0 ||
								x + w > disparity_map.cols || y + h > disparity_map.rows) {
								throw new Error("invalid param: ROI out of image bounds");
							}
							const roi = disparity_map.getRegion(new cv.Rect(x, y, w, h));
							const data = roi.getDataAsArray().flat();
							data.sort((a, b) => a - b);
							const median = data[Math.floor(data.length / 2)];
							const min_val = m_depth[direction].min_val;
							const max_val = m_depth[direction].max_val;
							const uint16_max = ((1 << 16) - 1);
							const disparity = median / uint16_max * (max_val - min_val) + min_val;
							if(disparity > 0){
								const depth = sd * f / disparity;
								obj.depth = depth;
							}else{
								console.log("disparity is invalid");
							}
						}catch(err){
							console.log(err);
						}
					}
					m_client.publish('picam360-vord-person-output-ext', JSON.stringify(params));
				}
				//console.log(params['objects']);
			}
		});

		subscriber.subscribe('picam360-depth-output', (data, key) => {
			//console.log(data);
			const params = JSON.parse(data);

			if (params['type'] == 'info') {
				if (params['msg'] == 'startup') {
					m_depth.state = 1;
				}
			} else if (params['type'] == 'disparity') {
				const direction = params['user_data'].direction;
				const now = Date.now();
				m_depth.state = 1;
				m_depth[direction].et = now;
				const buffer = Buffer.from(params['disparity'], 'base64');
				m_depth[direction].disparity = cv.imdecode(buffer, cv.IMREAD_UNCHANGED);//16bit png
				m_depth[direction].min_val = params['min_val'];
				m_depth[direction].max_val = params['max_val'];
				const elapsed = m_depth[direction].et - m_depth[direction].st;
				//console.log(`depth (${direction}) updated in ${elapsed}ms`);
			}
		});

		if (m_options["vord_enabled"]) {
			killVordPerson();
			launchVordPerson();
			killVordTree();
			launchVordTree();
			killDepth();
			launchDepth();
		}

		setInterval(() => {

			const sysinfo = {};
			try {
				const path = '/sys/class/thermal/thermal_zone0/temp';
				const raw = fs.readFileSync(path, 'utf8');
				const tempC = parseInt(raw.trim()) / 1000;
				sysinfo.temp = tempC;
			} catch (err) {
				console.error('Error reading temperature:', err);
			}
			sysinfo.latest_confidence = VslamOdometry.status.latest_confidence;


			const elapsed = Date.now() - last_vslam_pst_ts;
			if (elapsed > 1000) {
				m_client.publish('pserver-auto-drive-info', JSON.stringify({
					"mode": m_drive_mode,
					"state": "WAITING_PST",
					"sysinfo": sysinfo,
				}));
			} else {
				m_client.publish('pserver-auto-drive-info', JSON.stringify({
					"mode": m_drive_mode,
					"state": "RECEIVING_PST",
					"sysinfo": sysinfo,
				}));
			}
		}, 1000);
	});
}
function command_handler(cmd) {
	let split = cmd.split(' ');
	switch (split[0]) {
		case "START_RECORD":
			stop_robot();
			if (m_drive_mode == "STANBY") {
				const options = (split[1] && split[1][0] == '{' ? JSON.parse(split[1]) : {
					extend : split[1] == "EXTEND",
					tracking : split[1] == "TRACKING",
				});

				let pif_dirpath = `${m_options.data_filepath}/waypoint_images`;
				let succeeded = false;
				if (fs.existsSync(pif_dirpath)) {
					if (options.extend) {
						for (let i = 1; ; i++) {
							const pif_dirpath_ext = `${pif_dirpath}_ext_${i}`;
							if (!fs.existsSync(pif_dirpath_ext)) {
								pif_dirpath = pif_dirpath_ext;
								break;
							}
						}
					} else {
						const now = new Date();
						const formattedDate = now.getFullYear() +
							String(now.getMonth() + 1).padStart(2, '0') +
							String(now.getDate()).padStart(2, '0') + "_" +
							String(now.getHours()).padStart(2, '0') +
							String(now.getMinutes()).padStart(2, '0') + "_" +
							String(now.getSeconds()).padStart(2, '0');
						try {
							fs.renameSync(m_options.data_filepath, `${m_options.data_filepath}.${formattedDate}`);
							console.log(`folder moved to ${m_options.data_filepath}.${formattedDate}`);
						} catch (err) {
							console.error('folder move faild:', err);
						}
					}
				}
				if (!fs.existsSync(pif_dirpath)) {
					try {
						fs.mkdirSync(pif_dirpath, { recursive: true });
						succeeded = true;
					} catch (err) {
						console.log(err);
					}
				} else {
					succeeded = true;
				}
				if (!succeeded) {
					break;
				}
				m_record_pif_dirpath = pif_dirpath;
				m_drive_mode = "RECORD";
				m_client.publish('pserver-auto-drive-info', JSON.stringify({
					"mode": "RECORD",
					"state": "START_RECORD",
				}));

				if(options.tracking){
					const obj_distance = options.distance || 10 + 10;
					const obj_heading = options.heading || 0;
					const waypoints = gen_tracking_waypoints(obj_distance, obj_heading);

					update_auto_drive_waypoints({
						src: waypoints
					});
					update_auto_drive_cur(-1);

					m_odometry_conf[ODOMETRY_TYPE.ENCODER].handler = new EncoderOdometry();
					m_odometry_conf[ODOMETRY_TYPE.ENCODER].handler.init(waypoints, () => {
						m_drive_submode = "TRACKING";
						m_auto_drive_direction = "forward";
					}, false);
					m_odometry_conf[ODOMETRY_TYPE.ENCODER].last_waypoints_updated_ms = Date.now();
					m_odometry_conf[ODOMETRY_TYPE.ENCODER].in_tolerance_count = 0;
				}else{
					m_odometry_conf[ODOMETRY_TYPE.ENCODER].handler = new EncoderOdometry();
					m_odometry_conf[ODOMETRY_TYPE.ENCODER].handler.init({}, () => {
					}, false);
				}

				if (m_odometry_conf.odom_type == ODOMETRY_TYPE.VSLAM) {
					const pif_dirpath = `${m_options.data_filepath}/waypoint_images`;
					load_auto_drive_waypoints_ext(pif_dirpath, 0, null, (waypoints) => {
						waypoints = reindex_waypoints(waypoints, m_options.reverse);

						VslamOdometry.clear_reconstruction();
						m_odometry_conf[ODOMETRY_TYPE.VSLAM].handler = new VslamOdometry({
							incremental_reconstruction: true,
							reverse: false,
							//host : m_argv.host,
							transforms_callback: (vslam_waypoints, active_points) => {
							},
						});
						m_odometry_conf[ODOMETRY_TYPE.VSLAM].handler.init(waypoints, () => {
							if (m_odometry_conf[ODOMETRY_TYPE.VSLAM].handler) {
								m_odometry_conf[ODOMETRY_TYPE.VSLAM].handler.deinit();
								m_odometry_conf[ODOMETRY_TYPE.VSLAM].handler = null;
							}
						});
					});
				}
			}
			console.log("drive mode", m_drive_mode);
			break;
		case "STOP_RECORD": {
			stop_robot();
			m_drive_mode = "STANBY";
			execSync('sync');
			const pif_dirpath = `${m_options.data_filepath}/waypoint_images`;
			load_auto_drive_waypoints_ext(pif_dirpath, 0, null, (waypoints) => {
				waypoints = reindex_waypoints(waypoints, m_options.reverse);

				update_auto_drive_waypoints({
					src: waypoints
				});
				update_auto_drive_cur(-1);
			});

			if (m_odometry_conf.odom_type == ODOMETRY_TYPE.VSLAM && m_odometry_conf[ODOMETRY_TYPE.VSLAM].handler) {
				m_odometry_conf[ODOMETRY_TYPE.VSLAM].handler.reconstruction_finalize();
				//pending deinit()
			}
			if (m_odometry_conf[ODOMETRY_TYPE.ENCODER].handler) {
				m_odometry_conf[ODOMETRY_TYPE.ENCODER].handler.deinit();
				m_odometry_conf[ODOMETRY_TYPE.ENCODER].handler = null;
			}

			m_client.publish('pserver-auto-drive-info', JSON.stringify({
				"mode": "RECORD",
				"state": "STOP_RECORD",
			}));
			console.log("drive mode", m_drive_mode);

			break;
		}
		case "START_AUTO":
			stop_robot();
			if (m_drive_mode == "STANBY") {
				m_options.reverse = (split[1] == "REVERSE");

				if(m_auto_drive_last_reverse == m_options.reverse){
					console.log("reverse value is invalid", m_options.reverse, m_auto_drive_last_reverse);

					//clear waypoints
					let pif_dirpath = `${m_options.data_filepath}/waypoint_images`;
					if (fs.existsSync(pif_dirpath)) {
						const now = new Date();
						const formattedDate = now.getFullYear() +
							String(now.getMonth() + 1).padStart(2, '0') +
							String(now.getDate()).padStart(2, '0') + "_" +
							String(now.getHours()).padStart(2, '0') +
							String(now.getMinutes()).padStart(2, '0') + "_" +
							String(now.getSeconds()).padStart(2, '0');
						try {
							fs.renameSync(m_options.data_filepath, `${m_options.data_filepath}.${formattedDate}`);
							console.log(`folder moved to ${m_options.data_filepath}.${formattedDate}`);
						} catch (err) {
							console.error('folder move faild:', err);
						}
					}

					command_handler("STOP_AUTO");
					return;
				}

				m_auto_drive_ready = false;
				m_drive_mode = "AUTO";
				m_client.publish('pserver-auto-drive-info', JSON.stringify({
					"mode": "AUTO",
					"state": "START_AUTO",
				}));

				const pif_dirpath = `${m_options.data_filepath}/waypoint_images`;
				load_auto_drive_waypoints_ext(pif_dirpath, 0, null, (waypoints) => {
					waypoints = reindex_waypoints(waypoints, m_options.reverse);

					const msg = {
						src: waypoints
					};
					const keys = Object.keys(m_odometry_conf);
					function build_odometry_handler(cur) {
						const key = keys[cur];
						const next_cb = (converted_waypoints) => {
							m_odometry_conf[key].converted_waypoints = converted_waypoints;
							msg[key] = converted_waypoints;
							if (cur == keys.length - 1) {
								m_auto_drive_ready = true;
								update_auto_drive_waypoints(msg);
								update_auto_drive_cur(-1);
								console.log("drive mode", m_drive_mode);

								m_client.publish('pserver-auto-drive-info', JSON.stringify({
									"mode": "AUTO",
									"state": "READY_AUTO",
								}));

							} else {
								build_odometry_handler(cur + 1);
							}
						};
						if (!m_odometry_conf[key].enabled) {
							next_cb();
							return;
						}
						if (m_odometry_conf[key].handler) {
							setTimeout(() => {
								build_odometry_handler(cur);
							}, 1000);
							console.log(`wait ${key} handler finished`);
							return;
						}
						switch (key) {
							case ODOMETRY_TYPE.GPS:
								m_odometry_conf[key].handler = new GpsOdometry();
								break;
							case ODOMETRY_TYPE.ENCODER:
								m_odometry_conf[key].handler = new EncoderOdometry();
								break;
							case ODOMETRY_TYPE.VSLAM:
								m_odometry_conf[key].handler = new VslamOdometry({
									incremental_reconstruction: false,
									reverse: m_options.reverse,
									//host : m_argv.host,
									transforms_callback: (vslam_waypoints, active_points) => {
										msg["VSLAM"] = vslam_waypoints;
										msg["VSLAM_ACTIVE"] = active_points;
										update_auto_drive_waypoints(msg);
									},
								});
								break;
						}
						m_odometry_conf[key].handler.init(waypoints, next_cb, m_options.reverse);
					}
					build_odometry_handler(0);
				});
			} else {
				console.log("drive mode", m_drive_mode);
			}
			break;
		case "STOP_AUTO":
			stop_robot();

			if (m_drive_mode == "STANBY" || m_drive_mode == "RECORD") {
				console.log("skip STOP_AUTO : drive mode", m_drive_mode);
				return;
			}

			setTimeout(() => {
				stop_robot();
			}, 1000)
			m_drive_mode = "STANBY";

			const keys = Object.keys(m_odometry_conf);
			for (const key of keys) {
				if (m_odometry_conf[key].handler) {
					m_odometry_conf[key].handler.deinit();
					m_odometry_conf[key].handler = null;
				}
			}

			m_client.publish('pserver-auto-drive-info', JSON.stringify({
				"mode": "AUTO",
				"state": "STOP_AUTO",
			}));

			console.log("drive mode", m_drive_mode);
			break;
	}
}
function push_nmea(nmea) {
	return;
	if (!m_last_nmea) {
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
	process.on('SIGINT', () => {
		console.log('Ctrl+C detected! Gracefully exiting...');
		process.exit(0);
	});
}
