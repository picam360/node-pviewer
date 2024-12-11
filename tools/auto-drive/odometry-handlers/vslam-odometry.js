
const nmea = require('nmea-simple');
const utm = require('utm');
const fxp = require('fast-xml-parser');
const xml_parser = new fxp.XMLParser({
	ignoreAttributes: false,
	attributeNamePrefix: "",
});
const fs = require("fs");
const path = require("path");
const { execSync } = require('child_process');
const { spawn } = require('child_process');

const PifRosMessagePublisher = require('../pif-ros-message-publisher-humble');

// Convert degrees to radians
function degreesToRadians(degrees) {
    return degrees * (Math.PI / 180);
}

// Convert radians to degrees
function radiansToDegrees(radians) {
    return radians * (180 / Math.PI);
}

function quaternionToYaw(orientation) {
  const { x, y, z, w } = orientation;
  const yaw = Math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
  return yaw;
}

function launchDockerContainer() {
	const vslam_process = spawn('bash', ['launch_vslam.sh'], { cwd: path.resolve(__dirname, '../ros_packages') });
	vslam_process.stdout.on('data', (data) => {
	  console.log(`ISAAC_ROS_VSLAM STDOUT: ${data}`);
	});
	
	vslam_process.stderr.on('data', (data) => {
	  console.error(`ISAAC_ROS_VSLAM STDERR: ${data}`);
	});
	
	vslam_process.on('close', (code) => {
	  console.log(`ISAAC_ROS_VSLAM STDOUT CLOSED(: ${code})`);
	});
}
function killDockerContainer() {
    try {
        execSync('docker kill vslam-container');
    } catch (err) {
    }
}

class VslamOdometry {
    constructor() {
        this.initialized = false;
        this.waypoints = null;
        this.positions = null;
        this.current_nmea = null;
        this.current_imu = null;
        this.current_odom = null;
        this.publisher = null;
        this.push_cur = 0;
        this.m_ros_timestamp_base_ms = Date.now();
    }

    init(waypoints, callback){
        const keys = Object.keys(waypoints);

        this.waypoints = waypoints;
        this.waypoints_keys = keys;

        killDockerContainer();
		launchDockerContainer();

        setTimeout(() => {
            this.positions = {};

            this.publisher = new PifRosMessagePublisher();
            this.publisher.subscribeOdometry((odom) => {
                this.current_odom = odom;
                if(!this.initialized){
                    const push_cur = Math.round((odom.header.stamp.sec * 1e3 + odom.header.stamp.nanosec / 1e6) / (1e3 / 30));
                    if(push_cur < keys.length){
                        const cur = keys.length - 1 - push_cur;//reverse
                        this.positions[keys[cur]] = odom;
                    }else{
                        this.initialized = true;
                        clearInterval(push_timer);
                        if(callback){
                            callback();
                        }
                    }
                }
            });

            let heading = 0;
            this.publisher.initialize({ heading });

            const push_timer = setInterval(() => {
                const cur = keys.length - 1 - this.push_cur;//reverse
                const waypoint = waypoints[keys[Math.max(cur, 0)]];
                const jpeg_filepath = waypoint.jpeg_filepath;
                console.log(jpeg_filepath);
                const jpeg_data = fs.readFileSync(jpeg_filepath);
                this.push(null, waypoint.meta, jpeg_data);
            }, 200);//5fps
        }, 10000);
    }

    static cal_xy(waypoints){
        return null;
    }
  
    push(header, meta, jpeg_data) {
        const frame_dom = xml_parser.parse(meta);
        this.current_nmea = nmea.parseNmeaSentence(frame_dom['picam360:frame']['passthrough:nmea']);
        this.current_imu = JSON.parse(frame_dom['picam360:frame']['passthrough:imu']);
        this.current_jpeg_data = jpeg_data;

        const rosTimestamp = (() => {
            const { seconds, nanoseconds } = { seconds : Math.floor(0 / 1e3), nanoseconds : 0 };
            const additionalNanoseconds = Math.floor(this.push_cur * (1 / 30) * 1e9);
            const totalNanoseconds = nanoseconds + additionalNanoseconds;
            return {
                sec: seconds + Math.floor(totalNanoseconds / 1e9),
                nanosec: totalNanoseconds % 1e9,
            };
        })();

        this.publisher.publishImu(this.current_imu, rosTimestamp);
        this.publisher.publishVslam(this.current_jpeg_data, rosTimestamp);

        this.push_cur++;
    }

    calculateDistance(cur){
        const key = this.waypoints_keys[cur];
        if(this.positions[key] === undefined){
            return 0;
        }
        const target_position = this.positions[key];
        const dx = target_position.pose.position.x - this.current_odom.pose.position.x;
        const dy = target_position.pose.position.y - this.current_odom.pose.position.y;
        return Math.sqrt(dx * dx + dy * dy);
    }
    calculateBearing(cur){
        const key = this.waypoints_keys[cur];
        if(this.positions[key] === undefined){
            return 0;
        }
        const target_position = this.positions[key];
        const dx = target_position.pose.position.x - this.current_odom.pose.position.x;
        const dy = target_position.pose.position.y - this.current_odom.pose.position.y;
        const θ = Math.atan2(dy, dx);
        const bearing = (Math.PI / 2 - θ);
        return (radiansToDegrees(bearing) + 360) % 360; // Bearing in degrees
    }
    calculateHeadingError(cur){
        const heading = 90 - radiansToDegrees(quaternionToYaw(this.current_odom.pose.orientation));
        const targetHeading = this.calculateBearing(cur);
		let headingError = targetHeading - heading;
		if(headingError <= -180){
			headingError += 360;
		}else if(headingError > 180){
			headingError -= 360;
		}
        return headingError;
    }
  }
  
  module.exports = {
    VslamOdometry
  };