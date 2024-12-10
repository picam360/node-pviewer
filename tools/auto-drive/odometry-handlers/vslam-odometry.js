
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
            let positions_cur = 0;
            this.positions = {};

            this.publisher = new PifRosMessagePublisher();
            this.publisher.subscribeOdometry((odom) => {
                this.current_odom = odom;
                if(!this.initialized){
                    this.positions[keys[positions_cur]] = odom;
                    positions_cur++;
                    if(positions_cur >= keys.length){
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
                const waypoint = waypoints[keys[Math.min(this.push_cur, keys.length - 1)]];
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
            const { seconds, nanoseconds } = { seconds : Math.floor(this.m_ros_timestamp_base_ms / 1e3), nanoseconds : 0 };
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
        const target_position = this.positions[key];
        const dx = this.current_odom.pose.position.x - target_position.pose.position.x;
        const dy = this.current_odom.pose.position.y - target_position.pose.position.y;
        return Math.sqrt(dx * dx + dy * dy);
    }
    calculateBearing(cur){
        const key = this.waypoints_keys[cur];
        const target_position = this.positions[key];
        const dx = this.current_odom.pose.position.x - target_position.pose.position.x;
        const dy = this.current_odom.pose.position.y - target_position.pose.position.y;
        const θ = Math.atan2(dy, dx);
        const bearing = (Math.PI / 2 - θ);
        return (radiansToDegrees(bearing) + 360) % 360; // Bearing in degrees
    }
    calculateHeadingError(cur){
        const targetHeading = this.calculateBearing(cur);
		let headingError = targetHeading - this.current_imu.heading;
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