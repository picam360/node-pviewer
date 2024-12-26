
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

const encoder_odometry = require('./encoder-odometry');

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
    return;

    const vslam_path = '/home/picam360/github/picam360-vslam';
    const vslam_process = spawn('python', [`${vslam_path}/vslam.py`], { cwd: path.resolve(vslam_path) });
    vslam_process.stdout.on('data', (data) => {
        console.log(`PICAM360_VSLAM STDOUT: ${data}`);
    });

    vslam_process.stderr.on('data', (data) => {
        console.error(`PICAM360_VSLAM STDERR: ${data}`);
    });

    vslam_process.on('close', (code) => {
        console.log(`PICAM360_VSLAM STDOUT CLOSED(: ${code})`);
    });
}
function killDockerContainer() {
    return;

    const processName = "picam360-vslam"; // プロセス名を指定

    try {
        // プロセスIDを取得
        const output = execSync(`pgrep ${processName}`, { encoding: "utf-8" }); // 出力を文字列で取得
        const pids = output.split("\n").filter(pid => pid); // 改行で分割し、空行を削除

        if (pids.length === 0) {
            console.log(`No process found with name: ${processName}`);
        } else {
            // 各PIDを終了
            pids.forEach(pid => {
                try {
                    execSync(`kill ${pid}`);
                    console.log(`Process ${pid} (${processName}) killed successfully.`);
                } catch (killError) {
                    console.error(`Error killing process ${pid}: ${killError.message}`);
                }
            });
        }
    } catch (error) {
        console.error(`Error finding process: ${error.message}`);
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
        this.m_client = null;
    }

    init(waypoints, callback) {
        const keys = Object.keys(waypoints);

        this.waypoints = waypoints;
        this.waypoints_keys = keys;

        killDockerContainer();
        launchDockerContainer();

        const host = "localhost";
        const port = 6379;
        const redis = require('redis');
        const client = redis.createClient({
            url: `redis://${host}:${port}`
        });
        client.on('error', (err) => {
            console.error('redis error:', err);
            this.m_client = null;
        });
        client.connect().then(() => {
            console.log('redis connected:');
            this.m_client = client;
        });
        const subscriber = client.duplicate();
        subscriber.connect().then(() => {
            console.log('redis connected:');

            this.positions = {};
            subscriber.subscribe('picam360-vslam-odom', (data, key) => {
                console.log(data);
                const params = JSON.parse(data);
                if(!this.initialized){
                    if(params['type'] == 'backend'){
                        for(const odom of params['odom']){
                            const cur = keys.length - 1 - odom['timestamp'];//reverse
                            this.positions[keys[cur]] = odom;
                        }
                        this.initialized = true;
                        if(callback){
                            callback();
                        }
                    }
                }else{
                    this.current_odom = params['odom'][0];
                }
            });
            const enc_positions = encoder_odometry.EncoderOdometry.cal_xy(waypoints, encoder_odometry.EncoderOdometry.settings);

            let ref_cur = keys.length - 1;

            for (let i = 0; i < keys.length; i++) {
                const cur = keys.length - 1 - i;//reverse
                const pos = enc_positions[cur];
                let is_keyframe = false;
                if (cur == 0 || cur == keys.length - 1) {
                    is_keyframe = true;
                } else {
                    const threashold = 1;
                    const ref_pos = enc_positions[ref_cur];
                    const dx = pos.x - ref_pos.x;
                    const dy = pos.y - ref_pos.y;
                    const d = Math.sqrt(dx * dx + dy * dy);
                    if (d > threashold) {
                        is_keyframe = true;
                    }
                }

                if (!is_keyframe) {
                    continue;
                }

                this.push_cur = i;

                const waypoint = waypoints[keys[cur]];
                const jpeg_filepath = waypoint.jpeg_filepath;
                const jpeg_data = fs.readFileSync(jpeg_filepath);
                this.push(null, waypoint.meta, jpeg_data);

                console.log(`timestamp ${i} : ${jpeg_filepath}`);

                ref_cur = cur;
            }

            m_client.publish('picam360-vslam', JSON.stringify({
                "cmd": "backend",
                "itr": 7,
            }));
        });
    }

    static cal_xy(waypoints) {
        return null;
    }

    push(header, meta, jpeg_data) {
        const frame_dom = xml_parser.parse(meta);
        this.current_nmea = nmea.parseNmeaSentence(frame_dom['picam360:frame']['passthrough:nmea']);
        this.current_imu = JSON.parse(frame_dom['picam360:frame']['passthrough:imu']);
        this.current_jpeg_data = jpeg_data;

        m_client.publish('picam360-vslam', JSON.stringify({
            "cmd": "track",
            "timestamp": `${this.push_cur}`,
            "jpeg_data": jpeg_data.toString("base64"),
        }));

        this.push_cur++;
    }

    /**
     * Computes the transformation between two coordinate systems
     * @param {Array} points1 - Corresponding points in coordinate system 1 [[x1, y1], [x2, y2]]
     * @param {Array} points2 - Corresponding points in coordinate system 2 [[x1', y1'], [x2', y2']]
     * @param {Array} point - An arbitrary point in coordinate system 1 [x, y]
     * @returns {Array} - The corresponding point in coordinate system 2 [x', y']
     */
    transformCoordinates(points1, points2, point) {
        // Retrieve the corresponding points
        const [p1, p2] = points1;
        const [q1, q2] = points2;

        // Compute vectors
        const deltaP = [p2[0] - p1[0], p2[1] - p1[1]];
        const deltaQ = [q2[0] - q1[0], q2[1] - q1[1]];

        // Scale factor
        const scale = math.norm(deltaQ) / math.norm(deltaP);

        // Rotation angle
        const angleP = Math.atan2(deltaP[1], deltaP[0]);
        const angleQ = Math.atan2(deltaQ[1], deltaQ[0]);
        const rotation = angleQ - angleP;

        // Rotation matrix
        const rotationMatrix = [
            [Math.cos(rotation), -Math.sin(rotation)],
            [Math.sin(rotation), Math.cos(rotation)],
        ];

        // Translation vector
        const translation = [
            q1[0] - scale * (rotationMatrix[0][0] * p1[0] + rotationMatrix[0][1] * p1[1]),
            q1[1] - scale * (rotationMatrix[1][0] * p1[0] + rotationMatrix[1][1] * p1[1]),
        ];

        // Transform the given point
        const pointTransformed = [
            scale * (rotationMatrix[0][0] * point[0] + rotationMatrix[0][1] * point[1]) + translation[0],
            scale * (rotationMatrix[1][0] * point[0] + rotationMatrix[1][1] * point[1]) + translation[1],
        ];

        return pointTransformed;
    }

    getPosition() {
        if (this.current_odom) {
            return {
                x: this.current_odom.pose.position.x,
                y: this.current_odom.pose.position.y,
                heading: 90 - radiansToDegrees(quaternionToYaw(this.current_odom.pose.orientation)),
            };
        } else {
            return {
                x: 0,
                y: 0,
                heading: 0,
            };
        }
    }

    calculateDistance(cur) {
        let key;
        for (; cur < this.waypoints_keys.length; cur++) {
            key = this.waypoints_keys[cur];
            if (this.positions[key] !== undefined) {
                break;
            }
        }
        if (this.positions[key] === undefined) {
            return 0;
        }
        const target_position = this.positions[key];
        const dx = target_position.pose.position.x - this.current_odom.pose.position.x;
        const dy = target_position.pose.position.y - this.current_odom.pose.position.y;
        return Math.sqrt(dx * dx + dy * dy);
    }
    calculateBearing(cur) {
        let key;
        for (; cur < this.waypoints_keys.length; cur++) {
            key = this.waypoints_keys[cur];
            if (this.positions[key] !== undefined) {
                break;
            }
        }
        if (this.positions[key] === undefined) {
            return 0;
        }
        const target_position = this.positions[key];
        const dx = target_position.pose.position.x - this.current_odom.pose.position.x;
        const dy = target_position.pose.position.y - this.current_odom.pose.position.y;
        const θ = Math.atan2(dy, dx);
        const bearing = (Math.PI / 2 - θ);
        return (radiansToDegrees(bearing) + 360) % 360; // Bearing in degrees
    }
    calculateHeadingError(cur) {
        let key;
        for (; cur < this.waypoints_keys.length; cur++) {
            key = this.waypoints_keys[cur];
            if (this.positions[key] !== undefined) {
                break;
            }
        }
        if (this.positions[key] === undefined) {
            return 0;
        }
        const heading = 90 - radiansToDegrees(quaternionToYaw(this.current_odom.pose.orientation));
        const targetHeading = this.calculateBearing(cur);
        let headingError = targetHeading - heading;
        if (headingError <= -180) {
            headingError += 360;
        } else if (headingError > 180) {
            headingError -= 360;
        }
        return headingError;
    }
}

module.exports = {
    VslamOdometry
};