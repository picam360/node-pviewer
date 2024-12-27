
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

const { EncoderOdometry } = require('./encoder-odometry');

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
function quaternionToYXZ(orientation) {
    const { x, y, z, w } = orientation;

    // Yaw (Y軸周りの回転)
    const yaw = Math.atan2(2.0 * (w * y - z * x), 1.0 - 2.0 * (x * x + y * y));

    // Pitch (X軸周りの回転)
    const pitch = Math.asin(2.0 * (w * x + y * z));

    // Roll (Z軸周りの回転)
    const roll = Math.atan2(2.0 * (w * z - x * y), 1.0 - 2.0 * (x * x + z * z));

    return { yaw, pitch, roll };
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
        this.push_nodes = null;
        this.last_pushVslam_cur = 0;
        this.last_odom_cur = 0;
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
                for(const odom of params['odom']){
                    const cur = keys.length - 1 - odom['timestamp'];//reverse
                    if(keys[cur] !== undefined){
                        this.positions[keys[cur]] = odom;
                    }
                }
                if(!this.initialized){
                    if(params['type'] == 'backend'){
                        this.initialized = true;
                        this.current_odom = null;
                        this.push_cur = keys.length;
                        this.last_pushVslam_cur = keys.length;
                        this.last_odom_cur = keys.length;
                        if(callback){
                            callback();
                        }
                    }
                }else{
                    const odom = params['odom'][params['odom'].length - 1];//last one
                    const odom_cur = odom['timestamp'];
                    if(odom_cur > this.last_odom_cur){
                        this.current_odom = odom;
                        for(let i=this.last_odom_cur;i<odom_cur;i++){
                            const key = this.numToKey(i);
                            delete this.push_nodes[key];
                        }
                        this.last_odom_cur = odom_cur;
                    }
                }
            });
            this.enc_positions = EncoderOdometry.cal_xy(waypoints, EncoderOdometry.settings);

            let ref_cur = keys.length - 1;

            for (let i = 0; i < keys.length; i++) {
                const cur = keys.length - 1 - i;//reverse
                const pos = this.enc_positions[cur];
                let is_keyframe = false;
                if (cur == 0 || cur == keys.length - 1) {
                    is_keyframe = true;
                } else {
                    const threashold = 1;
                    const ref_pos = this.enc_positions[ref_cur];
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

                const waypoint = waypoints[keys[cur]];
                const jpeg_filepath = waypoint.jpeg_filepath;
                const jpeg_data = fs.readFileSync(jpeg_filepath);
                this.pushVslam(`${i}`, jpeg_data);

                console.log(`timestamp ${i} : ${jpeg_filepath}`);

                ref_cur = cur;
            }

            m_client.publish('picam360-vslam', JSON.stringify({
                "cmd": "backend",
                "itr": 7,
            }));
        });
    }

    is_ready() {
        return (this.current_odom != null);
    }

    static cal_xy(waypoints) {
        return null;
    }

    pushVslam(timestamp, jpeg_data) {
        m_client.publish('picam360-vslam', JSON.stringify({
            "cmd": "track",
            "timestamp": `${timestamp}`,
            "jpeg_data": jpeg_data.toString("base64"),
        }));
    }

    numToKey(num) {
        return num.toString().padStart(10, '0');
    }

    push(header, meta, jpeg_data) {
        const frame_dom = xml_parser.parse(meta);
        this.current_nmea = nmea.parseNmeaSentence(frame_dom['picam360:frame']['passthrough:nmea']);
        this.current_imu = JSON.parse(frame_dom['picam360:frame']['passthrough:imu']);
        this.current_encoder = JSON.parse(frame_dom['picam360:frame']['passthrough:encoder']);

        if(this.push_nodes == null){
            this.pushVslam(`${this.push_cur}`, jpeg_data);
            this.last_pushVslam_cur = this.push_cur;
            this.push_nodes = {};
        }

        this.push_nodes[this.numToKey(this.push_cur)] = {
            nmea : this.current_nmea,
            imu : this.current_imu,
            encoder : this.current_encoder,
        };

        {
            const threashold = 1;
            const nodes = [];
            for(let i=this.last_pushVslam_cur;i<this.push_cur+1;i++){
                const key = this.numToKey(i);
                nodes.push(this.push_nodes[key]);
            }
    
            const pos = this.appendEncoderPosition({
                x:0, y:0, heading:0
            }, nodes);

            const d = Math.sqrt(pos.x*pos.x + pos.y*pos.y);
            if(d > threashold){
                this.pushVslam(`${this.push_cur}`, jpeg_data);
                this.last_pushVslam_cur = this.push_cur;
            }
        }

        this.push_cur++;
    }
    
    static transformCoordinates(points1, points2, point) {
        // Retrieve the corresponding points
        const [p1, p2] = points1; // p1, p2 are objects with {x, y}
        const [q1, q2] = points2; // q1, q2 are objects with {x, y}
    
        // Compute vectors
        const deltaP = { x: p2.x - p1.x, y: p2.y - p1.y };
        const deltaQ = { x: q2.x - q1.x, y: q2.y - q1.y };
    
        // Scale factor
        const normDeltaP = Math.sqrt(deltaP.x ** 2 + deltaP.y ** 2);
        const normDeltaQ = Math.sqrt(deltaQ.x ** 2 + deltaQ.y ** 2);
        const scale = normDeltaQ / normDeltaP;
    
        // Rotation angle
        const angleP = Math.atan2(deltaP.y, deltaP.x);
        const angleQ = Math.atan2(deltaQ.y, deltaQ.x);
        const rotation = angleQ - angleP;
    
        // Rotation matrix
        const rotationMatrix = [
            [Math.cos(rotation), -Math.sin(rotation)],
            [Math.sin(rotation), Math.cos(rotation)],
        ];
    
        // Translation vector
        const translation = {
            x: q1.x - scale * (rotationMatrix[0][0] * p1.x + rotationMatrix[0][1] * p1.y),
            y: q1.y - scale * (rotationMatrix[1][0] * p1.x + rotationMatrix[1][1] * p1.y),
        };

        const dheading_p = p2.heading - p1.heading;
        const dheading_q = q2.heading - q1.heading;
    
        // Transform the given point
        const pointTransformed = {
            x: scale * (rotationMatrix[0][0] * point.x + rotationMatrix[0][1] * point.y) + translation.x,
            y: scale * (rotationMatrix[1][0] * point.x + rotationMatrix[1][1] * point.y) + translation.y,
            heading: point.heading + (dheading_p + dheading_q) / 2,
        };
    
        return pointTransformed;
    }

    getKeyframePosition(cur) {
        let cur1 = -1;
        for (let i=cur; i >= 0; i--) {
            const key = this.waypoints_keys[i];
            if (this.positions[key] !== undefined) {
                cur1 = i;
                break;
            }
        }
        if(cur1 < 0){
            for (let i=cur; i < this.waypoints_keys.length; i++) {
                const key = this.waypoints_keys[i];
                if (this.positions[key] !== undefined) {
                    cur1 = i;
                    break;
                }
            }
        }
        let cur2 = -1;
        for (let i=cur1 + 1; i < this.waypoints_keys.length; i++) {
            const key = this.waypoints_keys[i];
            if (this.positions[key] !== undefined) {
                cur2 = i;
                break;
            }
        }
        if(cur2 < 0){
            cur2 = cur1;
            cur1 = -1;
            for (let i=cur2 - 1; i >= 0; i--) {
                const key = this.waypoints_keys[i];
                if (this.positions[key] !== undefined) {
                    cur1 = i;
                    break;
                }
            }
        }
        if(cur1 < 0 || cur2 < 0 || this.current_odom == null){
            return {
                x: 0,
                y: 0,
                heading: 0,
            };
        }

        function convert_odom_to_pos(odom){
            return {
                x : odom.pose.position.x,
                y : odom.pose.position.z,
                heading : quaternionToYXZ(odom.pose.orientation).yaw * 180 / Math.PI,
            };
        }
        const vslam_pos1 = convert_odom_to_pos(this.positions[cur1]);
        const vslam_pos2 = convert_odom_to_pos(this.positions[cur2]);
        const vslam_pos = convert_odom_to_pos(this.current_odom);
        const enc_pos = VslamOdometry.transformCoordinates(
            [vslam_pos1, this.enc_positions[cur1]],
            [vslam_pos2, this.enc_positions[cur2]],
            vslam_pos);

        return enc_pos;
    }

    appendEncoderPosition(pos, nodes) {
            
        const encoder_params = {
            right_gain : EncoderOdometry.settings.right_gain,
            meter_per_pulse : EncoderOdometry.settings.meter_per_pulse,
            wheel_separation : EncoderOdometry.settings.wheel_separation,
            last_left_counts : nodes[0].encoder.left * EncoderOdometry.left_direction,
            last_right_counts : nodes[0].encoder.right * EncoderOdometry.right_direction,
            x : pos.x,
            y : pos.y,
            heading : pos.heading,
        };
        for (let i=1; i < nodes.length; i++) {
            EncoderOdometry.inclement_xy(
                encoder_params,
                nodes[i].encoder.left * EncoderOdometry.left_direction,
                nodes[i].encoder.right * EncoderOdometry.right_direction);
        }
        return {
            x : encoder_params.x,
            y : encoder_params.y,
            heading : encoder_params.heading,
        };
    }

    getPosition(cur) {
        if(this.push_nodes == null){
            return {
                x: 0,
                y: 0,
                heading: 0,
            };
        }

        const kf_pos = this.getKeyframePosition(cur);

        const nodes = [];
        for(let i=this.last_odom_cur;i<this.push_cur;i++){
            const key = this.numToKey(i);
            nodes.push(this.push_nodes[key]);
        }

        return this.appendEncoderPosition(kf_pos, nodes);
    }
    calculateDistance(cur) {
        const pos = this.getPosition(cur);
        const dx = this.enc_positions[cur].x - pos.x;
        const dy = this.enc_positions[cur].y - pos.y;
        const distance = Math.sqrt(dx * dx + dy * dy);
        console.log("distance", distance);
        return distance;
    }
    calculateBearing(cur, pos) {
        if(!pos){
            pos = this.getPosition(cur);
        }
        const dx = this.enc_positions[cur].x - pos.x;
        const dy = this.enc_positions[cur].y - pos.y;
        const θ = Math.atan2(dy, dx);
        const bearing = (Math.PI / 2 - θ);
        return (radiansToDegrees(bearing) + 360) % 360; // Bearing in degrees
    }
    calculateHeadingError(cur) {
        const pos = this.getPosition(cur);
        const targetHeading = this.calculateBearing(cur, pos);
        let headingError = targetHeading - pos.heading;
        if (headingError <= -180) {
            headingError += 360;
        } else if (headingError > 180) {
            headingError -= 360;
        }
        console.log("headingError", headingError);
        return headingError;
    }
}

module.exports = {
    VslamOdometry
};