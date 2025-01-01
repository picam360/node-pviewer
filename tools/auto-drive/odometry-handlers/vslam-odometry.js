
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

function getYawFromRotationMatrix(matrix) {
    // 行列要素を取得
    const r31 = matrix[2][0];

    // Y軸回転（ラジアン）を計算
    const thetaY = Math.asin(-r31); // arcsin(-r31)

    return thetaY;
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
    constructor(options) {
        this.options = options || {};
        this.host = this.options.host || "localhost";

        this.initialized = false;
        this.waypoints = null;
        this.vslam_waypoints = null;
        this.active_points = null;
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

        const host = this.host;
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

            function convert_transforms_to_positions(nodes, enc_positions){
                function convert_transform_to_pos(node){
                    const heading = getYawFromRotationMatrix(node.transform) * 180 / Math.PI;
                    return {
                        x : node.transform[0][3],
                        y : node.transform[2][3],
                        heading : (heading + 180) % 360,//back camera
                    };
                }
                const keys = Object.keys(enc_positions);
                const vslam_waypoints = {};
                const active_points = {};
                for(const node of nodes){
                    const cur = keys.length - 1 - node['timestamp'];//reverse
                    if(keys[cur] !== undefined){
                        vslam_waypoints[keys[cur]] = convert_transform_to_pos(node);
                    }else{
                        active_points[node['timestamp']] = convert_transform_to_pos(node);
                    }
                }
                const keys2 = Object.keys(vslam_waypoints);
                let first_key = keys2[0];
                let last_key = keys2[keys2.length - 1];

                const first_vslam_position = Object.assign({}, vslam_waypoints[first_key]);
                const last_vslam_position = vslam_waypoints[last_key];
                const diff_vslam_position = {
                    x : last_vslam_position.x - first_vslam_position.x,
                    y : last_vslam_position.y - first_vslam_position.y,
                };
                const vslam_heading = Math.atan2(diff_vslam_position.x, diff_vslam_position.y) * 180 / Math.PI;//from y axis
                const vslam_scale = Math.sqrt(diff_vslam_position.x ** 2 + diff_vslam_position.y ** 2);
                
                const first_enc_position = enc_positions[first_key];
                const last_enc_position = enc_positions[last_key];
                const diff_enc_position = {
                    x : last_enc_position.x - first_enc_position.x,
                    y : last_enc_position.y - first_enc_position.y,
                };
                const enc_heading = Math.atan2(diff_enc_position.x, diff_enc_position.y) * 180 / Math.PI;//from y axis
                const enc_scale = Math.sqrt(diff_enc_position.x ** 2 + diff_enc_position.y ** 2);

                const heading_diff = (enc_heading - vslam_heading) % 360;
                // const first_heading_diff = first_enc_position.heading - first_vslam_position.heading;
                // const last_heading_diff = last_enc_position.heading - last_vslam_position.heading;
                // const heading_diff = (first_heading_diff + last_heading_diff) / 2;
                
                const rad = (Math.PI / 180) * -heading_diff;
                const cos = Math.cos(rad);
                const sin = Math.sin(rad);
                const scale = enc_scale / vslam_scale;
                for(const key of Object.keys(vslam_waypoints)){
                    const x = vslam_waypoints[key].x - first_vslam_position.x;
                    const y = vslam_waypoints[key].y - first_vslam_position.y;
                    vslam_waypoints[key].x = scale * (x * cos - y * sin);
                    vslam_waypoints[key].y = scale * (x * sin + y * cos);
                    vslam_waypoints[key].heading += heading_diff;

                }
                for(const key of Object.keys(active_points)){
                    const x = active_points[key].x - first_vslam_position.x;
                    const y = active_points[key].y - first_vslam_position.y;
                    active_points[key].x = scale * (x * cos - y * sin);
                    active_points[key].y = scale * (x * sin + y * cos);
                    active_points[key].heading += heading_diff;
                }
                return {
                    vslam_waypoints,
                    active_points,
                };
            }

            function convert_odom_to_positions(odoms, enc_positions){
                function convert_odom_to_pos(odom){
                    return {
                        x : -odom.pose.position.x,
                        y : odom.pose.position.z,
                        heading : -quaternionToYXZ(odom.pose.orientation).yaw * 180 / Math.PI,
                    };
                }
                const keys = Object.keys(enc_positions);
                const vslam_waypoints = {};
                const active_points = {};
                for(const odom of odoms){
                    const cur = keys.length - 1 - odom['timestamp'];//reverse
                    if(keys[cur] !== undefined){
                        vslam_waypoints[keys[cur]] = convert_odom_to_pos(odom);
                    }else{
                        active_points[odom['timestamp']] = convert_odom_to_pos(odom);
                    }
                }
                const keys2 = Object.keys(vslam_waypoints);
                let first_key = keys2[0];
                let last_key = keys2[keys2.length - 1];

                const first_vslam_position = Object.assign({}, vslam_waypoints[first_key]);
                const last_vslam_position = vslam_waypoints[last_key];
                const diff_vslam_position = {
                    x : last_vslam_position.x - first_vslam_position.x,
                    y : last_vslam_position.y - first_vslam_position.y,
                };
                const vslam_heading = Math.atan2(diff_vslam_position.x, diff_vslam_position.y) * 180 / Math.PI;//from y axis
                const vslam_scale = Math.sqrt(diff_vslam_position.x ** 2 + diff_vslam_position.y ** 2);

                const first_enc_position = enc_positions[first_key];
                const last_enc_position = enc_positions[last_key];
                const diff_enc_position = {
                    x : last_enc_position.x - first_enc_position.x,
                    y : last_enc_position.y - first_enc_position.y,
                };
                const enc_heading = Math.atan2(diff_enc_position.x, diff_enc_position.y) * 180 / Math.PI;//from y axis
                const enc_scale = Math.sqrt(diff_enc_position.x ** 2 + diff_enc_position.y ** 2);

                const heading_diff = enc_heading - vslam_heading;
                
                const rad = (Math.PI / 180) * heading_diff;
                const cos = Math.cos(rad);
                const sin = Math.sin(rad);
                const scale = enc_scale / vslam_scale;
                for(const key of Object.keys(vslam_waypoints)){
                    const x = vslam_waypoints[key].x - first_vslam_position.x;
                    const y = vslam_waypoints[key].y - first_vslam_position.y;
                    //vslam_waypoints[key].x = -scale * (x * cos - y * sin);
                    //vslam_waypoints[key].y = -scale * (x * sin + y * cos);
                    vslam_waypoints[key].x = -scale * x;
                    vslam_waypoints[key].y = -scale * y;
                    vslam_waypoints[key].heading += heading_diff;
                }
                for(const key of Object.keys(active_points)){
                    const x = active_points[key].x - first_vslam_position.x;
                    const y = active_points[key].y - first_vslam_position.y;
                    active_points[key].x = -scale * (x * cos - y * sin);
                    active_points[key].y = -scale * (x * sin + y * cos);
                    active_points[key].heading += heading_diff;
                }
                return {
                    vslam_waypoints,
                    active_points,
                };
            }

            subscriber.subscribe('picam360-vslam-odom', (data, key) => {
                console.log(data);
                const params = JSON.parse(data);

                if(!this.initialized){
                    if(params['type'] == 'backend'){
                        this.initialized = true;
                        this.current_odom = null;
                        this.push_cur = keys.length;
                        this.last_pushVslam_cur = keys.length;
                        this.last_odom_cur = keys.length;

                        const { vslam_waypoints, active_points } = convert_transforms_to_positions(params['transforms'], this.enc_positions);
                        this.vslam_waypoints = vslam_waypoints;
                        this.active_points = active_points;
                        
                        if(callback){
                            callback(this.vslam_waypoints);
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

                        const { vslam_waypoints, active_points } = convert_transforms_to_positions(params['transforms'], this.enc_positions);
                        this.vslam_waypoints = vslam_waypoints;
                        this.active_points = active_points;

                        if(this.options.transforms_callback){
                            this.options.transforms_callback(this.vslam_waypoints, this.active_points);
                        }
                    }
                }
            });
            {
                const settings = Object.assign({}, EncoderOdometry.settings);
                settings.x_initial = 0;
                settings.y_initial = 0;
                settings.heading_initial = EncoderOdometry.cal_heading(this.waypoints);
                this.enc_positions = EncoderOdometry.cal_xy(this.waypoints, settings);
            }

            let ref_cur = keys.length - 1;

            for (let i = 0; i < keys.length; i++) {
                const cur = keys.length - 1 - i;//reverse
                const pos = this.enc_positions[cur];
                let is_keyframe = false;
                if (cur == 0 || cur == keys.length - 1) {
                    is_keyframe = true;
                } else {
                    const dr_threashold = 1;
                    const dh_threashold = 5;
                    const ref_pos = this.enc_positions[ref_cur];
                    const dx = pos.x - ref_pos.x;
                    const dy = pos.y - ref_pos.y;
                    const dh = pos.heading - ref_pos.heading;
                    const dr = Math.sqrt(dx * dx + dy * dy);
                    if (dr > dr_threashold || Math.abs(dh) > dh_threashold) {
                        console.log(`dr=${dr}, dh=${dh}`);
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

        const dheading_1 = q1.heading - p1.heading;
        const dheading_2 = q2.heading - p2.heading;

        const p_vec = {
            x : point.x - p1.x,
            y : point.y - p1.y,
            heading : point.heading,
        };
    
        // Transform the given point
        const pointTransformed = {
            x: scale * (rotationMatrix[0][0] * p_vec.x + rotationMatrix[0][1] * p_vec.y) + q1.x,
            y: scale * (rotationMatrix[1][0] * p_vec.x + rotationMatrix[1][1] * p_vec.y) + q1.y,
            heading: p_vec.heading + (dheading_1 + dheading_2) / 2,
        };
    
        return pointTransformed;
    }

    getKeyframePosition(cur) {
        let cur1 = -1;
        for (let i=cur; i >= 0; i--) {
            const key = this.waypoints_keys[i];
            if (this.vslam_waypoints[key] !== undefined) {
                cur1 = i;
                break;
            }
        }
        if(cur1 < 0){
            for (let i=cur; i < this.waypoints_keys.length; i++) {
                const key = this.waypoints_keys[i];
                if (this.vslam_waypoints[key] !== undefined) {
                    cur1 = i;
                    break;
                }
            }
        }
        let cur2 = -1;
        for (let i=cur1 + 1; i < this.waypoints_keys.length; i++) {
            const key = this.waypoints_keys[i];
            if (this.vslam_waypoints[key] !== undefined) {
                cur2 = i;
                break;
            }
        }
        if(cur2 < 0){
            cur2 = cur1;
            cur1 = -1;
            for (let i=cur2 - 1; i >= 0; i--) {
                const key = this.waypoints_keys[i];
                if (this.vslam_waypoints[key] !== undefined) {
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

        return this.active_points[this.current_odom['timestamp']];
        
        const enc_pos = VslamOdometry.transformCoordinates(
            [this.vslam_waypoints[cur1], this.vslam_waypoints[cur2]],
            [this.enc_positions[cur1], this.enc_positions[cur2]],
            this.active_points[this.current_odom['timestamp']]);

        return enc_pos;
    }

    appendEncoderPosition(pos, nodes) {
            
        const encoder_params = {
            right_gain : EncoderOdometry.settings.right_gain,
            meter_per_pulse : EncoderOdometry.settings.meter_per_pulse,
            wheel_separation : EncoderOdometry.settings.wheel_separation,
            last_left_counts : nodes[0].encoder.left * EncoderOdometry.settings.left_direction,
            last_right_counts : nodes[0].encoder.right * EncoderOdometry.settings.right_direction,
            x : pos.x,
            y : pos.y,
            heading : pos.heading,
        };
        for (let i=1; i < nodes.length; i++) {
            EncoderOdometry.inclement_xy(
                encoder_params,
                nodes[i].encoder.left * EncoderOdometry.settings.left_direction,
                nodes[i].encoder.right * EncoderOdometry.settings.right_direction);
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

        return kf_pos;

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