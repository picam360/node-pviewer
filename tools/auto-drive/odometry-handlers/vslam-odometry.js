
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
const numeric = require('numeric');

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

function rotate_vec(vec, heading){
    const rad = (Math.PI / 180) * -heading;
    const cos = Math.cos(rad);
    const sin = Math.sin(rad);
    return {
        x : vec.x * cos - vec.y * sin,
        y : vec.x * sin + vec.y * cos,
        heading : (vec.heading || 0) + heading,
    }
}
function rotate_positions(positions, heading){
    for(const key of Object.keys(positions)){
        Object.assign(positions[key], rotate_vec(positions[key], heading));
    }
}
function scale_positions(positions, scale){
    for(const key of Object.keys(positions)){
        positions[key].x *= scale;
        positions[key].y *= scale;
    }
}
function aply_cam_offset(positions, cam_offset){
    for(const key of Object.keys(positions)){
        const offset = rotate_vec(cam_offset, positions[key].heading);
        positions[key].x += offset.x;
        positions[key].y += offset.y;
    }
}
function minus_offset_from_positions(positions, offset){
    for(const key of Object.keys(positions)){
        positions[key].x -= offset.x;
        positions[key].y -= offset.y;
    }
}
function add_offset_from_positions(positions, offset){
    for(const key of Object.keys(positions)){
        positions[key].x += offset.x;
        positions[key].y += offset.y;
    }
}
function createErrorFunction(vslam_waypoints, enc_waypoints, settings, types) {
    return (solving_params) => {
        for(const i in types){
            settings[types[i]] = solving_params[i];
        }
        const positions = JSON.parse(JSON.stringify(vslam_waypoints));

        const keys = Object.keys(positions);

        scale_positions(positions, settings.scale);
        aply_cam_offset(positions, {
            x : settings.cam_offset_x,
            y : settings.cam_offset_y,
        });
        rotate_positions(positions, settings.heading_diff);
        const base_position = Object.assign({}, positions[keys[0]]);
        minus_offset_from_positions(positions, base_position);

        let totalError = 0;
        for(const key of keys){
            const error_X = (positions[key].x - enc_waypoints[key].x) ** 2;
            const error_Y = (positions[key].y - enc_waypoints[key].y) ** 2;
            totalError += error_X + error_Y;
        }
        
        return totalError;
    }
}

function cal_fitting_params(vslam_waypoints, enc_waypoints, settings){
    const keys2 = Object.keys(vslam_waypoints);
    let first_key = keys2[0];
    let last_key = keys2[keys2.length - 1];

    const first_vslam_position = vslam_waypoints[first_key];
    const last_vslam_position = vslam_waypoints[last_key];
    const vslam_scale = Math.sqrt(
        (last_vslam_position.x - first_vslam_position.x) ** 2 + (last_vslam_position.y - first_vslam_position.y) ** 2);
    
    const first_enc_position = enc_waypoints[first_key];
    const last_enc_position = enc_waypoints[last_key];
    const enc_scale = Math.sqrt(
        (last_enc_position.x - first_enc_position.x) ** 2 + (last_enc_position.y - first_enc_position.y) ** 2);
    
    const scale = enc_scale / vslam_scale;

    const vslam_heading = Math.atan2(
        last_vslam_position.x - first_vslam_position.x,
        last_vslam_position.y - first_vslam_position.y) * 180 / Math.PI;//from y axis
    const enc_heading = Math.atan2(
        last_enc_position.x - first_enc_position.x,
        last_enc_position.y - first_enc_position.y) * 180 / Math.PI;//from y axis

    const heading_diff = (enc_heading - vslam_heading) % 360;
    // const first_heading_diff = first_enc_position.heading - first_vslam_position.heading;
    // const last_heading_diff = last_enc_position.heading - last_vslam_position.heading;
    // const heading_diff = (first_heading_diff + last_heading_diff) / 2;

    settings = settings || {};
    settings.scale = scale;
    settings.heading_diff = heading_diff;
    settings.cam_offset_x = settings.cam_offset_x || 0.0;
    settings.cam_offset_y = settings.cam_offset_y || 0.0;
    
    let result = numeric.uncmin(
        createErrorFunction(vslam_waypoints, enc_waypoints, settings, ["scale", "heading_diff"]),
        [settings.scale, settings.heading_diff]);
    settings.scale = result.solution[0];
    settings.heading_diff = result.solution[1] % 360;

    //createErrorFunction(vslam_waypoints, enc_waypoints, settings, ["scale", "heading_diff"])([settings.scale, settings.heading_diff]);

    return settings;
}

function convert_transforms_to_positions(nodes, enc_waypoints){
    function convert_transform_to_pos(node){
        const heading = getYawFromRotationMatrix(node.transform) * 180 / Math.PI;
        return {
            x : node.transform[0][3],
            y : node.transform[2][3],
            heading : (heading + 180) % 360,//back camera
        };
    }
    const keys = Object.keys(enc_waypoints);
    const vslam_waypoints = {};
    const active_points = {};
    for(const node of nodes){
        //const cur = keys.length - 1 - node['timestamp'];//reverse
        const cur = node['timestamp'];
        if(keys[cur] !== undefined){
            vslam_waypoints[keys[cur]] = convert_transform_to_pos(node);
        }else{
            active_points[node['timestamp']] = convert_transform_to_pos(node);
        }
    }

    const settings = {
        scale : 1.0,
        heading_diff : 0.0,
        cam_offset_x : VslamOdometry.settings.cam_offset.x,
        cam_offset_y : VslamOdometry.settings.cam_offset.y,
    };
    cal_fitting_params(vslam_waypoints, enc_waypoints, settings);

    if(VslamOdometry.settings.calib_enabled){
        let result = numeric.uncmin(
            createErrorFunction(vslam_waypoints, enc_waypoints, settings, ["scale", "heading_diff", "cam_offset_y"]),
            [settings.scale, settings.heading_diff, settings.cam_offset_y]);
            settings.scale = result.solution[0];
            settings.heading_diff = result.solution[1] % 360;
            settings.cam_offset_y = result.solution[2];
    }

    const keys2 = Object.keys(vslam_waypoints);
    let first_key = keys2[0];

    scale_positions(vslam_waypoints, settings.scale);
    aply_cam_offset(vslam_waypoints, {
        x : settings.cam_offset_x,
        y : settings.cam_offset_y,
    });
    rotate_positions(vslam_waypoints, settings.heading_diff);
    const base_position = Object.assign({}, vslam_waypoints[first_key]);
    minus_offset_from_positions(vslam_waypoints, base_position);

    scale_positions(active_points, settings.scale);
    aply_cam_offset(active_points, {
        x : settings.cam_offset_x,
        y : settings.cam_offset_y,
    });
    rotate_positions(active_points, settings.heading_diff);
    minus_offset_from_positions(active_points, base_position);

    return {
        vslam_waypoints,
        active_points,
    };
}

class VslamOdometry {
    static settings = {
        cam_offset : {
            x : 0.0,
            y : 2.2,
            //y : 2.228590172834311,
        },
        dr_threashold : 1,
        dh_threashold : 5,

        calib_enabled : false,
    };
    constructor(options) {
        this.options = options || {};
        this.host = this.options.host || "localhost";

        this.initialized = false;
        this.waypoints = null;
        this.vslam_waypoints = null;
        this.active_points = null;
        this.push_cur = 0;
        this.backend_pending = 0;
        this.m_client = null;
        this.first_push = true;
        this.last_pushVslam_cur = 0;
        this.last_odom_cur = 0;

        this.enc_odom = new EncoderOdometry();
        this.enc_waypoints = {};
        this.enc_positions = {};
    }

    init(waypoints, callback) {
        this.enc_odom.init(waypoints, (enc_waypoints) => {
            this.enc_waypoints = enc_waypoints;

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

                subscriber.subscribe('picam360-vslam-odom', (data, key) => {
                    console.log(data);
                    const params = JSON.parse(data);

                    if(!this.initialized){
                        if(params['type'] == 'backend'){
                            this.initialized = true;
                            this.current_odom = null;
                            this.push_cur = keys.length;
                            this.last_pushVslam_cur = keys.length - 1;
                            this.last_odom_cur = keys.length - 1;

                            const { vslam_waypoints, active_points } = convert_transforms_to_positions(params['transforms'], this.enc_waypoints);
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
                            this.last_odom_cur = odom_cur;

                            const { vslam_waypoints, active_points } = convert_transforms_to_positions(params['transforms'], this.enc_waypoints);
                            this.vslam_waypoints = vslam_waypoints;
                            this.active_points = active_points;

                            const update_gain = 0.1;
                            const update_r_cutoff = 0.5;
                            const update_h_cutoff = 1.0;

                            const kf_pos = this.getKeyframePosition();
                            const enc_pos = this.enc_positions[odom_cur];
                            const diff_x = kf_pos.x - enc_pos.x;
                            const diff_y = kf_pos.y - enc_pos.y;
                            const diff_r = Math.sqrt(diff_x ** 2 + diff_y ** 2);
                            const diff_h = kf_pos.heading - enc_pos.heading;
                            const diff_h_abs = Math.abs(diff_h);
                            const update_gain_r = Math.min(diff_r, update_r_cutoff) / Math.max(diff_r, 1e-3) * update_gain;
                            const update_gain_h = Math.min(diff_h_abs, update_h_cutoff) / Math.max(diff_h_abs, 1e-3) * update_gain;
                            this.enc_odom.encoder_params.x += diff_x * update_gain_r;
                            this.enc_odom.encoder_params.y += diff_y * update_gain_r;
                            this.enc_odom.encoder_params.heading += diff_h * update_gain_h;

                            console.log("diff_x", diff_x);
                            console.log("diff_y", diff_y);
                            console.log("diff_heading", diff_heading);

                            if(this.options.transforms_callback){
                                this.options.transforms_callback(this.vslam_waypoints, this.active_points);
                            }
                        }
                    }
                });

                //let ref_cur = keys.length - 1;//reverse
                let ref_cur = 0;

                for (let i = 0; i < keys.length; i++) {
                    //const cur = keys.length - 1 - i;//reverse
                    const cur = i;
                    const pos = this.enc_waypoints[cur];
                    let is_keyframe = false;
                    if (cur == 0 || cur == keys.length - 1) {
                        is_keyframe = true;
                    } else {
                        const ref_pos = this.enc_waypoints[ref_cur];
                        const dx = pos.x - ref_pos.x;
                        const dy = pos.y - ref_pos.y;
                        const dh = pos.heading - ref_pos.heading;
                        const dr = Math.sqrt(dx * dx + dy * dy);
                        if (dr > VslamOdometry.settings.dr_threashold || Math.abs(dh) > VslamOdometry.settings.dh_threashold) {
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
                    this.pushVslam(`${i}`, jpeg_data, (cur == 0 || cur == keys.length - 1));

                    console.log(`timestamp ${i} : ${jpeg_filepath}`);

                    ref_cur = cur;
                }

                m_client.publish('picam360-vslam', JSON.stringify({
                    "cmd": "backend",
                    "itr": 1,
                }));
                m_client.publish('picam360-vslam', JSON.stringify({
                    "cmd": "reset_pos",
                }));
            });

        });
    }

    is_ready() {
        return (this.current_odom != null);
    }

    static cal_xy(waypoints) {
        return null;
    }

    pushVslam(timestamp, jpeg_data, keyframe, backend) {
        m_client.publish('picam360-vslam', JSON.stringify({
            "cmd": "track",
            "timestamp": `${timestamp}`,
            "jpeg_data": jpeg_data.toString("base64"),
            "keyframe" : (keyframe || false),
            "backend": (backend || 0),
        }));
    }

    numToKey(num) {
        return num.toString().padStart(10, '0');
    }

    push(header, meta, jpeg_data) {
        this.enc_odom.push(header, meta, jpeg_data);
        this.enc_positions[this.push_cur] = this.enc_odom.getPosition();

        if(this.first_push){
            this.first_push = false;

            //this.pushVslam(`${this.push_cur}`, jpeg_data, true, 1);
            this.pushVslam(`${this.push_cur}`, jpeg_data, true, 0);
            this.backend_pending++;
            this.last_pushVslam_cur = this.push_cur;
        }

        {
            const pos = {
                x : this.enc_positions[this.push_cur].x - this.enc_positions[this.last_pushVslam_cur].x,
                y : this.enc_positions[this.push_cur].y - this.enc_positions[this.last_pushVslam_cur].y,
                heading : this.enc_positions[this.push_cur].heading - this.enc_positions[this.last_pushVslam_cur].heading,
            };

            const dr = Math.sqrt(pos.x*pos.x + pos.y*pos.y);
            const dh = pos.heading;
            if (dr > VslamOdometry.settings.dr_threashold || Math.abs(dh) > VslamOdometry.settings.dh_threashold) {
                console.log(`dr=${dr}, dh=${dh}`);

                //this.pushVslam(`${this.push_cur}`, jpeg_data, false, (this.backend_pending % 10) == 0);
                this.pushVslam(`${this.push_cur}`, jpeg_data, false, 0);
                this.backend_pending++;
                this.last_pushVslam_cur = this.push_cur;
            }
        }

        this.push_cur++;
    }

    getKeyframePosition() {

        function findClosestWaypoint(point, waypoints) {
            let closestKey = null;
            let minDistance = Infinity;
            const calculateDistance = (p1, p2) => {
                const dx = p1.x - p2.x;
                const dy = p1.y - p2.y;
                return Math.sqrt(dx * dx + dy * dy);
            };
            for (const [key, waypoint] of Object.entries(waypoints)) {
                const distance = calculateDistance(point, waypoint);
                if (distance < minDistance) {
                    minDistance = distance;
                    closestKey = key;
                }
            }
        
            return closestKey;
        }
        
        function getSurroundingKeys(keys, centerKey, range = 5) {
            const centerIndex = keys.indexOf(centerKey);
        
            if (centerIndex === -1) {
                throw new Error("Center key not found in keys array.");
            }
        
            const totalKeys = keys.length;
            const startIndex = Math.max(0, centerIndex - range); // 前方の開始インデックス
            const endIndex = Math.min(totalKeys, centerIndex + range + 1); // 後方の終了インデックス（+1はslice用）
        
            // 前後の範囲を考慮して不足分を調整
            const preKeys = centerIndex - startIndex; // 前方に取れたキー数
            const postKeys = endIndex - centerIndex - 1; // 後方に取れたキー数
        
            const adjustStartIndex = Math.max(0, startIndex - (range - postKeys)); // 後方不足分を前方に補う
            const adjustEndIndex = Math.min(totalKeys, endIndex + (range - preKeys)); // 前方不足分を後方に補う
        
            return keys.slice(adjustStartIndex, adjustEndIndex);
        }

        const vslam_pos = Object.assign({}, this.active_points[this.current_odom['timestamp']]);
        
        const center_key = findClosestWaypoint(vslam_pos, this.vslam_waypoints);
        const keys = getSurroundingKeys(Object.keys(this.vslam_waypoints), center_key, 1);

        const vslam_waypoints = {};
        const enc_waypoints = {};
        for(const key of keys){
            vslam_waypoints[key] = Object.assign({}, this.vslam_waypoints[key]);
            enc_waypoints[key] = Object.assign({}, this.enc_waypoints[key]);
        }

        const vslam_base_position = Object.assign({}, vslam_waypoints[keys[0]]);
        minus_offset_from_positions(vslam_waypoints, vslam_base_position);
        minus_offset_from_positions({ "keyframe" : vslam_pos }, vslam_base_position);

        const enc_base_position = Object.assign({}, enc_waypoints[keys[0]]);
        minus_offset_from_positions(enc_waypoints, enc_base_position);

        const settings = {
            scale : 1.0,
            heading_diff : 0.0,
        };
        cal_fitting_params(vslam_waypoints, enc_waypoints, settings);

        vslam_waypoints["keyframe"] = vslam_pos;
        scale_positions(vslam_waypoints, settings.scale);
        rotate_positions(vslam_waypoints, settings.heading_diff);
        add_offset_from_positions(vslam_waypoints, enc_base_position);

        let heading_error = 0.0;
        for(const key of keys){
            heading_error += enc_waypoints[key].heading - vslam_waypoints[key].heading;
        }
        heading_error /= keys.length;
        
        vslam_pos.heading += heading_error;

        return vslam_pos;
    }

    getPosition() {
        return this.enc_odom.getPosition();
    }
    calculateDistance(cur) {
        return this.enc_odom.calculateDistance(cur);
    }
    calculateBearing(cur, pos) {
        return this.enc_odom.calculateBearing(cur);
    }
    calculateHeadingError(cur) {
        return this.enc_odom.calculateHeadingError(cur);
    }
}

module.exports = {
    VslamOdometry
};