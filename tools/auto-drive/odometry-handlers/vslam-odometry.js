
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

const vslam_path = '/home/picam360/github/picam360-vslam';

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
        positions[key].heading += cam_offset.heading;
        const offset = rotate_vec(cam_offset, positions[key].heading);
        positions[key].x += offset.x;
        positions[key].y += offset.y;
    }
}
function formay_angle_180(angle){
    angle = (angle % 360);
    if(angle > 180){
        angle -= 360;
    }
    return angle;
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
function cal_camera_heading_offset(vslam_positions, enc_positions){
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
        const vslam_positions = JSON.parse(JSON.stringify(vslam_waypoints));
        const keys = Object.keys(vslam_positions);
        const enc_positions = {};
        for(const key of keys){
            enc_positions[key] = Object.assign({}, enc_waypoints[key]);
        }

        scale_positions(vslam_positions, settings.scale);
        aply_cam_offset(vslam_positions, {
            x : settings.cam_offset_x,
            y : settings.cam_offset_y,
            heading : settings.cam_offset_heading,
        });
        rotate_positions(vslam_positions, settings.heading_diff);

        const vslam_base = Object.assign({}, vslam_positions[keys[0]]);//need to be after aply_cam_offset
        minus_offset_from_positions(vslam_positions, vslam_base);

        const enc_base = Object.assign({}, enc_positions[keys[0]]);
        minus_offset_from_positions(enc_positions, enc_base);

        let totalError = 0;
        for(const key of keys){
            const error_X = (vslam_positions[key].x - enc_positions[key].x) ** 2;
            const error_Y = (vslam_positions[key].y - enc_positions[key].y) ** 2;
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

    const first_heading_diff = first_enc_position.heading - first_vslam_position.heading;
    const last_heading_diff = last_enc_position.heading - last_vslam_position.heading;
    const cam_offset_heading = (first_heading_diff + last_heading_diff) / 2 - heading_diff;
    console.log("cam_offset_heading estimation", cam_offset_heading);

    settings = settings || {};
    settings.scale = scale;
    settings.heading_diff = heading_diff;
    settings.cam_offset_x = settings.cam_offset_x || 0.0;
    settings.cam_offset_y = settings.cam_offset_y || 0.0;
    settings.cam_offset_heading = settings.cam_offset_heading || 0.0;
    
    let result = numeric.uncmin(
        createErrorFunction(vslam_waypoints, enc_waypoints, settings, ["scale", "heading_diff"]),
        [settings.scale, settings.heading_diff]);
    settings.scale = result.solution[0];
    settings.heading_diff = result.solution[1] % 360;

    //createErrorFunction(vslam_waypoints, enc_waypoints, settings, ["scale", "heading_diff"])([settings.scale, settings.heading_diff]);

    return settings;
}

function convert_transforms_to_positions(nodes, _enc_waypoints){
    function convert_transform_to_pos(node){
        const heading = getYawFromRotationMatrix(node.transform) * 180 / Math.PI;
        return {
            x : node.transform[0][3],
            y : node.transform[2][3],
            heading : (heading + 180) % 360,//back camera
        };
    }
    const keys = Object.keys(_enc_waypoints);
    const enc_waypoints = {};
    const vslam_waypoints = {};
    const active_points = {};
    for(const node of nodes){
        //const cur = keys.length - 1 - node['timestamp'];//reverse
        const cur = node['timestamp'];
        if(keys[cur] !== undefined){
            vslam_waypoints[keys[cur]] = convert_transform_to_pos(node);
            enc_waypoints[keys[cur]] = Object.assign({}, _enc_waypoints[keys[cur]]);
        }else{
            active_points[node['timestamp']] = convert_transform_to_pos(node);
        }
    }

    const settings = {
        scale : 1.0,
        heading_diff : 0.0,
        cam_offset_x : VslamOdometry.settings.cam_offset.x,
        cam_offset_y : VslamOdometry.settings.cam_offset.y,
        cam_offset_heading : VslamOdometry.settings.cam_offset.heading,
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
        heading : settings.cam_offset_heading,
    });
    rotate_positions(vslam_waypoints, settings.heading_diff);

    const vslam_base = Object.assign({}, vslam_waypoints[first_key]);
    minus_offset_from_positions(vslam_waypoints, vslam_base);//need to be after aply_cam_offset
    const enc_base = Object.assign({}, enc_waypoints[first_key]);
    add_offset_from_positions(vslam_waypoints, enc_base);

    scale_positions(active_points, settings.scale);
    aply_cam_offset(active_points, {
        x : settings.cam_offset_x,
        y : settings.cam_offset_y,
        heading : settings.cam_offset_heading,
    });
    rotate_positions(active_points, settings.heading_diff);
    minus_offset_from_positions(active_points, vslam_base);//need to be after aply_cam_offset
    add_offset_from_positions(active_points, enc_base);

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
            heading : -8.5,
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
                        if(params['type'] == 'load' || params['type'] == 'backend'){
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
                            this.active_points = Object.assign(this.active_points, active_points);

                            const update_gain = 0.1;
                            const update_r_cutoff = 0.5;
                            const update_h_cutoff = 1.0;

                            const kf_pos = this.active_points[odom_cur];
                            const enc_pos = this.enc_positions[odom_cur];
                            const diff_x = kf_pos.x - enc_pos.x;
                            const diff_y = kf_pos.y - enc_pos.y;
                            const diff_r = Math.sqrt(diff_x ** 2 + diff_y ** 2);
                            const diff_h = formay_angle_180(kf_pos.heading - enc_pos.heading);
                            const diff_h_abs = Math.abs(diff_h);
                            const update_gain_r = Math.min(diff_r, update_r_cutoff) / Math.max(diff_r, 1e-3) * update_gain;
                            const update_gain_h = Math.min(diff_h_abs, update_h_cutoff) / Math.max(diff_h_abs, 1e-3) * update_gain;
                            this.enc_odom.encoder_params.x += diff_x * update_gain_r;
                            this.enc_odom.encoder_params.y += diff_y * update_gain_r;
                            this.enc_odom.encoder_params.heading += diff_h * update_gain_h;

                            console.log("diff_x", diff_x, diff_x * update_gain_r);
                            console.log("diff_y", diff_y, diff_y * update_gain_r);
                            console.log("diff_h", diff_h, diff_h * update_gain_h);

                            if(this.options.transforms_callback){
                                this.options.transforms_callback(this.vslam_waypoints, this.active_points);
                            }

                            if(true){//debug
                                const nmea = this.enc_positions[odom_cur].nmea;
                                for(const key in this.waypoints){
                                    if(this.waypoints[key].nmea == nmea){
                                        const enc_pos2 = this.enc_waypoints[key];
                                        const kf_pos2 = vslam_waypoints[key];
                                        console.log("diff", key, {
                                            kf_pos,
                                            enc_pos,
                                            kf_pos2,
                                            enc_pos2,
                                        });
                                        break;
                                    }
                                }
                            }
                        }
                    }
                });

                const filename = "waypoints.data";
                const data_path = `${vslam_path}/reconstructions/${filename}`;
				if (fs.existsSync(data_path)) {
                    m_client.publish('picam360-vslam', JSON.stringify({
                        "cmd": "load",
                        "filename": filename,
                    }));
                }else{ //reconstruct

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
                        //this.requestTrack(`${i}`, jpeg_data, (cur == 0 || cur == keys.length - 1));
                        this.requestTrack(`${i}`, jpeg_data, true);
    
                        console.log(`timestamp ${i} : ${jpeg_filepath}`);
    
                        ref_cur = cur;
                    }
    
                    m_client.publish('picam360-vslam', JSON.stringify({
                        "cmd": "backend",
                        "itr": 8,
                    }));
                    m_client.publish('picam360-vslam', JSON.stringify({
                        "cmd": "save",
                        "filename": filename,
                    }));
                    m_client.publish('picam360-vslam', JSON.stringify({
                        "cmd": "reset_pos",
                    }));
                }
            });

        });
    }

    is_ready() {
        return (this.current_odom != null);
    }

    static cal_xy(waypoints) {
        return null;
    }

    requestTrack(timestamp, jpeg_data, keyframe, backend) {
        m_client.publish('picam360-vslam', JSON.stringify({
            "cmd": "track",
            "timestamp": `${timestamp}`,
            "jpeg_data": jpeg_data.toString("base64"),
            "keyframe" : (keyframe || false),
            "backend": (backend || 0),
        }));
    }

    requestEstimation(timestamps, timestamp, jpeg_data, itr) {
        m_client.publish('picam360-vslam', JSON.stringify({
            "cmd": "estimate",
            "timestamps": timestamps,
            "timestamp": `${timestamp}`,
            "jpeg_data": jpeg_data.toString("base64"),
            "itr": (itr || 1),
        }));
    }

    numToKey(num) {
        return num.toString().padStart(10, '0');
    }

    push(header, meta, jpeg_data) {
        this.enc_odom.push(header, meta, jpeg_data);
        this.enc_positions[this.push_cur] = this.enc_odom.getPosition();
        
        const frame_dom = xml_parser.parse(meta);
        this.enc_positions[this.push_cur].nmea = frame_dom['picam360:frame']['passthrough:nmea'];

        if(this.first_push){
            this.first_push = false;

            const keys = Object.keys(this.vslam_waypoints);
            //this.requestTrack(`${this.push_cur}`, jpeg_data, true, 1);
            const ref_timestamps = keys.slice(0, 5);
            this.requestEstimation(ref_timestamps, `${this.push_cur}`, jpeg_data, 8);
            this.backend_pending++;
            this.last_pushVslam_cur = this.push_cur;
        }else if(this.current_odom == null){
            return;
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
        
                const center_key = this.findClosestWaypoint(this.enc_positions[this.push_cur], this.vslam_waypoints);
                const ref_timestamps = this.getSurroundingKeys(Object.keys(this.vslam_waypoints), center_key, 1);

                console.log("diff", ref_timestamps);
        
                //this.requestTrack(`${this.push_cur}`, jpeg_data, false, (this.backend_pending % 10) == 0);
                this.requestEstimation(ref_timestamps, `${this.push_cur}`, jpeg_data, 4);
                this.backend_pending++;
                this.last_pushVslam_cur = this.push_cur;

                this.enc_positions[this.push_cur].keyframe = true;
            }
        }

        this.push_cur++;
    }

    findClosestWaypoint(point, waypoints) {
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
    
    getSurroundingKeys(keys, centerKey, range = 5) {
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