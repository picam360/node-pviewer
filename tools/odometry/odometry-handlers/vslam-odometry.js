
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
    const yaw = Math.atan2(matrix[2][0], matrix[0][0]);
    return yaw;
}

function launchDockerContainer() {
    const command = `
        source /home/picam360/miniconda3/etc/profile.d/conda.sh && \
        conda activate droidslam && \
        python ${VslamOdometry.settings.vslam_path}/vslam.py ${VslamOdometry.settings.vslam_option}
    `;
    const vslam_process = spawn(command, { shell: '/bin/bash', cwd: path.resolve(VslamOdometry.settings.vslam_path) });
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
    const processName = "picam360-vslam"; // プロセス名を指定

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
function calcScaleFromPointPairs(points0, points1) {
    const keys = Object.keys(points0).filter(k => points1[k]);
    if (keys.length === 0) return null;

    let cx0 = 0, cy0 = 0, cx1 = 0, cy1 = 0;
    for (const k of keys) {
        cx0 += points0[k].x;
        cy0 += points0[k].y;
        cx1 += points1[k].x;
        cy1 += points1[k].y;
    }
    cx0 /= keys.length;
    cy0 /= keys.length;
    cx1 /= keys.length;
    cy1 /= keys.length;

    let var0 = 0, var1 = 0;
    for (const k of keys) {
        const dx0 = points0[k].x - cx0;
        const dy0 = points0[k].y - cy0;
        const dx1 = points1[k].x - cx1;
        const dy1 = points1[k].y - cy1;
        var0 += dx0 * dx0 + dy0 * dy0;
        var1 += dx1 * dx1 + dy1 * dy1;
    }

    if (var0 === 0) return null;
    return Math.sqrt(var1 / var0);
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
function createErrorFunction(vslam_waypoints, ref_waypoints, settings, types) {
    return (solving_params) => {
        for(const i in types){
            settings[types[i]] = solving_params[i];
        }
        const vslam_positions = JSON.parse(JSON.stringify(vslam_waypoints));
        const keys = Object.keys(vslam_positions);
        const enc_positions = {};
        for(const key of keys){
            enc_positions[key] = Object.assign({}, ref_waypoints[key]);
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
    if(!keys2.length){
        return settings;
    }

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

function convert_transforms_to_positions(nodes, _ref_waypoints){
    function convert_transform_to_pos(node){
        const heading = -getYawFromRotationMatrix(node.transform) * 180 / Math.PI;
        return {
            x : node.transform[0][3],
            y : node.transform[2][3],
            heading : ((heading + 180) % 360) - 180,//-180 < heading < 180
        };
    }
    const vslam_waypoints = {};
    for(const node of nodes){
        vslam_waypoints[node['timestamp']] = convert_transform_to_pos(node);
    }
    return vslam_waypoints;
}

function euclidean(p1, p2) {
    const dx = p1.x - p2.x;
    const dy = p1.y - p2.y;
    return Math.sqrt(dx * dx + dy * dy);
}

function computeCentroid(points) {
    const entries = Object.entries(points);
    const sum = entries.reduce((acc, [, p]) => {
        acc.x += p.x;
        acc.y += p.y;
        return acc;
    }, {x: 0, y: 0});

    return {
        x: sum.x / entries.length,
        y: sum.y / entries.length
    };
}

class VslamOdometry {
    // static settings = {
    //     vslam_path : '/home/picam360/github/picam360-vslam',
    //     vslam_option : '--disable_vis',
    //     //vslam_option : '',
    //     vslam_filename : 'map.data',
    //     cam_offset : {
    //         x : 0.0,
    //         y : 2.2,
    //         heading : 0,
    //     },

    //     dr_threashold_waypoint : 2.0,
    //     dh_threashold_waypoint : 10,
    //     dr_threashold : 1.0,
    //     dh_threashold : 10,

    //     launch_vslam : true,
    //     calib_enabled : false,
    // };
    static settings = {//jetchariot
        vslam_path : '/home/picam360/github/picam360-vslam',
        vslam_option : '--disable_vis',
        vslam_option : '',
        vslam_filename : 'map.data',
        cam_offset : {
            x : 0.0,
            y : 2.2,
            heading : 0,
        },
        initial_pos : {
            x : 0,
            y : 0.6,
            heading : 180,
        },

        //for auto-drive
        dr_threashold_waypoint : 0.1,
        dh_threashold_waypoint : 10,
        dr_threashold : 0.05,
        dh_threashold : 10,

        //for map
        // dr_threashold_waypoint : 0.2,
        // dh_threashold_waypoint : 20,
        // dr_threashold : 0.1,
        // dh_threashold : 10,

        launch_vslam : true,
        calib_enabled : false,
        kf_mode : "manual",//"manual", "auto"

        refpoints_calib : {
            x : 0,
            y : 0,
            heading : 0,
            scale : 1,
        },
    };
    constructor(options) {
        this.options = options || {};
        this.host = this.options.host || "localhost";

        this.initialized = false;
        this.waypoints = null;
        this.vslam_waypoints = null;
        this.vslam_refpoints = null;
        this.vslam_actpoints = null;
        this.push_cur = 0;
        this.m_client = null;
        this.m_subscriber = null;
        this.req_first_estimation = (VslamOdometry.settings.kf_mode == "auto");
        this.waiting_first_estimation = false;
        this.req_manual_estimation = false;
        this.last_pushVslam_cur = 0;
        this.last_odom_cur = 0;
        this.reconstruction_progress = 0;

        this.enc_available = false;
        this.enc_odom = new EncoderOdometry();
        this.enc_positions = {};
    }


    static get_data_path(){
        return `${VslamOdometry.settings.vslam_path}/reconstructions/${VslamOdometry.settings.vslam_filename}`;
    }

    static clear_reconstruction(){
        if (fs.existsSync(VslamOdometry.get_data_path())) { //reconstruct
            fs.rmSync(VslamOdometry.get_data_path());
            console.log(`${VslamOdometry.get_data_path()} has been deleted.`);
        }
    }

    init(callback) {
        this.enc_odom.init(() => {
            const host = this.host;
            const port = 6379;
            const redis = require('redis');
            this.m_client = redis.createClient({
                url: `redis://${host}:${port}`
            });
            this.m_client.on('error', (err) => {
                console.error('redis error:', err);
                this.m_client = null;
            });
            this.m_client.connect().then(() => {
                console.log('redis connected:');

                this.update_reconstruction_progress(0);
            });
            this.m_subscriber = this.m_client.duplicate();
            this.m_subscriber.on('error', (err) => {
                console.error('redis error:', err);
                this.m_subscriber = null;
            });
            this.m_subscriber.connect().then(() => {
                console.log('redis connected:');

                function apply_calib(points){
                    const new_points= JSON.parse(JSON.stringify(points));
                    add_offset_from_positions(new_points, {
                        x : VslamOdometry.settings.refpoints_calib.x,
                        y : VslamOdometry.settings.refpoints_calib.y,
                    });
                    rotate_positions(new_points, VslamOdometry.settings.refpoints_calib.heading);
                    scale_positions(new_points, VslamOdometry.settings.refpoints_calib.scale);
                    return new_points;
                }

                this.m_subscriber.subscribe('picam360-vslam-odom', (data, key) => {
                    //console.log(data);
                    const params = JSON.parse(data);

                    if(params['type'] == 'info'){
                        if(VslamOdometry.settings.launch_vslam && params['msg'] == 'startup'){
                            this.m_client.publish('picam360-vslam', JSON.stringify({
                                "cmd": "load",
                                "filename": VslamOdometry.settings.vslam_filename,
                            }));
                            this.update_reconstruction_progress(50);
                        }
                        return;
                    }

                    if(!this.initialized){
                        this.update_reconstruction_progress(Math.min(this.reconstruction_progress + 1, 90));
                        if(params['type'] == 'load'){
                            this.initialized = true;
                            this.current_odom = null;

                            const odom = params['odom'][params['odom'].length - 1];//last one
                            this.push_cur = Math.ceil(odom['timestamp'] / 10000) * 10000;

                            this.vslam_waypoints = convert_transforms_to_positions(params['transforms']);
                            this.vslam_refpoints = apply_calib(this.vslam_waypoints);
                            
                            if(callback){
                                callback();
                            }

                            setInterval(() => {
                                this.vslam_refpoints = apply_calib(this.vslam_waypoints);//update

                                m_client.publish('pserver-odometry-info', JSON.stringify({
                                    "mode" : "INFO",
                                    "state" : "REFPOINTS",
                                    "refpoints" : this.vslam_refpoints,
                                    "actpoints" : this.vslam_actpoints,
                                }));
                            }, 1000);

                            this.update_reconstruction_progress(100);
                        }
                    }else{
                        const odom = params['odom'][params['odom'].length - 1];//last one
                        const odom_cur = odom['timestamp'];
                        if(odom_cur > this.last_odom_cur){
                            this.current_odom = odom;
                            this.last_odom_cur = odom_cur;

                            const vslam_actpoints = convert_transforms_to_positions(params['transforms']);
                            const map_scale = calcScaleFromPointPairs(vslam_actpoints, this.vslam_refpoints);
                            scale_positions(vslam_actpoints, map_scale);
                            this.vslam_actpoints = vslam_actpoints;

                            const kf_pos = this.vslam_actpoints[odom_cur];
                            if(this.waiting_first_estimation){
                                this.waiting_first_estimation = false;
                                
                                this.enc_odom.encoder_params.x = kf_pos.x;
                                this.enc_odom.encoder_params.y = kf_pos.y;
                                this.enc_odom.encoder_params.heading = kf_pos.heading;
                            }else{

                                const update_gain = 0.3;
                                const update_r_cutoff = 2.0;
                                const update_h_cutoff = 2.0;

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
                            }
                        }
                    }
                });
            });//end of substract

            if(VslamOdometry.settings.launch_vslam){
                killDockerContainer();
                launchDockerContainer();
            }
        });//end of redis connected
    }

    update_reconstruction_progress(progress){
        this.reconstruction_progress = progress;
        if(this.m_client){
            this.m_client.publish('pserver-odometry-info', JSON.stringify({
                "mode" : "INFO",
                "state" : "VSLAM_RECONSTRUCTION_PROGRESS",
                "progress" : this.reconstruction_progress,
            }));
        }
    }

    deinit() {
        if(this.m_client){
            this.m_client.quit();
            this.m_client = null;
        }
        if(this.m_subscriber){
            this.m_subscriber.quit();
            this.m_subscriber = null;
        }
        if(VslamOdometry.settings.launch_vslam){
            killDockerContainer();
        }
    }

    is_ready() {
        return (this.current_odom != null);
    }

    static cal_xy(waypoints) {
        return null;
    }

    requestTrack(timestamp, jpeg_data, keyframe, backend) {
        this.m_client.publish('picam360-vslam', JSON.stringify({
            "cmd": "track",
            "timestamp": `${timestamp}`,
            "jpeg_data": jpeg_data.toString("base64"),
            "keyframe" : (keyframe || false),
            "backend": (backend || 0),
        }));
    }

    requestEstimation(timestamps, timestamp, jpeg_data, itr) {
        this.m_client.publish('picam360-vslam', JSON.stringify({
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
        if(!this.initialized){
            //TODO
            return;
        }
        this.enc_odom.push(header, meta, jpeg_data);
        this.enc_positions[this.push_cur] = this.enc_odom.getPosition();

        if(!this.enc_available){
            const frame_dom = xml_parser.parse(meta);
            if(frame_dom['picam360:frame']['passthrough:encoder']){
                this.enc_available = true;
            }
        }
        
        const frame_dom = xml_parser.parse(meta);
        this.enc_positions[this.push_cur].nmea = frame_dom['picam360:frame']['passthrough:nmea'];

        if(this.req_first_estimation){

            const keys = Object.keys(this.vslam_waypoints);
            const ref_timestamps = keys.slice(0, 5);
            this.requestEstimation(ref_timestamps, `${this.push_cur}`, jpeg_data, 8);
            console.log("requestEstimation", ref_timestamps);

            this.last_pushVslam_cur = this.push_cur;

            this.push_cur++;
            this.req_first_estimation = false;
            this.waiting_first_estimation = true;
            return;

        }else if(this.current_odom == null){

            this.push_cur++;
            return;
            
        }

        let req_estimation = false;
        if(this.enc_available && VslamOdometry.settings.kf_mode == "auto"){
            const pos = {
                x : this.enc_positions[this.push_cur].x - this.enc_positions[this.last_pushVslam_cur].x,
                y : this.enc_positions[this.push_cur].y - this.enc_positions[this.last_pushVslam_cur].y,
                heading : this.enc_positions[this.push_cur].heading - this.enc_positions[this.last_pushVslam_cur].heading,
            };

            const dr = Math.sqrt(pos.x*pos.x + pos.y*pos.y);
            const dh = pos.heading;
            if (dr > VslamOdometry.settings.dr_threashold || Math.abs(dh) > VslamOdometry.settings.dh_threashold) {
                req_estimation = true;
            }
        }

        if(this.req_manual_estimation){
            this.req_manual_estimation = false;
            req_estimation = true;
        }

        if(req_estimation){
            let points = this.getKeysByHeading(this.vslam_waypoints, this.enc_positions[this.push_cur].heading, 30);
            const num = Object.keys(points).length;
            if(num == 0){
                console.log("requestEstimation", `dr=${dr}, dh=${dh}`, "no reference");
                return;
            }else if(num > 5){
                points = this.selectFarPoints(points, 5);
            }
            const ref_timestamps = Object.keys(points);
            console.log("requestEstimation", `dr=${dr}, dh=${dh}`, ref_timestamps);
    
            this.requestEstimation(ref_timestamps, `${this.push_cur}`, jpeg_data, 4);
            this.last_pushVslam_cur = this.push_cur;

            this.enc_positions[this.push_cur].keyframe = true;
        }

        this.push_cur++;
    }

    getKeysByHeading(waypoints, heading, criteria) {
        const points = {};
        for(const key of Object.keys(waypoints)){
            const diff_h = formay_angle_180(waypoints[key].heading - heading);
            if(Math.abs(diff_h) < criteria){
                points[key] = waypoints[key];
            }
        }
    
        return points;
    }
    
    selectFarPoints(points, count) {
        const entries = Object.entries(points).map(([key, value]) => [Number(key), value]);
        const selected = [];
    
        const centroid = computeCentroid(points);
    
        let firstKey = null;
        let minDist = Infinity;
        for (const [key, p] of entries) {
            const dist = euclidean(p, centroid);
            if (dist < minDist) {
                minDist = dist;
                firstKey = key;
            }
        }
    
        selected.push(firstKey);
    
        while (selected.length < count) {
            let bestKey = null;
            let bestDistance = -1;
    
            for (const [key, p] of entries) {
                if (selected.includes(key)) continue;
    
                const minDistToSelected = Math.min(...selected.map(selKey =>
                    euclidean(p, points[selKey])
                ));
    
                if (minDistToSelected > bestDistance) {
                    bestDistance = minDistToSelected;
                    bestKey = key;
                }
            }
    
            if (bestKey !== null) selected.push(bestKey);
            else break;
        }
        selected.sort((a, b) => a - b);
    
        const result = {};
        for (const key of selected) {
            result[key] = points[key];
        }
    
        return result;
    }

    getPosition() {
        return this.enc_odom.getPosition();
    }

    calib_odom(dodom){
        this.enc_odom.calib_odom(dodom);
    }

    calib_refpoints(dodom){
        VslamOdometry.settings.refpoints_calib.x += dodom.x || 0;
        VslamOdometry.settings.refpoints_calib.y += dodom.y || 0;
        VslamOdometry.settings.refpoints_calib.heading += dodom.heading || 0;
        VslamOdometry.settings.refpoints_calib.scale *= dodom.scale || 1.0;
    }

    setKfMode(mode) {
        if(mode == "manual" && VslamOdometry.settings.kf_mode == "manual"){
            this.req_manual_estimation = true;
        }
        VslamOdometry.settings.kf_mode = mode;
    }

    set_odom(odom){
        this.enc_odom.set_odom(odom);

        if(odom.heading == VslamOdometry.settings.initial_pos.heading){
            this.req_first_estimation = true;
        }
    }
}

module.exports = {
    VslamOdometry
};
