
const nmea = require('nmea-simple');
const fxp = require('fast-xml-parser');
const xml_parser = new fxp.XMLParser({
	ignoreAttributes: false,
	attributeNamePrefix: "",
});
const { GpsOdometry } = require('./gps-odometry');
const numeric = require('numeric');

// Convert degrees to radians
function degreesToRadians(degrees) {
    return degrees * (Math.PI / 180);
}

// Convert radians to degrees
function radiansToDegrees(radians) {
    return radians * (180 / Math.PI);
}

class EncoderOdometry {
    // static settings = {//juki
    //     lr_ratio_forward: 1.0,
    //     lr_ratio_backward: 1.0,
    //     meter_per_pulse: 0.00902127575039515,
    //     wheel_separation: 3.093597564134178,
    //     imu_heading_error: 0.0,
    //     left_direction: -1,
    //     right_direction: 1,

    //     lock_gps_heading : true,
    // };
    static settings = {//jetchariot
        lr_ratio_forward: 1.04,
        lr_ratio_backward: 0.96,
        meter_per_pulse: 0.00004,
        wheel_separation: 0.208,
        imu_heading_error: 0,
        left_direction: -1,
        right_direction: 1,
    
       lock_gps_heading : true,
    };
    constructor() {
        this.waypoints = null;
        this.waypoints_reverse = false;
        this.enc_waypoints = null;
        this.current_nmea = null;
        this.current_imu = null;
        this.encoder_params = {
            meter_per_pulse : null,
            wheel_separation : null,
            last_left_counts : null,
            last_right_counts : null,
            x : null,
            y : null,
            heading : null,
        };

        this.calib_enabled = false;
    }

    static inclement_xy(encoder_params, left_counts, right_counts, reverse) {
        //encoder_params : {meter_per_pulse, wheel_separation, last_left_counts, last_right_counts, x, y, heading}
        try {
            let delta_left = left_counts - encoder_params.last_left_counts;
            let delta_right = right_counts - encoder_params.last_right_counts;
            let lr_ratio_forward = (reverse ? encoder_params.lr_ratio_backward : encoder_params.lr_ratio_forward);
            let lr_ratio_backward = (reverse ? encoder_params.lr_ratio_forward : encoder_params.lr_ratio_backward);

            if(delta_left > 0){
                delta_left *= lr_ratio_forward;
            }else{
                delta_left *= lr_ratio_backward;
            }
    
            let distance_left = delta_left * encoder_params.meter_per_pulse;
            let distance_right = delta_right * encoder_params.meter_per_pulse;
    
            let distance = (distance_left + distance_right) / 2;
            let dtheta = (distance_right - distance_left) / encoder_params.wheel_separation;

            if (distance === 0 && dtheta === 0) {
                return;
            }

            //dtheta *= -1;//for jikki
    
            let dx, dy;
            const theta = -encoder_params.heading * Math.PI / 180 + Math.PI / 2;
            if (Math.abs(dtheta) < 1e-6) {
                dx = distance * Math.cos(theta);
                dy = distance * Math.sin(theta);
            } else {
                let radius = distance / dtheta;
                dx = radius * (Math.sin(theta + dtheta) - Math.sin(theta));
                dy = radius * (Math.cos(theta) - Math.cos(theta + dtheta));
            }

            //console.log("update encoder_params", distance_left, distance_right, encoder_params.x, dx, encoder_params.y, dy, encoder_params.heading, -dtheta * 180 / Math.PI);
    
            encoder_params.x += dx;
            encoder_params.y += dy;
            encoder_params.heading += -dtheta * 180 / Math.PI;
            encoder_params.heading = encoder_params.heading % 360;
    
            encoder_params.last_left_counts = left_counts;
            encoder_params.last_right_counts = right_counts;
        } catch (err) {
            console.error("Error in inclement_xy:", err);
        }
    }

    static cal_xy(waypoints, settings, reverse){
        const positions = {};
        const keys = Object.keys(waypoints);
        if(!keys.length){
            return {};
        }
        const base_encoder = (waypoints[keys[0]].encoder ? JSON.parse(waypoints[keys[0]].encoder) : {
            left : 0,
            right : 0,
        });
        const encoder_params = {
            lr_ratio_forward : settings.lr_ratio_forward,
            lr_ratio_backward : settings.lr_ratio_backward,
            meter_per_pulse : settings.meter_per_pulse,
            wheel_separation : settings.wheel_separation,
            last_left_counts : base_encoder.left * settings.left_direction,
            last_right_counts : base_encoder.right * settings.right_direction,
            x : settings.x_initial || 0,
            y : settings.y_initial || 0,
            heading : settings.heading_initial || 0,
        };
        for(const key of keys){
            const current_encoder = (waypoints[keys[key]].encoder ? JSON.parse(waypoints[keys[key]].encoder) : {
                left : 0,
                right : 0,
            });
            EncoderOdometry.inclement_xy(
                encoder_params,
                current_encoder.left * settings.left_direction,
                current_encoder.right * settings.right_direction,
                reverse
            );
            positions[key] = {
                x : encoder_params.x,
                y : encoder_params.y,
                heading : encoder_params.heading,
            };
        }
        return positions;
    }
    static cal_heading(waypoints, reverse){
        const waypoints_keys = Object.keys(waypoints);
        if(!waypoints_keys.length){
            return 0;
        }
        const settings = Object.assign({}, EncoderOdometry.settings);
        if(settings.lock_gps_heading){
            const gps_positions = GpsOdometry.cal_xy(waypoints);
            const enc_positions = EncoderOdometry.cal_xy(waypoints, settings, reverse);
            const last_key = waypoints_keys[waypoints_keys.length - 1];

            const first_gps_position = gps_positions[0];
            const last_gps_position = gps_positions[last_key];
            const gps_heading = Math.atan2(
                last_gps_position.x - first_gps_position.x, 
                last_gps_position.y - first_gps_position.y,
            ) * 180 / Math.PI;//from y axis

            const first_enc_position = enc_positions[0];
            const last_enc_position = enc_positions[last_key];
            const enc_heading = Math.atan2(
                last_enc_position.x - first_enc_position.x,
                last_enc_position.y - first_enc_position.y,
            ) * 180 / Math.PI;//from y axis

            return gps_heading - enc_heading;
        }else{
            const base_imu = JSON.parse(waypoints[waypoints_keys[0]].imu);
            return base_imu.heading + settings.imu_heading_error;
        }
    }

    init(waypoints, callback, reverse){
        this.waypoints = waypoints;
        this.waypoints_reverse = reverse;
        this.waypoints_keys = Object.keys(waypoints);

        // {
        //     const points = [];
        //     for(const waypoint of waypoints){
        //         const parsedData = nmea.parseNmeaSentence(waypoint.nmea);
        //         const { easting, northing } = 
        //         	utm.fromLatLon(parsedData.latitude, parsedData.longitude);
        //         points.push([easting, northing]);
        //     }
        //     const simplify = require('simplify-geometry');
        //     console.log(simplify(points, 1.0));
        // }
        {
            function createErrorFunction(waypoints, gps_positions, settings, types) {
                return (solving_params) => {
                    let gain = 1.0;
                    if (settings.meter_per_pulse < 0){
                        gain = Math.abs(settings.meter_per_pulse + 1) * 100;
                    }
                    if (settings.meter_per_pulse > 100) {
                        gain = Math.abs(settings.meter_per_pulse);
                    }
                    if (settings.wheel_separation < 0){
                        gain = Math.abs(settings.wheel_separation + 1) * 100;
                    }
                    if (settings.wheel_separation > 100) {
                        gain = Math.abs(settings.wheel_separation);
                    }
                    if (Math.abs(settings.lr_ratio_forward) > 1.5) {
                        gain = Math.abs(settings.lr_ratio_forward);
                    }
                    if (Math.abs(settings.lr_ratio_forward) < 0.5) {
                        gain = 1.0 / Math.abs(settings.lr_ratio_forward);
                    }
                    for(const i in types){
                        settings[types[i]] = solving_params[i];
                    }
                    const positions = EncoderOdometry.cal_xy(waypoints, settings);
                    const keys = Object.keys(positions);

                    let totalError = 0;
                    for(const key of keys){
                        const error_X = (positions[key].x - gps_positions[key].x) ** 2;
                        const error_Y = (positions[key].y - gps_positions[key].y) ** 2;
                        totalError += error_X + error_Y;
                    }
                
                    return totalError * gain;
                }
            }
            
            this.gps_positions = GpsOdometry.cal_xy(waypoints);
            if(Object.keys(this.gps_positions).length){
                const keys = Object.keys(this.gps_positions);
                const gps_base_x = this.gps_positions[keys[0]].x;
                const gps_base_y = this.gps_positions[keys[0]].y;
                for(const key of keys){
                    this.gps_positions[key].x -= gps_base_x;
                    this.gps_positions[key].y -= gps_base_y;
                }
            }
            
            if(this.calib_enabled){
                //types
                //    meter_per_pulse
                //    wheel_separation
                //    imu_heading_error
                let result = numeric.uncmin(
                    createErrorFunction(waypoints, this.gps_positions, Object.assign({}, EncoderOdometry.settings), ["imu_heading_error"]),
                    [EncoderOdometry.settings.imu_heading_error]);
                EncoderOdometry.settings.imu_heading_error = result.solution[0] % 360;
                console.log(result.message, result.f, result.iterations);
                console.log('imu_heading_error:', result.solution[0]);
                
                result = numeric.uncmin(
                    createErrorFunction(waypoints, this.gps_positions, Object.assign({}, EncoderOdometry.settings), ["lr_ratio_forward"]),
                    [EncoderOdometry.settings.lr_ratio_forward]);
                //EncoderOdometry.settings.lr_ratio_forward = result.solution[0];
                console.log(result.message, result.f, result.iterations);
                console.log('lr_ratio_forward:', result.solution[0]);

                result = numeric.uncmin(
                    createErrorFunction(waypoints, this.gps_positions, Object.assign({}, EncoderOdometry.settings), ["meter_per_pulse"]),
                    [EncoderOdometry.settings.meter_per_pulse]);
                EncoderOdometry.settings.meter_per_pulse = result.solution[0];
                console.log(result.message, result.f, result.iterations);
                console.log('meter_per_pulse:', result.solution[0]);

                result = numeric.uncmin(
                    createErrorFunction(waypoints, this.gps_positions, Object.assign({}, EncoderOdometry.settings), ["wheel_separation"]),
                    [EncoderOdometry.settings.wheel_separation]);
                EncoderOdometry.settings.wheel_separation = result.solution[0];
                console.log(result.message, result.f, result.iterations);
                console.log('wheel_separation:', result.solution[0]);

                result = numeric.uncmin(
                    createErrorFunction(waypoints, this.gps_positions, Object.assign({}, EncoderOdometry.settings), ["meter_per_pulse", "wheel_separation", "imu_heading_error"]),
                    [EncoderOdometry.settings.meter_per_pulse, EncoderOdometry.settings.wheel_separation, EncoderOdometry.settings.imu_heading_error]);
                EncoderOdometry.settings.meter_per_pulse = result.solution[0];
                EncoderOdometry.settings.wheel_separation = result.solution[1];
                EncoderOdometry.settings.imu_heading_error = result.solution[2] % 360;
                console.log(result.message, result.f, result.iterations);
                console.log('meter_per_pulse:', result.solution[0]);
                console.log('wheel_separation:', result.solution[1]);
                console.log('imu_heading_error:', result.solution[2]);
            }
        }
        {
            const settings = Object.assign({}, EncoderOdometry.settings);
            settings.x_initial = 0;
            settings.y_initial = 0;
            settings.heading_initial = EncoderOdometry.cal_heading(this.waypoints, this.waypoints_reverse);
            this.enc_waypoints = EncoderOdometry.cal_xy(this.waypoints, settings, this.waypoints_reverse);
        }

        if(callback){
            callback(this.enc_waypoints);
        }
    }

    deinit() {
    }

    is_ready() {
        return true;
    }
  
    push(header, meta, jpeg_data) {
        const frame_dom = xml_parser.parse(meta);
        if(frame_dom['picam360:frame']['passthrough:nmea']){
            this.current_nmea = nmea.parseNmeaSentence(frame_dom['picam360:frame']['passthrough:nmea']);
        }else{
            this.current_nmea = null;
        }
        if(frame_dom['picam360:frame']['passthrough:imu']){
            this.current_imu = JSON.parse(frame_dom['picam360:frame']['passthrough:imu']);
        }else{
            this.current_imu = {heading : 0};
        }
        if(frame_dom['picam360:frame']['passthrough:encoder']){
            this.current_encoder = JSON.parse(frame_dom['picam360:frame']['passthrough:encoder']);
        }else{
            this.current_encoder = {
                left : 0,
                right : 0,
            };
        }

        if(this.encoder_params.last_left_counts === null){
            const settings = Object.assign({}, EncoderOdometry.settings);
            settings.x_initial = 0;
            settings.y_initial = 0;
            if(this.waypoints.length){
                settings.heading_initial = EncoderOdometry.cal_heading(this.waypoints, this.waypoints_reverse);
            }else{
                settings.heading_initial = this.current_imu.heading;//jissaitonozure ha kouryosubeki
            }
            this.encoder_params = {
                lr_ratio_forward : settings.lr_ratio_forward,
                lr_ratio_backward : settings.lr_ratio_backward,
                meter_per_pulse : settings.meter_per_pulse,
                wheel_separation : settings.wheel_separation,
                last_left_counts : this.current_encoder.left * settings.left_direction,
                last_right_counts : this.current_encoder.right * settings.right_direction,
                x : settings.x_initial || 0,
                y : settings.y_initial || 0,
                heading : settings.heading_initial || 0,
            };
        }
        EncoderOdometry.inclement_xy(
            this.encoder_params,
            this.current_encoder.left * EncoderOdometry.settings.left_direction,
            this.current_encoder.right * EncoderOdometry.settings.right_direction);
    }

    getPosition(){
        return {
            x : this.encoder_params.x,
            y : this.encoder_params.y,
            heading : this.encoder_params.heading,
        };
    }

    calculateDistance(cur){
        function getVerticalAndHorizontalDistance(X, Y, headingDeg, Px, Py) {
            const theta = (90 - headingDeg) * Math.PI / 180;
            
            const dx = Px - X;
            const dy = Py - Y;
            
            const forwardX = Math.cos(theta);
            const forwardY = Math.sin(theta);
          
            const rightX = Math.cos(theta + Math.PI / 2);
            const rightY = Math.sin(theta + Math.PI / 2);
            const forwardDist = dx * forwardX + dy * forwardY;
            const rightDist = dx * rightX + dy * rightY;
          
            return [forwardDist, rightDist];
        }
        const key = this.waypoints_keys[cur];
        const pos = this.enc_waypoints[key];
        return getVerticalAndHorizontalDistance(this.encoder_params.x, this.encoder_params.y, this.encoder_params.heading, pos.x, pos.y);
    }
    calculateHeadingError(cur){
        const key = this.waypoints_keys[cur];
        const target_position = this.enc_waypoints[key];
		let headingError = target_position.heading - this.encoder_params.heading;
		if(headingError <= -180){
			headingError += 360;
		}else if(headingError > 180){
			headingError -= 360;
		}
        return headingError;
    }
  }
  
  module.exports = {
    EncoderOdometry
  };