
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
    // static settings = {
    //     right_gain: 1.0,
    //     meter_per_pulse: 0.00902127575039515,
    //     wheel_separation: 3.093597564134178,
    //     imu_heading_error: 0.0,
    //     left_direction: -1,
    //     right_direction: 1,

    //     lock_gps_heading : true,
    // };
    static settings = {//jetchariot
        right_gain: 1.0,
        meter_per_pulse: 0.00004,
        wheel_separation: 0.208,
        imu_heading_error: 0,
        left_direction: -1,
        right_direction: 1,
    
       lock_gps_heading : true,
    };
    constructor() {
        this.encoder_params = {
            meter_per_pulse : null,
            wheel_separation : null,
            last_left_counts : null,
            last_right_counts : null,
            x : null,
            y : null,
            heading : null,
        };
    }

    static inclement_xy(encoder_params, left_counts, right_counts) {
        //encoder_params : {meter_per_pulse, wheel_separation, last_left_counts, last_right_counts, x, y, heading}
        try {
            let delta_left = left_counts - encoder_params.last_left_counts;
            let delta_right = right_counts - encoder_params.last_right_counts;

            delta_right *= encoder_params.right_gain;
    
            let distance_left = delta_left * encoder_params.meter_per_pulse;
            let distance_right = delta_right * encoder_params.meter_per_pulse;
    
            let distance = (distance_left + distance_right) / 2;
            let dtheta = (distance_right - distance_left) / encoder_params.wheel_separation;

            if (distance === 0 && dtheta === 0) {
                return;
            }

            //dtheta *= -1;//for jikki no ushiromuki
    
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

    static cal_xy(waypoints, settings){
        const positions = {};
        const keys = Object.keys(waypoints);
        const base_encoder = (waypoints[keys[0]].encoder ? JSON.parse(waypoints[keys[0]].encoder) : {
            left : 0,
            right : 0,
        });
        const encoder_params = {
            right_gain : settings.right_gain,
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
                current_encoder.right * settings.right_direction);
            positions[key] = {
                x : encoder_params.x,
                y : encoder_params.y,
                heading : encoder_params.heading,
            }
        }
        return positions;
    }

    init(callback){
        if(callback){
            callback();
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
            settings.heading_initial = 0;
            this.encoder_params = {
                right_gain : settings.right_gain,
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

    calib_odom(dodom){
        this.encoder_params.x += dodom.x;
        this.encoder_params.y += dodom.y;
        this.encoder_params.heading += dodom.heading;
    }
  }
  
  module.exports = {
    EncoderOdometry
  };