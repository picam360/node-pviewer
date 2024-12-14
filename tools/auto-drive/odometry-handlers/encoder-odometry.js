
const nmea = require('nmea-simple');
const fxp = require('fast-xml-parser');
const xml_parser = new fxp.XMLParser({
	ignoreAttributes: false,
	attributeNamePrefix: "",
});
const gps_odometry = require('./gps-odometry');
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
    static left_direction = 1;
    static right_direction = 1;
    constructor() {
        this.waypoints = null;
        this.positions = null;
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

        this.calib_enabled = true;

        // this.settings = {
        //     right_gain: 1.0,
        //     meter_per_pulse: 0.0078445,
        //     wheel_separation: 2.5,
        //     imu_heading_error: 0.0,
        // };
        this.settings = {
            right_gain: 0.9299863819615063,
            meter_per_pulse: 0.000045879703712042854,
            wheel_separation: 0.28712089481288977,
            imu_heading_error: -16.79000670316058,
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
    
            encoder_params.x += dx;
            encoder_params.y += dy;
            encoder_params.heading += -dtheta * 180 / Math.PI;
    
            encoder_params.last_left_counts = left_counts;
            encoder_params.last_right_counts = right_counts;
        } catch (err) {
            console.error("Error in inclement_xy:", err);
        }
    }

    static cal_xy(waypoints, settings){
        const positions = {};
        const keys = Object.keys(waypoints);
        const base_encoder = JSON.parse(waypoints[keys[0]].encoder);
        const base_imu = JSON.parse(waypoints[keys[0]].imu);
        const encoder_params = {
            right_gain : settings.right_gain,
            meter_per_pulse : settings.meter_per_pulse,
            wheel_separation : settings.wheel_separation,
            last_left_counts : base_encoder.left * EncoderOdometry.left_direction,
            last_right_counts : base_encoder.right * EncoderOdometry.right_direction,
            x : 0,
            y : 0,
            heading : base_imu.heading + settings.imu_heading_error,
        };
        for(const key of keys){
            const current_encoder = JSON.parse(waypoints[key].encoder);
            EncoderOdometry.inclement_xy(
                encoder_params,
                current_encoder.left * EncoderOdometry.left_direction,
                current_encoder.right * EncoderOdometry.right_direction);
            positions[key] = {
                x : encoder_params.x,
                y : encoder_params.y,
            }
        }
        return positions;
    }

    init(waypoints, callback){
        this.waypoints = waypoints;
        this.waypoints_keys = Object.keys(waypoints);

        {
            function createErrorFunction(waypoints, gps_positions, settings, types) {
                return (solving_params) => {
                    let gain = 1.0;
                    if (settings.meter_per_pulse < 0 || settings.meter_per_pulse > 100) {
                        gain = Math.abs(settings.meter_per_pulse + 1);
                    }
                    if (settings.wheel_separation < 0 || settings.wheel_separation > 100) {
                        gain = Math.abs(settings.wheel_separation + 1);
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
            
            const gps_positions = gps_odometry.GpsOdometry.cal_xy(waypoints);
            {
                const keys = Object.keys(gps_positions);
                const gps_base_x = gps_positions[keys[0]].x;
                const gps_base_y = gps_positions[keys[0]].y;
                for(const key of keys){
                    gps_positions[key].x -= gps_base_x;
                    gps_positions[key].y -= gps_base_y;
                }
            }
            //types
            //    meter_per_pulse
            //    wheel_separation
            //    imu_heading_error
            let result = numeric.uncmin(
                createErrorFunction(waypoints, gps_positions, Object.assign({}, this.settings), ["imu_heading_error"]),
                [this.settings.imu_heading_error]);
            this.settings.imu_heading_error = result.solution[0] % 360;
            console.log(result.message, result.f, result.iterations);
            console.log('imu_heading_error:', result.solution[0]);
            
            if(this.calib_enabled){
                result = numeric.uncmin(
                    createErrorFunction(waypoints, gps_positions, Object.assign({}, this.settings), ["right_gain"]),
                    [this.settings.right_gain]);
                this.settings.right_gain = result.solution[0];
                console.log(result.message, result.f, result.iterations);
                console.log('right_gain:', result.solution[0]);

                result = numeric.uncmin(
                    createErrorFunction(waypoints, gps_positions, Object.assign({}, this.settings), ["meter_per_pulse"]),
                    [this.settings.meter_per_pulse]);
                this.settings.meter_per_pulse = result.solution[0];
                console.log(result.message, result.f, result.iterations);
                console.log('meter_per_pulse:', result.solution[0]);

                result = numeric.uncmin(
                    createErrorFunction(waypoints, gps_positions, Object.assign({}, this.settings), ["wheel_separation"]),
                    [this.settings.wheel_separation]);
                this.settings.wheel_separation = result.solution[0];
                console.log(result.message, result.f, result.iterations);
                console.log('wheel_separation:', result.solution[0]);

                result = numeric.uncmin(
                    createErrorFunction(waypoints, gps_positions, Object.assign({}, this.settings), ["right_gain", "meter_per_pulse", "wheel_separation", "imu_heading_error"]),
                    [this.settings.right_gain, this.settings.meter_per_pulse, this.settings.wheel_separation, this.settings.imu_heading_error]);
                this.settings.right_gain = result.solution[0];
                this.settings.meter_per_pulse = result.solution[1];
                this.settings.wheel_separation = result.solution[2];
                this.settings.imu_heading_error = result.solution[3] % 360;
                console.log(result.message, result.f, result.iterations);
                console.log('right_gain:', result.solution[0]);
                console.log('meter_per_pulse:', result.solution[1]);
                console.log('wheel_separation:', result.solution[2]);
                console.log('imu_heading_error:', result.solution[3]);
            }
        }
        this.positions = EncoderOdometry.cal_xy(waypoints, this.settings);

        if(callback){
            callback();
        }
    }
  
    push(header, meta, jpeg_data) {
        const frame_dom = xml_parser.parse(meta);
        this.current_nmea = nmea.parseNmeaSentence(frame_dom['picam360:frame']['passthrough:nmea']);
        this.current_imu = JSON.parse(frame_dom['picam360:frame']['passthrough:imu']);
        this.current_encoder = JSON.parse(frame_dom['picam360:frame']['passthrough:encoder']);

        if(this.encoder_params.last_left_counts === null){
            this.encoder_params = {
                right_gain : this.settings.right_gain,
                meter_per_pulse : this.settings.meter_per_pulse,
                wheel_separation : this.settings.wheel_separation,
                last_left_counts : this.current_encoder.left * EncoderOdometry.left_direction,
                last_right_counts : this.current_encoder.right * EncoderOdometry.right_direction,
                x : 0,
                y : 0,
                heading : this.current_imu.heading + this.settings.imu_heading_error,
            };
        }
        EncoderOdometry.inclement_xy(
            this.encoder_params,
            this.current_encoder.left * EncoderOdometry.left_direction,
            this.current_encoder.right * EncoderOdometry.right_direction);
    }

    calculateDistance(cur){
        const key = this.waypoints_keys[cur];
        const target_position = this.positions[key];
        const dx = target_position.x - this.encoder_params.x;
        const dy = target_position.y - this.encoder_params.y;
        return Math.sqrt(dx * dx + dy * dy);
    }
    calculateBearing(cur){
        const key = this.waypoints_keys[cur];
        const target_position = this.positions[key];
        const dx = target_position.x - this.encoder_params.x;
        const dy = target_position.y - this.encoder_params.y;
        const θ = Math.atan2(dy, dx);
        const bearing = (Math.PI / 2 - θ)
        return (radiansToDegrees(bearing) + 360) % 360; // Bearing in degrees
    }
    calculateHeadingError(cur){
        const targetHeading = this.calculateBearing(cur);
		let headingError = targetHeading - this.encoder_params.heading;
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