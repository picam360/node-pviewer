
const nmea = require('nmea-simple');
const fxp = require('fast-xml-parser');
const xml_parser = new fxp.XMLParser({
	ignoreAttributes: false,
	attributeNamePrefix: "",
});

// Calculate the distance using the Haversine formula
function calculateDistance(lat1, lon1, lat2, lon2) {
    const R = 6371000; // Radius of the Earth in meters
    const φ1 = degreesToRadians(lat1);
    const φ2 = degreesToRadians(lat2);
    const Δφ = degreesToRadians(lat2 - lat1);
    const Δλ = degreesToRadians(lon2 - lon1);

    const a = Math.sin(Δφ / 2) ** 2 +
              Math.cos(φ1) * Math.cos(φ2) * Math.sin(Δλ / 2) ** 2;
    const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));

    return R * c; // Distance in meters
}

// Calculate the bearing from the current location to the target
function calculateBearing(lat1, lon1, lat2, lon2) {
    const φ1 = degreesToRadians(lat1);
    const φ2 = degreesToRadians(lat2);
    const Δλ = degreesToRadians(lon2 - lon1);

    const y = Math.sin(Δλ) * Math.cos(φ2);
    const x = Math.cos(φ1) * Math.sin(φ2) -
              Math.sin(φ1) * Math.cos(φ2) * Math.cos(Δλ);
    const θ = Math.atan2(y, x);

    return (radiansToDegrees(θ) + 360) % 360; // Bearing in degrees
}

// Convert degrees to radians
function degreesToRadians(degrees) {
    return degrees * (Math.PI / 180);
}

// Convert radians to degrees
function radiansToDegrees(radians) {
    return radians * (180 / Math.PI);
}

class GpsOdometry {
    constructor() {
        this.waypoints = null;
        this.current_nmea = null;
        this.current_imu = null;
    }

    init(waypoints, callback){
        this.waypoints = waypoints;
        this.waypoints_keys = Object.keys(waypoints);
        if(callback){
            callback();
        }
    }
  
    push(header, meta, jpeg_data) {
        const frame_dom = xml_parser.parse(meta);
        this.current_nmea = nmea.parseNmeaSentence(frame_dom['picam360:frame']['passthrough:nmea']);
        this.current_imu = JSON.parse(frame_dom['picam360:frame']['passthrough:imu']);
    }

    calculateDistance(cur){
        const key = this.waypoints_keys[cur];
        const target_waypoint = this.waypoints[key];
		const target_nmea = nmea.parseNmeaSentence(target_waypoint['nmea']);
        return calculateDistance(
            target_nmea.latitude, target_nmea.longitude,
            this.current_nmea.latitude, this.current_nmea.longitude);
    }
    calculateBearing(cur){
        const key = this.waypoints_keys[cur];
        const target_waypoint = this.waypoints[key];
		const target_nmea = nmea.parseNmeaSentence(target_waypoint['nmea']);
        return calculateBearing(
            target_nmea.latitude, target_nmea.longitude,
            this.current_nmea.latitude, this.current_nmea.longitude);
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
    GpsOdometry
  };