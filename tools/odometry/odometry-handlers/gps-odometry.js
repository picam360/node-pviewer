
const nmea = require('nmea-simple');
const utm = require('utm');
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

function toWebMercator(lat, lon) {
    const R = 6378137; // 地球の赤道半径 (メートル)
    const x = R * lon * Math.PI / 180;
    const y = R * Math.log(Math.tan(Math.PI / 4 + (lat * Math.PI / 180) / 2));
    return { easting : x, northing : y }
    //return [x, y];
}

class GpsOdometry {
    static settings = {
    };
    constructor() {
        this.current_nmea = null;
        this.current_imu = null;
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

    static cal_xy(waypoints){
        const positions = {};
        const keys = Object.keys(waypoints);
        for(const key of keys){
            try{
                if(!waypoints[key].nmea){
                    positions[key] = {
                        x : 0,
                        y : 0,
                    }
                    continue;
                }
                const current_nmea = nmea.parseNmeaSentence(waypoints[key].nmea);
                const { easting, northing } = 
                    toWebMercator(current_nmea.latitude, current_nmea.longitude);
                    //utm.fromLatLon(current_nmea.latitude, current_nmea.longitude);
                positions[key] = {
                    x : easting,
                    y : northing,
                }
            }catch(err){
                console.log(err);
            }
        }
        return positions;
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
    }

    getPosition(){
        const current_post = 
            toWebMercator(this.current_nmea.latitude, this.current_nmea.longitude);
            //utm.fromLatLon(this.current_nmea.latitude, this.current_nmea.longitude);
        return {
            x : current_post.easting,
            y : current_post.northing,
            heading : this.current_imu.heading,
        };
    }
  }
  
  module.exports = {
    GpsOdometry
  };