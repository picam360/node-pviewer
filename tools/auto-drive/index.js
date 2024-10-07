const { J } = require('quaternion');
console.log("create motor plugin");
var fs = require("fs");
const nmea = require('nmea-simple');

var m_options = {
	"waypoint_threshold_m" : 10,
	"data_filepath" : "auto-drive-path"
};
var m_drive_mode = "STANBY";
var m_averaging_nmea = null;
var m_averaging_count = 0;
var m_last_nmea = null;
var m_auto_drive_data = {
	activated : false, //active
	path_confifg : {},
};

function latLonToXY(lat1, lon1, lat2, lon2) {
	const R = 6378137; // Earth's radius in meters
	
	// Convert latitude and longitude to radians
	const φ1 = lat1 * Math.PI / 180;
	const φ2 = lat2 * Math.PI / 180;
	const Δλ = (lon2 - lon1) * Math.PI / 180;
	
	// Calculate XY coordinates
	const x = R * Δλ * Math.cos((φ1 + φ2) / 2);
	const y = R * (φ2 - φ1);
	
	return { x, y };
}

function main() {

    const redis = require('redis');
    const client = redis.createClient({
        host: 'localhost',
        port: 6379,
    });
    client.on('error', (err) => {
        console.error('redis error:', err);
    });
    client.connect().then(() => {
        console.log('redis connected:');
	});

	const subscriber = client.duplicate();
	subscriber.connect().then(() => {
		console.log('redis connected:');

		subscriber.subscribe('pserver-auto-drive', (data, key) => {
			var params = data.trim().split(' ');
			switch (params[0]) {
				case "CMD":
					console.log(`"${data}" subscribed.`);
					command_handler(data.substr(params[0].length + 1));
					break;
			}
		});

		subscriber.subscribe('pserver-nmea', (data, key) => {
			const parsedData = nmea.parseNmeaSentence(data);

			if(m_averaging_count == 0){
				m_averaging_nmea = parsedData;
			}else{
				m_averaging_nmea.latitude += parsedData.latitude;
				m_averaging_nmea.longitude += parsedData.longitude;
			}
			m_averaging_count++;
			
			if(m_averaging_count == 10){
				m_averaging_nmea.latitude /= m_averaging_count;
				m_averaging_nmea.longitude /= m_averaging_count;
				m_averaging_count = 0;

				plugin.push_nmea(m_averaging_nmea);
			}
		});

		subscriber.subscribe('pserver-vslam-pst', (data, key) => {
			console.log('pserver-vslam-pst', data.length);
		});
	});
}
function command_handler(cmd) {
	var split = cmd.split(' ');
	switch(split[0]){
		case "RECORD_START":
			m_drive_mode = "RECORD";
			break;
		case "RECORD_STOP":
			m_drive_mode = "STANBY";
			break;
		case "AUTO_START":
			m_drive_mode = "AUTO";
			var config = split[1];
			
			var json_str = "";
			if(config.startsWith("data:text/plain;base64,")){
				Buffer.from(config.substr("data:text/plain;base64,".length), 'base64').toString('utf-8');
			}else{
				json_str = fs.readFileSync(config);
			}
			m_auto_drive_data.path_confifg = JSON.parse(json_str);
			break;
		case "AUTO_STOP":
			m_drive_mode = "STANBY";
			break;
	}
}
function push_nmea(nmea){
	if(!m_last_nmea){
		m_last_nmea = m_averaging_nmea;
		return;
	}

	const { x, y } = latLonToXY(
		m_last_nmea.latitude, m_last_nmea.longitude,
		m_averaging_nmea.latitude, m_averaging_nmea.longitude);
	console.log(`X: ${x} meters, Y: ${y} meters`);

	m_last_nmea = m_averaging_nmea;

	if(m_auto_drive_data.activated){

	}
}

if (require.main === module) {
    main();
}
