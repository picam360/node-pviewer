
module.exports = {
	create_plugin: function (plugin_host) {
		console.log("create motor plugin");
		const fs = require("fs");
		const path = require('path');
		const nmea = require('nmea-simple');

		let m_options = {
			"waypoint_threshold_m" : 10,
		};

		let m_active_recorder_path = "";
		let m_active_recorder_path_watch = null;
		
		function set_active_recorder_path(data) {
			m_active_recorder_path = data;
			if(m_active_recorder_path_watch){
				m_active_recorder_path_watch.close();
				m_active_recorder_path_watch = null;
			}

			fs.readdir(m_active_recorder_path, { withFileTypes: true }, (err, entries) => {
				if (err) {
					console.error('Error reading directory:', err);
					return;
				}
				const files = [];
				entries.forEach(entry => {
					if (entry.isFile()) {
						if(path.extname(entry.name) == ".pif"){
							files.push(entry.name);
							//console.log(`File: ${entry.name}`);
						}
					} else if (entry.isDirectory()) {
						//console.log(`Directory: ${entry.name}`);
					}
				});
				m_active_recorder_path_watch = fs.watch(m_active_recorder_path, (eventType, filename) => {
					if (eventType === 'rename' && path.extname(filename) == ".pif") {
						const fullPath = path.join(m_active_recorder_path, filename);
						if (fs.existsSync(fullPath) && fs.lstatSync(fullPath).isFile()) {
							console.log(`New file added: ${filename}`);
							files.push(filename);
						}
					}
				});
			});
		}

		var plugin = {
			name: "drive_path_from_recorder",
			init_options: function (options) {
				m_options = options["drive_path_from_recorder"];

				if (m_options.subscribe_redis && m_plugin_host.get_redis_client) {
					setInterval(() => {
						const client = m_plugin_host.get_redis_client();
						if (client) {
							client.get('pserver-active-recorder-path').then((data) => {
								if(data && data != m_active_recorder_path){

									try {
										const stats = fs.statSync(data);
										if (stats.isDirectory()) {
											set_active_recorder_path(data);
										} else {
											console.log('The path exists but is not a directory.');
										}
									} catch (err) {
										console.log('Error checking the path:', err);
									}
								}
							});
						}
					}, 1000);
				}
			},
			command_handler: function (cmd) {
			},
		};
		return plugin;
	}
};