
module.exports = {
	create_plugin: function (plugin_host) {
		console.log("create motor plugin");
		const fs = require("fs");
		const path = require('path');
		const nmea = require('nmea-simple');
		const xml2js = require('xml2js');

		let m_options = {
			"waypoint_threshold_m" : 10,
		};

		let m_active_recorder_filepath = "";
		let m_active_recorder_filepath_watch = null;
		let m_active_drive_path = {};
		let m_active_drive_path_buffer = {};
		
		function read_pif_meta(filename, callback) {
			try{

				fs.readFile(filename, (err, data) => {
					if (err) throw err;
	
					// 最初の2バイトを取得
					const header = data.slice(0, 2).toString('utf-8');
					if (header !== 'PI') {
						throw new Error('Invalid file format');
					}
	
					// 次の4バイトでサイズを取得（ビッグエンディアン）
					const xmlSize = data.readUInt16BE(2);
	
					// XMLデータを取得してパース
					const xmlData = data.slice(4, 4 + xmlSize).toString('utf-8');
					xml2js.parseString(xmlData, (err, result) => {
						if (err) throw err;
	
						// METASIZE属性を取得
						const metaSize = parseInt(result["picam360:image"].$.meta_size, 10);
	
						// METASIZE分の文字列を読み込む
						const metaString = data.slice(4 + xmlSize, 4 + xmlSize + metaSize).toString('utf-8');
						xml2js.parseString(metaString, (err, result) => {
							if (err) throw err;
							callback(filename, result["picam360:frame"].$);
						});
					});
				});
			}catch(err){

			};
		}
		function set_active_recorder_path(data) {
			m_active_recorder_filepath = data;
			if(m_active_recorder_filepath_watch){
				m_active_recorder_filepath_watch.close();
				m_active_recorder_filepath_watch = null;
			}
			m_active_drive_path = {};
			m_active_drive_path_buffer = {};

			fs.readdir(m_active_recorder_filepath, { withFileTypes: true }, (err, entries) => {
				if (err) {
					console.error('Error reading directory:', err);
					return;
				}
				entries.forEach(entry => {
					if (entry.isFile()) {
						if(path.extname(entry.name) == ".pif"){
							const fullPath = path.join(m_active_recorder_filepath, entry.name);
							read_pif_meta(fullPath, (file_path, meta) => {
								const fileNameWithoutExt = path.basename(file_path, path.extname(file_path));
								m_active_drive_path_buffer[fileNameWithoutExt] = {
									nmea : meta.nmea,
									image : `/active_drive_path/${entry.name}.0.0.jpeg`,
								};
							});
							//console.log(`File: ${entry.name}`);
						}
					} else if (entry.isDirectory()) {
						//console.log(`Directory: ${entry.name}`);
					}
				});
				setInterval(() => {
					if(Object.keys(m_active_drive_path_buffer).length == 0){
						return;
					}
					m_active_drive_path = Object.assign(m_active_drive_path, m_active_drive_path_buffer);
					m_active_drive_path_buffer = {};
					const client = m_plugin_host.get_redis_client();
					if (client) {
						client.set('pserver-active-drive-path', JSON.stringify(m_active_drive_path)).then((data) => {
							console.log(data);
						});
					}
				}, 3000);
				m_active_recorder_filepath_watch = fs.watch(m_active_recorder_filepath, (eventType, filename) => {
					if (eventType === 'rename' && path.extname(filename) == ".pif") {
						const fullPath = path.join(m_active_recorder_filepath, filename);
						read_pif_meta(fullPath, (file_path, meta) => {
							const fileNameWithoutExt = path.basename(file_path, path.extname(file_path));
							m_active_drive_path_buffer[fileNameWithoutExt] = {
								nmea : meta.nmea,
								image : `/active_drive_path/${entry.name}.0.0.jpeg`,
							};
						});
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
							client.get('pserver-active-recorder-filepath').then((data) => {
								if(data && data != m_active_recorder_filepath){

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