#! /usr/bin/env node
process.chdir(__dirname);
const os = require('os');
const path = require('path');
const child_process = require('child_process');
const async = require('async');
const fs = require("fs");
const moment = require("moment");
const sprintf = require('sprintf-js').sprintf;
const express = require('express');
const cors = require('cors');
const jsonc = require('jsonc-parser');
const WebSocket = require("ws");

const pstcore = require('node-pstcore');

var express_app = null;
var http = null;
var https = null;
var ws_server = null;
var wss_server = null;

var m_plugin_host = {};
var m_plugins = [];
var m_pstdefs = {};
var m_cmd_list = [];

var GC_THRESH = 16 * 1024 * 1024; // 16MB

var UPSTREAM_DOMAIN = "upstream.";

var m_options = {};
var m_pvf_filepath = null;
var m_calibrate = null;
var m_calibrate_hq = null;

function start_webserver(callback) { // start up websocket server
    console.log("websocket server starting up");
    express_app = express();
	express_app.use(cors());
	express_app.use(express.json());
    http = require('http').Server(express_app);
	http.keepAliveTimeout = 60000;

	var https_key_filepath = 'certs/https/localhost-key.pem';
	var https_cert_filepath = 'certs/https/localhost.pem';
	if(m_options['https_key_filepath'] &&
	   m_options['https_cert_filepath']){
		if(fs.existsSync(m_options['https_key_filepath']) &&
		   fs.existsSync(m_options['https_cert_filepath'])){
			https_key_filepath = m_options['https_key_filepath'];
			https_cert_filepath = m_options['https_cert_filepath'];
		}else{
			console.log("https key cert file not found.");
		}
	}
	var https_options = {
		key: fs.readFileSync(https_key_filepath),
		cert: fs.readFileSync(https_cert_filepath)
	};
	https = require('https').Server(https_options, express_app);
	https.keepAliveTimeout = 60000;

    express_app.get(/^\/img\/.*\.jpeg$/, function(req, res) {
		var url = req.url.split("?")[0];
		var query = req.url.split("?")[1];
		var filepath = 'userdata/' + url.split("/")[2];
		console.log(url);
		console.log(query);
		console.log(filepath);
		fs.readFile(filepath, function(err, data) {
			if (err) {
				res.writeHead(404);
				res.end();
				console.log("404");
			} else {
				res
					.writeHead(200, {
						'Content-Type': 'image/jpeg',
						'Content-Length': data.length,
						'Cache-Control': 'private, no-cache, no-store, must-revalidate',
						'Expires': '-1',
						'Pragma': 'no-cache',
					});
				res.end(data);
				console.log("200");
			}
		});
	});
    express_app.get(/^\/img\/.*\.mp4$/, function(req, res) {
        var url = req.url.split("?")[0];
        var query = req.url.split("?")[1];
        var filepath = 'userdata/' + url.split("/")[2];
        console.log(url);
        console.log(query);
        console.log(filepath);
        fs.readFile(filepath, function(err, data) {
            if (err) {
                res.writeHead(404);
                res.end();
                console.log("404");
            } else {
                var range = req.headers.range // bytes=0-1
                if (!range) {
                    res.writeHead(200, {
                        "Content-Type": "video/mp4",
                        "X-UA-Compatible": "IE=edge;chrome=1",
                        'Content-Length': data.length
                    });
                    res.end(data)
                } else {
                    var total = data.length;
                    var split = range.split(/[-=]/);
                    var ini = +split[1];
                    var end = split[2] ? +split[2] : total - 1;
                    var chunkSize = end - ini + 1;
                    res.writeHead(206, {
                        "Content-Range": "bytes " + ini + "-" + end +
                            "/" + total,
                        "Accept-Ranges": "bytes",
                        "Content-Length": chunkSize,
                        "Content-Type": "video/mp4",
                    })
                    res.end(data.slice(ini, chunkSize + ini))
                }
            }
        });
    });
    express_app.get(['/', '/index.html'], function(req, res) {
		console.log("start pviewer loading...");
		res.sendFile(path.resolve('../pviewer/index.html'));
    });
    express_app.use(express.static('../pviewer')); // this need be set
	var http_port = 36080;//PICAM360(36000)+HTTP(80)+APPCODE(0)
	if(m_options['http_port']){
		http_port = m_options['http_port'];
	}
    http.listen(http_port, function() {
        console.log('listening http on *:' + http_port);
    });

	var https_port = 36443;//PICAM360(36000)+HTTPS(443)+APPCODE(0)
	if(m_options['https_port']){
		https_port = m_options['https_port'];
	}
    https.listen(https_port, function() {
        console.log('listening https on *:' + https_port);
    });

	ws_server = new WebSocket.Server({ server : http });
	wss_server = new WebSocket.Server({ server : https });

    callback(null);
}

async.waterfall([
	function(callback) { // argv
		var wrtc_key = null;
		var conf_filepath = 'config.json';
		var replacements = {};
		for (var i = 0; i < process.argv.length; i++) {
			if (process.argv[i] == "-c") {//config
				conf_filepath = process.argv[i + 1];
				i++;
			}
			if (process.argv[i] == "-f") {//file
				m_pvf_filepath = process.argv[i + 1];
				i++;
			}
			if (process.argv[i] == "-w") {//wrtc or ws
				wrtc_key = process.argv[i + 1];
				i++;
			}
			if (process.argv[i] == "-r") {//replacements
				var [key, value] = process.argv[i + 1].split("=");
				replacements[key] = value;
				i++;
			}
			if (process.argv[i].startsWith("--calibrate=")) {
				m_calibrate = process.argv[i].split("=")[1];
			}
			if (process.argv[i].startsWith("--calibrate-hq=")) {
				m_calibrate_hq = process.argv[i].split("=")[1];
			}
		}
		if (!fs.existsSync(conf_filepath)) {
			conf_filepath = __dirname + "/" + conf_filepath;
		}
		if (fs.existsSync(conf_filepath)) {
			console.log("load config file : " + conf_filepath);
			
			var lines = fs.readFileSync(conf_filepath, 'utf-8').replace(/\r/g, '').split('\n')
			for(var i=0;i<lines.length;i++){
				if(lines[i][0] == '#'){
					lines[i] = "";
				}
			}
			var json_str = lines.join("\n");
			m_options = jsonc.parse(json_str);
		} else {
			m_options = {};
		}
		if(wrtc_key){
			m_options["wrtc"] = {
				"enabled" : true,
				"key" : wrtc_key
			};
		}
		if(m_options["pstdefs"]){
			m_options["pstdefs"]["replacements"] = Object.assign(m_options["pstdefs"]["replacements"] || {}, replacements);
		}

		callback(null);
	},
	function(callback) { // exit sequence
		function cleanup() {
		}
		process.on('uncaughtException', (err) => {
			cleanup();
			throw err;
		});
		process.on('SIGINT', function() {
			cleanup();
			console.log("exit process done");
			process.exit();
		});
		process.on('SIGUSR2', function() {
			if (agent.server) {
				agent.stop();
			} else {
				agent.start({
					port: 9999,
					bind_to: '192.168.3.103',
					ipc_port: 3333,
					verbose: true
				});
			}
		});
		callback(null);
	},
	function(callback) {
		if(m_options["license"] && m_options["license"]["app_key"]){
			console.log("init license");
			var cmd = sprintf("node %s/../tools/license_retriever %s %s %s %s",
				__dirname, m_options["license"]["app_key"], m_options["license"]["sku"], "license_key.json", m_options["license"]["iface"]);
			//console.log(cmd);
			child_process.exec(cmd);
		}
		
		callback(null);
	},
	function(callback) {
		console.log("init pstcore");
		
		pstcore.pstcore_add_log_callback((level, tag, msg) => {
			console.log(level, tag, msg);
		});

		var config_json = "";
		config_json += "{\n";
		config_json += "	\"plugin_paths\" : [\n";
		config_json += "		\"plugins/dummy_st.so\",\n";
		config_json += "		\"plugins/pvf_loader_st.so\",\n";
		config_json += "		\"plugins/psf_loader_st.so\",\n";
		config_json += "		\"plugins/libde265_decoder_st.so\",\n";
		config_json += "		\"plugins/recorder_st.so\",\n";
		config_json += "        \"plugins/tc_capture_st.so\",\n";
		config_json += "        \"plugins/nc_capture_st.so\",\n";
		config_json += "        \"plugins/dup_st.so\",\n";
		config_json += "        \"plugins/dealer_st.so\",\n";
		config_json += "        \"plugins/suspender_st.so\",\n";
		config_json += "        \"plugins/sampler_st.so\",\n";
		config_json += "        \"plugins/redis_st.so\",\n";
		if(process.platform === 'darwin') {
			config_json += "		\"plugins/vt_decoder_st.so\",\n";
			config_json += "        \"plugins/oal_capture_st.so\",\n";
			config_json += "        \"plugins/oal_player_st.so\",\n";
			config_json += "        \"plugins/opus_encoder_st.so\",\n";
			config_json += "        \"plugins/opus_decoder_st.so\",\n";
		}else if(process.platform === 'linux') {
			config_json += "        \"plugins/pcuda_remapper_st.so\",\n";
			config_json += "        \"plugins/v4l2_capture_st.so\",\n";
			config_json += "        \"plugins/mjpeg_tegra_decoder_st.so\",\n";
			config_json += "        \"plugins/v4l2_tegra_encoder_st.so\",\n";
			config_json += "        \"plugins/mjpeg_encoder_st.so\",\n";
			config_json += "        \"plugins/mjpeg_decoder_st.so\",\n";
			config_json += "        \"plugins/mux_st.so\",\n";
			//config_json += "        \"plugins/demux_st.so\",\n";
			config_json += "        \"plugins/oal_capture_st.so\",\n";
			config_json += "        \"plugins/oal_player_st.so\",\n";
			config_json += "        \"plugins/opus_encoder_st.so\",\n";
			config_json += "        \"plugins/opus_decoder_st.so\",\n";
			//config_json += "        \"plugins/icm20948_st.so\",\n";
			config_json += "        \"plugins/pantilt_st.so\",\n";
			//config_json += "        \"plugins/amimon_tx_st.so\",\n";
			//config_json += "        \"plugins/amimon_rx_st.so\",\n";
			config_json += "        \"plugins/ptpvf_generator_st.so\",\n";
		}else if(process.platform === 'win32') {
			config_json += "        \"plugins/pcuda_remapper_st.so\",\n";
			config_json += "        \"plugins/mjpeg_decoder_st.so\",\n";
			config_json += "        \"plugins/mux_st.so\",\n";
			config_json += "        \"plugins/demux_st.so\",\n";
		}
		config_json += "		\"plugins/pgl_renderer_st.so\",\n";
		config_json += "		\"plugins/pgl_remapper_st.so\",\n";
		config_json += "		\"plugins/pgl_calibrator_st.so\",\n";
		config_json += "		\"plugins/pipe_st.so\"\n";
		config_json += "	]\n";
		config_json += "}\n";
		pstcore.pstcore_init(JSON.stringify(JSON.parse(config_json)));
		
		setInterval(() => {
			pstcore.pstcore_poll_events();
		}, 33);
		
		if(m_calibrate){
			var [size_, device] = m_calibrate.split(":");
			var [size, framerate] = size_.split("@");
			var framerate_str = framerate ? " -r " + framerate : "";
			var def = "pipe name=capture t=I420 s=" + size + " ! pgl_calibrator w=1024 h=512";
			//var def = "tc_capture name=capture debayer=1 expo=20000 gain=1000 binning=2 ! pgl_calibrator w=1024 h=512";
			pstcore.pstcore_build_pstreamer(def, (pst) => {
				if(process.platform==='darwin'){
					var pipe_def = "/usr/local/bin/ffmpeg -f avfoundation -s @OWIDTH@x@OHEIGHT@" + framerate_str + " -i \""
									 + device + "\" -f rawvideo -pix_fmt yuv420p -";
					pstcore.pstcore_set_param(pst, "capture", "def", pipe_def);
	
					var meta = "<meta maptype=\"FISH\" lens_params=\"file://lens_params.json\" />";
					pstcore.pstcore_set_param(pst, "capture", "meta", meta);
				}
				else if(process.platform==='linux'){
					var pipe_def = "ffmpeg -f video4linux2 -s @OWIDTH@x@OHEIGHT@" + framerate_str + " -i \""
									 + device + "\" -f rawvideo -pix_fmt yuv420p -";
					pstcore.pstcore_set_param(pst, "capture", "def", pipe_def);
	
					var meta = "<meta maptype=\"FISH\" lens_params=\"file://lens_params.json\" />";
					pstcore.pstcore_set_param(pst, "capture", "meta", meta);
				}
				else if(process.platform==='win32'){
					var pipe_def = "ffmpeg -f dshow -s @OWIDTH@x@OHEIGHT@" + framerate_str + " -i video=\""
									 + device + "\" -f rawvideo -pix_fmt yuv420p -";
					pstcore.pstcore_set_param(pst, "capture", "def", pipe_def);
	
					var meta = "<meta maptype=\"FISH\" lens_params=\"file://lens_params.json\" />";
					pstcore.pstcore_set_param(pst, "capture", "meta", meta);
				}
				pstcore.pstcore_start_pstreamer(pst);
				//don't call callback(null);
			});
		}else if(m_calibrate_hq){
			var def = "nc_capture name=capture debayer=1 expo=20000 gain=1000 binning=0 ! pgl_calibrator w=1024 h=512";
			pstcore.pstcore_build_pstreamer(def, (pst) => {
				var meta = "<meta maptype=\"FISH\" lens_params=\"file://lens_params.json\" />";
				pstcore.pstcore_set_param(pst, "capture", "meta", meta);
	
				pstcore.pstcore_start_pstreamer(pst);
				//don't call callback(null);
			});
		}else{
			callback(null);
		}
	},
	function(callback) { // gc
		console.log("gc");
		if(process.platform === 'win32') {
		}else if(process.platform === 'darwin') {
		}else if(process.platform === 'linux') {
		}
		
		//check memory usage
		if(m_options["memory_usage_limit"]){
			setInterval(() => {
				const totalMem = os.totalmem();
				const freeMem = os.freemem();
				const usedMem = totalMem - freeMem;
				const usedMemPercentage = (usedMem / totalMem) * 100;
			
				console.log(`Memory Usage: ${usedMem} / ${totalMem} B ${usedMemPercentage.toFixed(2)}%`);
			
				const memory_usage_limit = (m_options["memory_usage_limit"] || 95);
				if (usedMemPercentage > memory_usage_limit) {
					console.log(`Memory usage exceeds ${memory_usage_limit}%. Killing the process...`);
					process.exit(1); // Exit the process with an error code
				}
			}, 10000);
		}
		callback(null);
	},
	function(callback) {
		// plugin host
		var m_view_quaternion = [0, 0, 0, 1.0];
		// cmd handling
		function command_handler(value, args) {
			let res = null;
			var split = value.split(' ');
			var domain = split[0].split('.');
			if (domain.length != 1 && domain[0] != "pserver") {
				// delegate to plugin
				for (var i = 0; i < m_plugins.length; i++) {
					if (m_plugins[i].name && m_plugins[i].name == domain[0]) {
						if (m_plugins[i].command_handler) {
							split[0] = split[0].substring(split[0].indexOf('.') + 1);
							res = m_plugins[i].command_handler(split.join(' '), args);
							break;
						}
					}
				}
				return res;
			}
			if (split[0] == "ping") {
				//
			} else if (split[0] == "snap") {
				var id = conn.frame_info.snapper_uuid;
				if (id) {
					var dirname = moment().format('YYYYMMDD_HHmmss');
					var filepath = (m_options['record_path'] || 'Videos') + '/' + dirname;
					var cmd = CAPTURE_DOMAIN + "set_vstream_param";
					cmd += " -p base_path=" + filepath;
					cmd += " -p mode=RECORD";
					cmd += " -u " + id;
					m_plugin_host.send_command(cmd, args);
					console.log("snap");
				}
			} else if (split[0] == "start_record") {
				if (conn.frame_info.is_recording)
					return;
				var id = conn.frame_info.recorder_uuid;
				if (id) {
					var dirname = moment().format('YYYYMMDD_HHmmss');
					var filepath = (m_options['record_path'] || 'Videos') + '/' + dirname;
					var cmd = CAPTURE_DOMAIN + "set_vstream_param";
					cmd += " -p base_path=" + filepath;
					cmd += " -p mode=RECORD";
					cmd += " -u " + id;
					res = m_plugin_host.send_command(cmd, args);
					conn.frame_info.is_recording = true;
					console.log("start record");
				}
			} else if (split[0] == "stop_record") {
				var id = conn.frame_info.recorder_uuid;
				if (id) {
					var cmd = CAPTURE_DOMAIN + "set_vstream_param";
					cmd += " -p mode=IDLE";
					cmd += " -u " + id;
					res = m_plugin_host.send_command(cmd, args);
					conn.frame_info.is_recording = false;
					console.log("stop record");
				}
			}
			return res;
		}
		setInterval(function() {
			if (m_cmd_list.length) {
				const params = m_cmd_list.shift();
				const res = command_handler(params.cmd, params.cmd_args);
				if(params.cmd_args && params.cmd_args.callback){
					params.cmd_args.callback.callback(res, params.cmd_args.callback_args);
				}
			}
		}, 20);
		m_plugin_host.send_command = function(cmd, cmd_args) {
			m_cmd_list.push({ cmd, cmd_args });
		};
		m_plugin_host.get_http = function() {
			return http;
		};
		m_plugin_host.get_https = function() {
			return https;
		};
		m_plugin_host.get_ws_server = function() {
			return ws_server;
		};
		m_plugin_host.get_wss_server = function() {
			return wss_server;
		};
		m_plugin_host.get_express_app = function() {
			return express_app;
		};
		m_plugin_host.fire_pst_started = function(pst) {
			for (var i = 0; i < m_plugins.length; i++) {
				if (m_plugins[i].pst_started) {
					m_plugins[i].pst_started(pstcore, pst);
				}
			}
		};
		m_plugin_host.fire_pst_stopped = function(pst) {
			for (var i = 0; i < m_plugins.length; i++) {
				if (m_plugins[i].pst_stopped) {
					m_plugins[i].pst_stopped(pstcore, pst);
				}
			}
		};
		/**
		 * build pstreamer
		 *
		 * @param {object} pstdef - { "base" : "string", "params" : {}, "replacements" : {} }
		 * @param {function} callback - callback( { pst, params, pstcore } )
		 * @returns {}
		 */
		m_plugin_host.build_pstreamer = function(pstdef, callback) {
			if(typeof pstdef === 'string'){
				pstdef = {
					"base" : pstdef
				};
			}
			if(!m_pstdefs[pstdef.base]){
				console.log("no stream definition : " + pstdef.base);
				pstdef.base = m_options["pstdefs"].default;
				console.log("use default stream definition : " + pstdef.base);
			}
			const name_list = [];
			{
				let name = pstdef.base;
				while(name && name_list.length < 10){
					if (m_pstdefs[name]) {
						name_list.push(name);
						name = m_pstdefs[name].base;
						if(!name){
							break;
						}
					}else{
						console.log("no stream definition : " + name);
						if(callback){
							callback( { pstcore, pst : null } );
						}
						return;
					}
				}
			}
			let def = "";
			let params = {};
			let replacements = {};
			let pviewer_config_ext = {};
			for (let i = 0; i < name_list.length; i++) {
				const name = name_list[i];
				if(m_pstdefs[name].def){
					if(m_pstdefs[name].def.startsWith("+ ")){
						def = def + m_pstdefs[name].def;
					}else if(m_pstdefs[name].def.endsWith(" +")){
						def = m_pstdefs[name].def + def;
					}else{
						def = m_pstdefs[name].def;
					}
					break;
				}
			}
			for (let i = name_list.length - 1; i >= 0; i--) {
				const name = name_list[i];
				params = Object.assign(params, m_pstdefs[name].params || {});
				replacements = Object.assign(replacements, m_pstdefs[name].replacements || {});
				pviewer_config_ext = Object.assign(pviewer_config_ext, m_pstdefs[name].pviewer_config_ext || {});
			}
			{//config.json is high priority
				params = Object.assign(params, m_options["pstdefs"].params || {});
				replacements = Object.assign(replacements, m_options["pstdefs"].replacements || {});
			}
			{//function param is high priority
				params = Object.assign(params, pstdef.params || {});
				replacements = Object.assign(replacements, pstdef.replacements || {});
			}

			function replace(str, replacements){
				if(!str){
					return str;
				}
				for(var key in replacements) {
					str = str.replace(new RegExp(key, "g"), replacements[key]);
				}
				return str;
			}
			def = replace(def, replacements);
			{
				const replaced = {};
				for(var key in params) {
					var value = params[key];
					if(value === null){
						continue;
					}
					replaced[key] = replace(value, replacements);
				}
				params = replaced;
			}
			pstcore.pstcore_build_pstreamer(def, (pst) => {
				if(callback){
					callback( { pstcore, pst, params, pviewer_config_ext } );
				}
			});
		}

		callback(null);
	},
	function(callback) {
		start_webserver(() => {
			callback(null);
		});
	},
	function(callback) {
		// load pstdefs
		if (m_options["pstdefs"]) {
			for (var k in m_options["pstdefs"]["paths"]) {
				const pstdef_path = m_options["pstdefs"]["paths"][k];
				console.log("loading... " + pstdef_path);
				if (fs.existsSync(pstdef_path)) {
					const lines = fs.readFileSync(pstdef_path, 'utf-8').replace(/\r/g, '').split('\n')
					for(var i=0;i<lines.length;i++){
						if(lines[i][0] == '#'){
							lines[i] = "";
						}
					}
					const json_str = lines.join("\n");
					const pstdef = jsonc.parse(json_str);
					let name = pstdef.name;
					if(!name){
						name = path.basename(pstdef_path, path.extname(pstdef_path));
					}
					m_pstdefs[name] = pstdef;
				}else{
					console.log("loading... not found " + pstdef_path);
				}
			}
		}
		callback(null);
	},
	function(callback) {
		// load plugin
		if (m_options["plugin_paths"]) {
			for (var k in m_options["plugin_paths"]) {
				var plugin_path = m_options["plugin_paths"][k];
				console.log("loading... " + plugin_path);
				var plugin_factory = require("./" + plugin_path);
				if(plugin_factory && plugin_factory.create_plugin){
					var plugin = plugin_factory.create_plugin(m_plugin_host);
					m_plugins.push(plugin);
				}else{
					console.log("fail... " + plugin_path);
				}
			}
			for (var i = 0; i < m_plugins.length; i++) {
				if (m_plugins[i].init_options) {
					m_plugins[i].init_options(m_options);
				}
			}
		}
		callback(null);
	},
], function(err, result) { });
