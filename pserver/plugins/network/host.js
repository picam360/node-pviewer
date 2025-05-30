
var async = require('async');
var fs = require("fs");
var sprintf = require('sprintf-js').sprintf;
var uuidgen = require('uuid/v4');
var url = require("url");
var EventEmitter = require('eventemitter3');
var xmlhttprequest = require('xmlhttprequest');
global.XMLHttpRequest = xmlhttprequest.XMLHttpRequest;

var pstcore = require('node-pstcore');

var mt_mod = require("./meeting.js");

var is_nodejs = (typeof process !== 'undefined' && process.versions && process.versions.node);
var rtp_mod;
var util;
if(is_nodejs){
	rtp_mod = require("./rtp.js");
	util = require('util');
}else{
	rtp_mod = {
		Rtp,
		PacketHeader,
	};
	util = {
		TextDecoder,
		TextEncoder,
	};
}

var UPSTREAM_DOMAIN = "upstream.";
var SERVER_DOMAIN = "";
var CAPTURE_DOMAIN = UPSTREAM_DOMAIN;
var DRIVER_DOMAIN = UPSTREAM_DOMAIN + UPSTREAM_DOMAIN;
var PT_STATUS = 100;
var PT_CMD = 101;
var PT_FILE = 102;
var PT_ENQUEUE = 110;
var PT_SET_PARAM = 111;
var PT_MT_ENQUEUE = 120;
var PT_MT_SET_PARAM = 121;


var SIGNALING_HOST = "peer.picam360.com";
// var SIGNALING_HOST = "test-peer-server.herokuapp.com";
var SIGNALING_PORT = 443;
var SIGNALING_SECURE = true;

var rtp_rx_conns = [];
var cmd2upstream_list = [];
var cmd_list = [];

var upstream_info = "";
var upstream_menu = "";
var upstream_quaternion = [0, 0, 0, 1.0];
var upstream_north = 0;

var options = {};
var PLUGIN_NAME = "host";
var m_plugin_host;
var m_mt_host;
var m_last_src = 0;

function init_data_stream(callback) {
    console.log("init data stream");

    var active_frame = null;
    var startTime = new Date();
    var num_of_frame = 0;
    var fps = 0;

    rtp_mod.send_error = function(rtp, err) {
        var name = "error";
        var value = err;
        var status = "<picam360:status name=\"" + name +
            "\" value=\"" + value + "\" />";
        var pack = rtp
            .build_packet(Buffer.from(status, 'ascii'), PT_STATUS);
        rtp.send_packet(pack);
    }

    rtp_mod.remove_conn = function(conn) {
        for (var i = rtp_rx_conns.length - 1; i >= 0; i--) {
            if (rtp_rx_conns[i] === conn) {
                console.log("connection closed : " +
                    rtp_rx_conns[i].attr.ip);
                rtp_rx_conns.splice(i, 1);

                clearInterval(conn.attr.timer);
                clearInterval(conn.attr.timer2);
                conn.close();
                if(conn.attr.pst){
                    m_plugin_host.fire_pst_stopped(conn.attr.pst);
                    pstcore.pstcore_destroy_pstreamer(conn.attr.pst);
                    conn.attr.pst = 0;
                }
                return;
            }
        }
    };
    
    rtp_mod.add_conn = function(conn) {
        var ip;
        if (conn.peerConnection) { // webrtc
            ip = " via webrtc";
        } else {
            ip = " via websocket";
        }
        
        conn.frame_info = {
            stream_uuid: uuidgen(),
            renderer_uuid: uuidgen(),
            snapper_uuid: uuidgen(),
            recorder_uuid: uuidgen(),
            stream_mode: "vid+mt",
            mode: options.frame_mode || "WINDOW",
            width: options.frame_width || 512,
            height: options.frame_height || 512,
            stream_def: options.stream_def || "default",
            fps: options.frame_fps || 5,
            bitrate: options.frame_bitrate,
        };

        conn.attr = {
            ip: ip,
            frame_queue: [],
            fps: 5,
            latency: 1.0,
            min_latency: 1.0,
            frame_num: 0,
            tmp_num: 0,
            tmp_latency: 0,
            tmp_time: 0,
            timeout: false,
            param_pendings: [],
        };
        var rtp = rtp_mod.Rtp(conn);
        conn.rtp = rtp;
        conn.rtp.src = ++m_last_src;

        if (rtp_rx_conns.length >= (options.n_clients_limit || 2)) { // exceed client
            console.log("exceeded_num_of_clients : " + ip);
            setTimeout(function() {
                rtp_mod.send_error(rtp, "exceeded_num_of_clients");
                setTimeout(function() {
                    conn.close();
                }, 1000);
            }, 1000);
            return;
        } else {
            console.log("connection opend : " + ip);
        }

        new Promise((resolve, reject) => {
            rtp.set_callback(function(packet) {
                conn.attr.timeout = new Date().getTime();
                if (packet.GetPayloadType() == PT_CMD) {
                    var cmd = new util.TextDecoder().decode(packet.GetPayload());
                    var split = cmd.split('\"');
                    var id = split[1];
                    var value = split[3].split(' ');
                    if (value[0] == "frame_mode") {
                        conn.frame_info.mode = value[1];
                        return;
                    } else if (value[0] == "frame_width") {
                        conn.frame_info.width = value[1];
                        return;
                    } else if (value[0] == "frame_height") {
                        conn.frame_info.height = value[1];
                        return;
                    } else if (value[0] == "frame_fps") {
                        conn.frame_info.fps = value[1];
                        return;
                    } else if (value[0] == "stream_mode") {
                        conn.frame_info.stream_mode = value[1];
                        return;
                    } else if (value[0] == "stream_def") {
                        conn.frame_info.stream_def = value[1];
                        return;
                    } else if (value[0] == "frame_bitrate") {
                        conn.frame_info.bitrate = value[1];
                        return;
                    } else if (value[0] == "ping") {
                        var status = "<picam360:status name=\"pong\" value=\"" +
                            value[1] +
                            " " +
                            new Date().getTime() +
                            "\" />";
                        var pack = rtp
                            .build_packet(Buffer.from(status, 'ascii'), PT_STATUS);
                        rtp.send_packet(pack);
                        if(!rtp.ping_cnt){
                            rtp.ping_cnt = 1;

                            var status = `<picam360:status name=\"src\" value=\"${rtp.src}\" />`;
                            var pack = rtp
                                .build_packet(Buffer.from(status, 'ascii'), PT_STATUS);
                            rtp.send_packet(pack);
                        }else{
                            rtp.ping_cnt++;
                        }
                        return;
                    } else if (value[0] == "set_timediff_ms") {
                        rtp.timediff_ms = parseFloat(value[1]);

                        resolve();
                    }
                }
            });
        }).then(() => {
            if(!m_mt_host){
                m_mt_host = mt_mod.MeetingHost(pstcore, options.meeting_enabled, options);
            }
            if((conn.frame_info.stream_mode == "mt" || conn.frame_info.stream_mode == "vid+mt")){
                m_mt_host.add_client(rtp);
            }

            if((conn.frame_info.stream_mode == "vid" || conn.frame_info.stream_mode == "vid+mt")){
                m_plugin_host.build_pstreamer(conn.frame_info.stream_def, (res) => {
                    conn.attr.pst = res.pst;
                    for(var key in res.params) {
                        var dotpos = key.lastIndexOf(".");
                        var name = key.substr(0, dotpos);
                        var param = key.substr(dotpos + 1);
                        var value = res.params[key];
                        if(!name || !param || !value){
                            continue;
                        }
                        pstcore.pstcore_set_param(res.pst, name, param, value);
                        console.log("stream params : ", name, param, value);
                        conn.attr.param_pendings.push([name, param, value]);
                    }
                    // pviewer_config_ext for client loading extra plugins
                    if(res.pviewer_config_ext) {
                        conn.attr.param_pendings.push(["network", "pviewer_config_ext", JSON.stringify(res.pviewer_config_ext)]);
                    }
        
                    pstcore.pstcore_set_dequeue_callback(res.pst, (data)=>{
                        try{
                            if(data == null){//eob
                                var pack = rtp.build_packet(Buffer.from("<eob/>", 'ascii'), PT_ENQUEUE);
                                rtp.send_packet(pack);
                            }else{
                                conn.attr.transmitbytes += data.length;
                                //console.log("dequeue " + data.length);
                                var MAX_PAYLOAD = conn.getMaxPayload() || 16*1024;//16k is webrtc max
                                var CHUNK_SIZE = MAX_PAYLOAD - rtp_mod.PacketHeaderLength;
                                for(var cur=0;cur<data.length;cur+=CHUNK_SIZE){
                                    var chunk = data.slice(cur, cur + CHUNK_SIZE);
                                    var pack = rtp.build_packet(chunk, PT_ENQUEUE);
                                    rtp.send_packet(pack);
                                }
                            }
                        }catch(err){
                            rtp_mod.remove_conn(conn);
                        }
                    });
            
                    pstcore.pstcore_add_set_param_done_callback(res.pst, (pst_name, param, value) => {
                        if(conn.attr.in_pt_set_param){//prevent loop back
                            return;
                        }
                        if(pst_name == "renderer" && param == "overlay"){
                            return;
                        }
                        if(value.length > 64*1024){//too long
                            console.log("send param too long", pst_name, param, value);
                            return;
                        }
                        conn.attr.param_pendings.push([pst_name, param, value]);
                        //console.log(pst_name, param, value);
                    });
                    m_plugin_host.fire_pst_started(res.pst);
                    pstcore.pstcore_start_pstreamer(res.pst);
                });
            }
    
            rtp_rx_conns.push(conn);

            conn.attr.timer = setInterval(function() {
                try{
                    var now = new Date().getTime();
                    if (now - conn.attr.timeout > 60000) {
                        console.log("timeout");
                        throw "TIMEOUT";
                    }
                    if(conn.attr.param_pendings.length > 0) {
                        var msg = JSON.stringify(conn.attr.param_pendings);
                        var pack = rtp.build_packet(Buffer.from(msg, 'ascii'), PT_SET_PARAM);
                        rtp.send_packet(pack);
                        conn.attr.param_pendings = [];
                    }
                }catch(err){
                    rtp_mod.remove_conn(conn);
                }
            }, 33);
            
            conn.attr.transmitbytes = 0;
            conn.attr.timer2 = setInterval(()=>{
                if(conn.attr.transmitbytes == 0){
                    return;
                }
                console.log(8*conn.attr.transmitbytes/1000);
                conn.attr.transmitbytes=0;
            },1000);
            
            rtp.set_callback(function(packet) {
                conn.attr.timeout = new Date().getTime();
                if (packet.GetPayloadType() == PT_ENQUEUE) {
                    console.log("PT_ENQUEUE from client");
                }else if (packet.GetPayloadType() == PT_SET_PARAM) { // set_param
                    var str = (new TextDecoder).decode(packet.GetPayload());
                    try{
                        var list = JSON.parse(str);
                        for(var ary of list){
                            if(ary[0] == "network"){
                                if(ary[1] == "get_file"){
                                    var [filename, key] = ary[2].split(' ');
                                    filerequest_handler(filename, key, conn);
                                }
                            }else if(conn.attr.pst){
                                conn.attr.in_pt_set_param = true;
                                pstcore.pstcore_set_param(conn.attr.pst, ary[0], ary[1], ary[2]);
                                conn.attr.in_pt_set_param = false;
                            }
                        }
                    }catch{
                        console.log("fail parse json", str);
                    }
                }else if (packet.GetPayloadType() == PT_CMD) {
                    var str = (new TextDecoder).decode(packet.GetPayload());
                    var split = str.split('\"');
                    var id = split[1];
                    var value = decodeURIComponent(split[3]);
                    m_plugin_host.send_command(value, (ret, ext) => {
                        var xml = "<picam360:command id=\"" + ext.id +
                            "\" value=\"" + encodeURIComponent(ret) + "\" />"
                        var pack = conn.rtp.build_packet(xml, PT_CMD);
                        ext.conn.rtp.send_packet(pack);
                    }, {conn, id});
                    if (options.debug >= 5) {
                        console.log("cmd got :" + cmd);
                    }
                }else{
                    m_mt_host.handle_packet(packet, rtp);
                }
            });
        });
    }
    callback(null);
}
function start_websocket(callback) {
    // websocket
    for(var server of [m_plugin_host.get_ws_server(), m_plugin_host.get_wss_server()]){
        if(!server){
            continue;
        }
        server.on("connection", (dc, request) => {
            const parsed = url.parse(request.url, true);
            if(parsed.pathname != "/"){
                return;
            }
            class DataChannel extends EventEmitter {
                constructor() {
                    super();
                    var self = this;
                    dc.on('message', function(data) {
                        self.emit('data', data);
                    });
                    dc.on('error', function(event) {
                        console.log(event);
                    });
                    dc.on('close', function(event) {
                        self.close();
                    });
                }
                getMaxPayload() {
                    return dc._maxPayload;
                }
                send(data) {
                    if (dc.readyState == 3) {
                        throw "already closed";
                    }
                    if (dc.readyState != 1) {
                        console.log('something wrong : ' + dc.readyState);
                        return;
                    }
                    if (!Array.isArray(data)) {
                        data = [data];
                    }
                    try {
                        for (var i = 0; i < data.length; i++) {
                            dc.send(data[i]);
                        }
                    } catch (e) {
                        console.log('error on dc.send');
                        this.close();
                        throw e;
                    }
                }
                close() {
                    dc.close();
                    console.log('WebSocket closed');
                    rtp_mod.remove_conn(this);
                    this.emit('closed');
                }
            }
            var conn = new DataChannel();
            rtp_mod.add_conn(conn);
        });
    }
    callback(null);
}
function start_wrtc(callback) {
    // wrtc
    if (options["wrtc"] && options["wrtc"].enabled) {
        var P2P_API_KEY = "v8df88o1y4zbmx6r";
        global.Blob = "blob";
        global.File = "file";
        global.WebSocket = require("ws");
        global.window = require("wrtc");
        global.window.evt_listener = [];
        global.window.postMessage = function(message, origin) {
            console.log(message);
            var event = {
                source: global.window,
                data: message,
            };
            if (global.window.evt_listener["message"]) {
                global.window.evt_listener["message"].forEach(function(
                    callback) {
                    callback(event);
                });
            }
        };
        global.window.addEventListener = function(name, callback, bln) {
            if (!global.window.evt_listener[name]) {
                global.window.evt_listener[name] = [];
            }
            global.window.evt_listener[name].push(callback);
        }
        var key = options["wrtc"].key || uuidgen();
        console.log("\n\n\n");
        console.log("webrtc key : " + key);
        console.log("https://picam360.github.io/pviewer/?wrtc-key=" + key);
        console.log("\n\n\n");
        var sig_options = {
            host: SIGNALING_HOST,
            port: SIGNALING_PORT,
            secure: SIGNALING_SECURE,
            key: P2P_API_KEY,
            local_peer_id: key,
            iceServers: [{
                "urls": "stun:stun.l.google.com:19302"
            },
            {
                "urls": "stun:stun1.l.google.com:19302"
            },
            {
                "urls": "stun:stun2.l.google.com:19302"
            },
            ],
            debug: options.debug || 0,
        };
        if (options.turn_server) {
            options.iceServers.push({
                urls: 'turn:turn.picam360.com:3478',
                username: "picam360",
                credential: "picam360"
            });
        }
        var Signaling = require("./signaling.js").Signaling;
        var connect = function() {
            var pc_map = {};
            var sig = new Signaling(sig_options);
            sig.connect(function() {
                sig.start_ping();
            });
            sig.onrequestoffer = function(request) {
                var pc = new global.window.RTCPeerConnection({
                    sdpSemantics: 'unified-plan',
                    iceServers: sig_options.iceServers,
                });
                pc_map[request.src] = pc;

                var dc = pc.createDataChannel('data');
                dc.onopen = function() {
                    console.log('Data channel connection success');
                    class DataChannel extends EventEmitter {
                        constructor() {
                            super();
                            var self = this;
                            this.peerConnection = pc;
                            dc.addEventListener('message', function(data) {
                                self.emit('data', Buffer.from(new Uint8Array(data.data)));
                            });
                        }
                        getMaxPayload() {
                            return dc.maxRetransmits;
                        }
                        send(data) {
                            if (dc.readyState == 'closed') {
                                throw "already closed";
                            }
                            if (dc.readyState != 'open') {
                                console.log('something wrong : ' + dc.readyState);
                                return;
                            }
                            if (!Array.isArray(data)) {
                                data = [data];
                            }
                            try {
                                for (var i = 0; i < data.length; i++) {
                                    dc.send(data[i]);
                                }
                            } catch (e) {
                                console.log('error on dc.send');
                                this.close();
                                throw e;
                            }
                        }
                        close() {
                            dc.close();
                            pc.close();
                            console.log('Data channel closed');
                            this.emit('closed');
                        }
                    }
                    dc.DataChannel = new DataChannel();
                    rtp_mod.add_conn(dc.DataChannel);
                }

                pc.createOffer().then(function(sdp) {
                    console.log('setLocalDescription');
                    pc.setLocalDescription(sdp);
                    sig.offer(request.src, sdp);
                }).catch(function(err) {
                    console.log('failed offering:' +
                        err);
                });
                pc.onicecandidate = function(event) {
                    if (event.candidate) {
                        sig.candidate(request.src, event.candidate);
                    } else {
                        // All ICE candidates have been sent
                    }
                };
                pc.onconnectionstatechange = function(event) {
                    console.log('peer connection state changed : ' + pc.connectionState);
                    switch (pc.connectionState) {
                        case "connected":
                            // The connection has become fully connected
                            break;
                        case "disconnected":
                        case "failed":
                        case "closed":
                            console.log('peer connection closed');
                            pc.close();
                            dc.close();
                            if (dc.DataChannel) {
                                rtp_mod.remove_conn(dc.DataChannel);
                            }
                            break;
                    }
                }
            };
            sig.onanswer = function(answer) {
                if (pc_map[answer.src]) {
                    pc_map[answer.src].setRemoteDescription(answer.payload.sdp);
                }
            };
            sig.oncandidate = function(candidate) {
                if (pc_map[candidate.src] && candidate.payload.ice.candidate) {
                    pc_map[candidate.src].addIceCandidate(candidate.payload.ice);
                }
            };
            sig.onclose = function(e) {
                // console.log('Socket closed : dump error object below');
                // console.dir(e);
                setTimeout(() => {
                    console.log('Try to reconnect');
                    connect();
                }, 1000);
            };
        };
        connect();
    }
    callback(null);
}

function send_file(filename, key, conn, data) {
    var MAX_PAYLOAD = conn.getMaxPayload() || 16*1024;//16k is webrtc max
    var CHUNK_SIZE = MAX_PAYLOAD - rtp_mod.PacketHeaderLength - 256;
    var length;
    for (var i = 0, seq = 0; i < data.length; i += length, seq++) {
        var eof;
        if (i + CHUNK_SIZE >= data.length) {
            eof = true;
            length = data.length - i;
        } else {
            eof = false;
            length = CHUNK_SIZE;
        }
        var header_str = sprintf("<picam360:file name=\"%s\" key=\"%s\" status=\"200\" seq=\"%d\" eof=\"%s\" />", filename, key, seq, eof
            .toString());
        var header = Buffer.from(header_str, 'ascii');
        var len = 2 + header.length + length;
        var buffer = Buffer.alloc(len);
        buffer.writeUInt16BE(header.length, 0);
        header.copy(buffer, 2);
        data.copy(buffer, 2 + header.length, i, i + length);
        var pack = conn.rtp.build_packet(buffer, PT_FILE);
        conn.rtp.send_packet(pack);
    }
}

function filerequest_handler(filename, key, conn) {
    fs.readFile("../pviewer/" + filename, function(err, data) {
        if (err) {
            var header_str = "<picam360:file name=\"" + filename +
                "\" key=\"" + key + "\" status=\"404\" />";
            data = Buffer.alloc(0);
            console.log("unknown :" + filename + ":" + key);
            var header = Buffer.from(header_str, 'ascii');
            var len = 2 + header.length;
            var buffer = Buffer.alloc(len);
            buffer.writeUInt16BE(header.length, 0);
            header.copy(buffer, 2);
//					var pack = rtp.build_packet(buffer, PT_FILE);
//					rtp.send_packet(pack);
        } else {
            send_file(filename, key, conn, data);
        }
    });
}

var self = {
    create_plugin: function (plugin_host) {
        m_plugin_host = plugin_host;
        console.log("create host plugin");
        var plugin = {
            name: PLUGIN_NAME,
            init_options: function (_options) {
                options = _options;
                init_data_stream(() => {
                    start_websocket(() => {
                        start_wrtc(() => {
                            console.log("host initiation done!");
                        });
                    });
                });
            },
            pst_started: function (pstcore, pst) {
            },
            pst_stopped: function (pstcore, pst) {
            },
            command_handler: function (cmd, conn) {
            },
        };
        return plugin;
    }
};
module.exports = self;