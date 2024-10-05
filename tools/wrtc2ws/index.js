
var async = require('async');
var fs = require("fs");
var sprintf = require('sprintf-js').sprintf;
var uuidgen = require('uuid/v4');
var EventEmitter = require('eventemitter3');
var xmlhttprequest = require('xmlhttprequest');
global.XMLHttpRequest = xmlhttprequest.XMLHttpRequest;

var SIGNALING_HOST = "peer.picam360.com";
// var SIGNALING_HOST = "test-peer-server.herokuapp.com";
var SIGNALING_PORT = 443;
var SIGNALING_SECURE = true;
var P2P_API_KEY = "v8df88o1y4zbmx6r";

var m_options = {};

var Signaling = require("./signaling.js").Signaling;

function start_webserver(callback) { // start up websocket server
    console.log("websocket server starting up");
    express_app = express();
	express_app.use(cors());
	express_app.use(express.json());
    http = require('http').Server(express_app);

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

    express_app.use(express.static('../pviewer')); // this need be set
	var http_port = 9001;
	if(m_options['http_port']){
		http_port = m_options['http_port'];
	}
    http.listen(http_port, function() {
        console.log('listening http on *:' + http_port);
    });

	var https_port = 9002;
	if(m_options['https_port']){
		https_port = m_options['https_port'];
	}
    https.listen(https_port, function() {
        console.log('listening https on *:' + https_port);
    });
    callback(null);
}

function start_websocket(callback) {
    // websocket
    var WebSocket = require("ws");
    for(var server of [m_plugin_host.get_http(), m_plugin_host.get_https()]){
        if(!server){
            continue;
        }
        var ws = new WebSocket.Server({ server });
    
        ws.on("connection", dc => {
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

var connect = function() {
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
        debug: m_options.debug || 0,
    };
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

function main() {
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
    var key = m_options.key || uuidgen();
    console.log("\n\n\n");
    console.log("webrtc key : " + key);
    console.log("https://picam360.github.io/pviewer/?wrtc-key=" + key);
    console.log("\n\n\n");
    if (m_options.turn_server) {
        m_options.iceServers.push({
            urls: 'turn:turn.picam360.com:3478',
            username: "picam360",
            credential: "picam360"
        });
    }
    connect();
}

if (require.main === module) {
    main();
}
