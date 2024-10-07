
const xmlhttprequest = require('xmlhttprequest');
global.XMLHttpRequest = xmlhttprequest.XMLHttpRequest;
const WebSocket = require("ws");
global.WebSocket = WebSocket;
const wrtc = require("wrtc");
global.window = wrtc;

global.Blob = "blob";
global.File = "file";
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

var Signaling = require("./signaling.js").Signaling;

var SIGNALING_HOST = "peer.picam360.com";
// var SIGNALING_HOST = "test-peer-server.herokuapp.com";
var SIGNALING_PORT = 443;
var SIGNALING_SECURE = true;
var P2P_API_KEY = "v8df88o1y4zbmx6r";

class DataChannel2WebSocket {
    constructor(dataChannel) {
        this.dataChannel = dataChannel;
        this.readyState = this.getReadyState(); // Convert RTCDataChannel readyState to WebSocket-compatible readyState

        // Event handlers
        this.onopen = null;
        this.onmessage = null;
        this.onclose = null;
        this.onerror = null;

        // Map RTCDataChannel events to WebSocket-compatible ones
        this.dataChannel.onopen = () => {
            this.readyState = this.getReadyState();
            if (this.onopen) {
                this.onopen();
            }
        };

        this.dataChannel.onmessage = (event) => {
            if (this.onmessage) {
                this.onmessage(event);
            }
        };

        this.dataChannel.onclose = () => {
            this.readyState = this.getReadyState();
            if (this.onclose) {
                this.onclose();
            }
        };

        this.dataChannel.onerror = (error) => {
            if (this.onerror) {
                this.onerror(error);
            }
        };
    }

    // Send data just like WebSocket's send method
    send(data) {
        if (this.dataChannel.readyState === "open") {
            this.dataChannel.send(data);
        } else {
            throw new Error("DataChannel is not open.");
        }
    }

    // Close the data channel similar to WebSocket's close method
    close() {
        this.dataChannel.close();
    }

    // Convert RTCDataChannel readyState to WebSocket-like readyState
    getReadyState() {
        const stateMap = {
            'connecting': 0, // WebSocket.CONNECTING
            'open': 1,       // WebSocket.OPEN
            'closing': 2,    // WebSocket.CLOSING
            'closed': 3      // WebSocket.CLOSED
        };
        return stateMap[this.dataChannel.readyState];
    }

    // AddEventListener wrapper
    addEventListener(event, listener) {
        this.dataChannel.addEventListener(event, listener);
    }

    removeEventListener(event, listener) {
        this.dataChannel.removeEventListener(event, listener);
    }
}

function start_wrtc_client(p2p_uuid, callback, err_callback, options) {
    options = options || {};

    var sig_options = {
        host: SIGNALING_HOST,
        port: SIGNALING_PORT,
        secure: SIGNALING_SECURE,
        key: P2P_API_KEY,
        iceServers : [
                            {"urls": "stun:stun.l.google.com:19302"},
                        {"urls": "stun:stun1.l.google.com:19302"},
                        {"urls": "stun:stun2.l.google.com:19302"},
                    ],
        debug: options['debug'] || 0,
    };
    if (options['turn-server']) {
        sig_options.iceServers.push({
            urls: 'turn:turn.picam360.com:3478',
            username: "picam360",
            credential: "picam360"
        });
    }
    var sig = new Signaling(sig_options);
    sig.connect(function() {
        var pc = new global.window.RTCPeerConnection({
            sdpSemantics: 'unified-plan',
            iceServers: options.iceServers
        });

        sig.onoffer = function(offer) {
            pc.setRemoteDescription(offer.payload.sdp).then(function() {
                return pc.createAnswer();
            }).then(function(sdp) {
                console.log('Created answer.');
                
                pc.setLocalDescription(sdp);
                sig.answer(offer.src, sdp);
            }).catch(function(err) {
                console.log('Failed answering:' + err);
            });
            pc.onicecandidate = function(event) {
                if (event.candidate) {
                    sig.candidate(offer.src, event.candidate);
                } else {
                    // All ICE candidates have been sent
                }
            };
            pc.ondatachannel = function(ev) {
                console.log('Data channel is created!');

                const dc = ev.channel;
                callback(dc);
            };
            pc.onerror = function(err) {
                if (err.type == "peer-unavailable") {
                    m_plugin_host.set_info("error : Could not connect " +
                        p2p_uuid);
                    err_callback();
                }
            };
        };
        sig.oncandidate = function(candidate) {
            pc.addIceCandidate(candidate.payload.ice);
        };
        sig.request_offer(p2p_uuid);
    });
}

function start_wrtc_host(p2p_uuid, callback, err_callback, options) {
    options = options || {};

    var P2P_API_KEY = "v8df88o1y4zbmx6r";
    console.log("\n\n\n");
    console.log("webrtc key : " + p2p_uuid);
    console.log("https://picam360.github.io/pviewer/?wrtc-key=" + p2p_uuid);
    console.log("\n\n\n");
    var sig_options = {
        host: SIGNALING_HOST,
        port: SIGNALING_PORT,
        secure: SIGNALING_SECURE,
        key: P2P_API_KEY,
        local_peer_id: p2p_uuid,
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

            const dc = pc.createDataChannel('data');
            callback(dc);

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

const bind_wrtc_and_ws = (dc, ws, options) => {
    options = options || {};

    // WebSocket -> DataChannel
    ws.binaryType = 'arraybuffer';// blob or arraybuffer
    ws.onopen = (event) => {
        console.log("ws opened.");
    };
    ws.onmessage = (event) => {
        if(options.debug){
            console.log("ws2dc send", event.data.length);
        }
        dc.send(event.data);
    };
    ws.onclose = (event) => {
        console.log("ws closed.");
        dc.close();
    };

    // DataChannel -> WebSocket
    dc.onopen = (event) => {
        console.log("dc opened.");
    };
    dc.onmessage = (event) => {
        if(options.debug){
            console.log("dc2ws send", event.data.length);
        }
        ws.send(event.data);
    };
    dc.onclose = (event) => {
        console.log("dc closed.");
        ws.close();
    };
};

const bind_wrtc_and_socket = (dc, socket, options) => {
    options = options || {};

    // Socket -> DataChannel
    socket.on('data', (data) => {
        if(options.debug){
            console.log("socket2dc send", data.length, dc.readyState);
        }
        if (dc.readyState === 'open') {
            dc.send(data);
        }else{
            dc.pendings = dc.pendings || [];
            dc.pendings.push(data);
        }
    });

    socket.on('end', () => {
        console.log("socket closed.");
        dc.close();
    });

    socket.on('error', (err) => {
        console.log("socket error.", err);
        dc.close();
    });

    // DataChannel -> Socket
    dc.onopen = (event) => {
        console.log("dc opened.");
        if(dc.pendings){
            for(const data of dc.pendings){
                console.log("socket2dc send pending", data.length, dc.readyState);
                dc.send(data);
            }
            dc.pendings = undefined;
        }
    };
    dc.onmessage = (event) => {
        const data = Buffer.from(event.data);
        if(options.debug){
            console.log("dc2socket send", data.length);
        }
        socket.write(data);
    };
    dc.onclose = (event) => {
        console.log("dc closed.");
        socket.end();
    };
};

module.exports = {
    DataChannel2WebSocket,
    start_wrtc_client,
    start_wrtc_host,
    bind_wrtc_and_ws,
    bind_wrtc_and_socket,
};