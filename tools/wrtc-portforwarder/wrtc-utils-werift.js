
//"tslib": "^2.8.1",
//"werift-webrtc": "0.0.6",

const { RTCPeerConnection, RTCIceCandidate } = require("werift-webrtc");
const WebSocket = require("ws");
const XMLHttpRequest = require("xmlhttprequest").XMLHttpRequest;
const Signaling = require("./signaling.js").Signaling;

global.WebSocket = WebSocket;
global.XMLHttpRequest = XMLHttpRequest;

const SIGNALING_HOST = "wrtc-pf.picam360.com";
const SIGNALING_PORT = 3000;
const SIGNALING_SECURE = false;
const P2P_API_KEY = "v8df88o1y4zbmx6r";

// --- クライアント用 WebRTC 接続 ---
function randomString(length = 8) {
    const chars = 'ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789';
    return Array.from({ length }, () => chars[Math.floor(Math.random() * chars.length)]).join('');
}
function start_wrtc_client(p2p_uuid, callback, err_callback, options = {}) {
    const sig_options = {
        host: SIGNALING_HOST,
        port: SIGNALING_PORT,
        secure: SIGNALING_SECURE,
        key: P2P_API_KEY,
        local_peer_id: randomString(),
        iceServers: [
            { urls: "stun:stun.l.google.com:19302" },
            { urls: "stun:stun1.l.google.com:19302" },
            { urls: "stun:stun2.l.google.com:19302" },
        ],
        debug: options.debug || 0,
    };

    const sig = new Signaling(sig_options);

    sig.connect(() => {
        const pc = new RTCPeerConnection({ iceServers: sig_options.iceServers });

        sig.onoffer = async (offer) => {

            pc.onicecandidate = ({ candidate }) => {
                if (candidate) {
                    sig.candidate(offer.src, candidate);
                }
            };

            pc.ondatachannel = (event) => {
                const dc = event.channel;
                callback(dc);
            };

            await pc.setRemoteDescription(offer.payload); // 修正ポイント
            const answer = await pc.createAnswer();
            await pc.setLocalDescription(answer);
            while (pc.iceGatheringState !== "complete") {
                await new Promise(resolve => setTimeout(resolve, 100));
            }
            sig.answer(offer.src, pc.localDescription.sdp);
        };

        sig.oncandidate = (candidate) => {
            if (candidate.payload.ice?.candidate) {
                pc.addIceCandidate(new RTCIceCandidate(candidate.payload.ice));
            }
        };

        sig.request_offer(p2p_uuid);
    });
}

// --- ホスト用 WebRTC 接続 ---
function start_wrtc_host(p2p_uuid, callback, err_callback, options = {}) {
    const sig_options = {
        host: SIGNALING_HOST,
        port: SIGNALING_PORT,
        secure: SIGNALING_SECURE,
        key: P2P_API_KEY,
        local_peer_id: p2p_uuid,
        iceServers: [
            { urls: "stun:stun.l.google.com:19302" },
            { urls: "stun:stun1.l.google.com:19302" },
            { urls: "stun:stun2.l.google.com:19302" },
        ],
        debug: options.debug || 0,
    };

    const connect = () => {
        const pc_map = {};
        const sig = new Signaling(sig_options);

        sig.connect(() => sig.start_ping());

        sig.onrequestoffer = async (request) => {
            const pc = new RTCPeerConnection({ iceServers: sig_options.iceServers });
            pc_map[request.src] = pc;

            pc.onicecandidate = ({ candidate }) => {
                if (candidate) {
                    sig.candidate(request.src, candidate);
                }
            };

            pc.onconnectionstatechange = () => {
                if (["disconnected", "failed", "closed"].includes(pc.connectionState)) {
                    dc.close();
                    pc.close();
                }
            };

            const dc = pc.createDataChannel("data");
            callback(dc);

            const offer = await pc.createOffer();
            await pc.setLocalDescription(offer);
            // while (pc.iceGatheringState !== "complete") {
            //     await new Promise(resolve => setTimeout(resolve, 100));
            // }
            sig.offer(request.src, pc.localDescription.sdp);
        };

        sig.onanswer = async (answer) => {
            const pc = pc_map[answer.src];
            if (pc) {
                await pc.setRemoteDescription(answer.payload); // 修正ポイント
            }
        };

        sig.oncandidate = (candidate) => {
            const pc = pc_map[candidate.src];
            if (pc && candidate.payload.ice?.candidate) {
                pc.addIceCandidate(new RTCIceCandidate(candidate.payload.ice));
            }
        };

        sig.onclose = () => {
            setTimeout(() => {
                console.log("Reconnecting signaling...");
                connect();
            }, 1000);
        };
    };

    connect();
}

// --- WebSocket ←→ DataChannel バインド ---
const bind_wrtc_and_ws = (dc, ws, options = {}) => {
    ws.binaryType = 'arraybuffer';

    ws.onopen = () => console.log("WebSocket opened");
    ws.onmessage = (event) => {
        if (options.debug) console.log("ws→dc", event.data.length);
        dc.send(event.data);
    };
    ws.onclose = () => {
        console.log("WebSocket closed");
        dc.close();
    };

    dc.onopen = () => console.log("DataChannel opened");
    dc.onmessage = (event) => {
        if (options.debug) console.log("dc→ws", event.data.length);
        ws.send(event.data);
    };
    dc.onclose = () => {
        console.log("DataChannel closed");
        ws.close();
    };
};

// --- TCP Socket ←→ DataChannel バインド ---
const bind_wrtc_and_socket = (dc, socket, options = {}) => {
    socket.on("data", (data) => {
        if (options.debug) console.log("socket→dc", data.length);
        if (dc.readyState === "open") {
            dc.send(data);
        } else {
            dc.pendings = dc.pendings || [];
            dc.pendings.push(data);
        }
    });

    socket.on("end", () => {
        console.log("Socket closed");
        dc.close();
    });

    socket.on("error", (err) => {
        console.log("Socket error", err);
        dc.close();
    });

    dc.onopen = () => {
        console.log("DataChannel opened");
        if (dc.pendings) {
            for (const data of dc.pendings) {
                dc.send(data);
            }
            delete dc.pendings;
        }
    };

    dc.onmessage = (event) => {
        const data = Buffer.from(event.data);
        if (options.debug) console.log("dc→socket", data.length);
        socket.write(data);
    };

    dc.onclose = () => {
        console.log("DataChannel closed");
        socket.end();
    };
};

module.exports = {
    start_wrtc_client,
    start_wrtc_host,
    bind_wrtc_and_ws,
    bind_wrtc_and_socket,
};
