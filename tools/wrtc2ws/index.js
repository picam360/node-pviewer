
const async = require('async');
const fs = require("fs");
const sprintf = require('sprintf-js').sprintf;
const uuidgen = require('uuid/v4');
const EventEmitter = require('eventemitter3');
const express = require('express');
const WebSocket = require("ws");
const http = require('http');
const cors = require('cors');
const jsonc = require('jsonc-parser');
const yargs = require('yargs');
const wrtc2ws = require("./wrtc2ws.js");


const m_options = {
    
};

function main() {
    const argv = yargs
        .option('bind', {
            alias: 'b',
            type: 'string',
            array: true,
            description: 'webrtc host',
        })
        .help()
        .alias('help', 'h')
        .argv;

    const isValidPort = (value) => {
        const port = Number(value);
        return Number.isInteger(port) && port >= 0 && port <= 65535;
    };
    const bind_wrtc_and_ws = (wrtc_ws, ws) => {
        ws.binaryType = 'arraybuffer';// blob or arraybuffer
        ws.onopen = (event) => {
            console.log("ws opened.");
        };
        ws.onmessage = (event) => {
            wrtc_ws.send(event.data);
        };
        ws.onclose = (event) => {
            console.log("ws closed.");
        };

        wrtc_ws.onopen = (event) => {
            console.log("wrtc_ws opened.");
        };
        wrtc_ws.onmessage = (event) => {
            ws.send(event.data);
        };
        wrtc_ws.onclose = (event) => {
            console.log("wrtc_ws closed.");
        };
    };
    for(const bind of argv.bind){
        const nodes = bind.split(':');
        if(isValidPort(nodes[0])){
            const port = nodes[0];
            const wrtckey = nodes[1];

            console.log("websocket server starting up");
            express_app = express();
            express_app.use(cors());
            express_app.use(express.json());
            server = http.Server(express_app);
            server.listen(port, function() {
                console.log('listening http on *:' + port);
            });

            const ws = new WebSocket.Server({ server });
            ws.on("connection", ws => {
                wrtc2ws.start_wrtc_client(wrtckey, (wrtc_ws) => {
                    bind_wrtc_and_ws(wrtc_ws, ws);
                });
            });
        }else{
            const port = nodes[1];
            const wrtckey = nodes[0];
            wrtc2ws.start_wrtc_host(wrtckey, (wrtc_ws) => {
                var ws = new WebSocket(`http://localhost:${port}`);
                bind_wrtc_and_ws(wrtc_ws, ws);
            });
        }
    }
}

if (require.main === module) {
    main();
}
