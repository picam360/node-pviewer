
const fs = require("fs");
const jsonc = require('jsonc-parser');
const net = require('net');
const yargs = require('yargs');
const wrtc_utils = require("./wrtc-utils.js");

//usage : node index.js -s ws://localhost:36080@your-wrtc-key

const m_options = {
    debug : false
};

function main() {
    const argv = yargs
        .option('server', {
            alias: 's',
            type: 'string',
            array: true,
            description: '[webrtc host] exp. : -s ws://localhost:36080@your-wrtc-key',
        })
        .option('client', {
            alias: 'c',
            type: 'string',
            array: true,
            description: '[webrtc client] exp. : -c localhost:8888@your-wrtc-key',
        })
        .option('debug', {
            alias: 'd',
            type: 'boolean',
            default: false,
            description: 'debug flag',
        })
        .help()
        .alias('help', 'h')
        .argv;

    m_options.debug = argv.debug;

    const isValidPort = (value) => {
        const port = Number(value);
        return Number.isInteger(port) && port >= 0 && port <= 65535;
    };
    for(const server of argv.server || []){
        const nodes = server.split('@');
        const wrtckey = nodes[1];
        
        wrtc_utils.start_wrtc_host(wrtckey, (dc) => {
            let socket = null;
            if(isValidPort(nodes[0])){
                const port = nodes[0];
                socket = net.createConnection({ host: 'localhost', port: port }, () => {
                    console.log('connection established');
                });
            }else if(nodes[0].startsWith('ws://')){
                const url = nodes[0];
                socket = new WebSocket(url);
                wrtc_utils.bind_wrtc_and_ws(dc, socket, m_options);
            };
        });
    }
    for(const client of argv.client || []){
        const nodes = client.split('@');
        if(isValidPort(nodes[0])){
            const port = nodes[0];
            const wrtckey = nodes[1];

            console.log("socket server starting up");
            const server = net.createServer((socket) => {
                console.log('client connection established');
                wrtc_utils.start_wrtc_client(wrtckey, (dc) => {
                    wrtc_utils.bind_wrtc_and_socket(dc, socket, m_options);
                });
            });
            
            server.listen(port, () => {
                console.log('listening socket on *:' + port);
            });
        }
    }
}

if (require.main === module) {
    main();
}
