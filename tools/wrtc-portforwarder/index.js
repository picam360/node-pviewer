
const fs = require("fs");
const jsonc = require('jsonc-parser');
const net = require('net');
const yargs = require('yargs');
const wrtc_utils = require("./wrtc-utils.js");


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
    for(const bind of argv.bind){
        const nodes = bind.split(':');
        if(isValidPort(nodes[0])){
            const port = nodes[0];
            const wrtckey = nodes[1];

            console.log("socket server starting up");
            const server = net.createServer((socket) => {
                console.log('client connection established');
                wrtc_utils.start_wrtc_client(wrtckey, (dc) => {
                    wrtc_utils.bind_wrtc_and_socket(dc, socket);
                });
            });
            
            server.listen(port, () => {
                console.log('listening socket on *:' + port);
            });
        }else{
            const port = nodes[1];
            const wrtckey = nodes[0];
            wrtc_utils.start_wrtc_host(wrtckey, (dc) => {
                const socket = net.createConnection({ host: 'localhost', port: port }, () => {
                    console.log('connection established');
                });
                wrtc_utils.bind_wrtc_and_socket(dc, socket);
            });
        }
    }
}

if (require.main === module) {
    main();
}
