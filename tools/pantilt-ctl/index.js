const fs = require("fs");
const os = require('os');
const { exec, execSync } = require('child_process');
const path = require('path');
const net = require('net');
const yargs = require('yargs');

var m_options = {
    debug: false,
};

function main() {
    const argv = yargs
        .option('pantilt', {
            type: 'string',
            default: '127.0.0.1:9999',
            description: 'pantilt "host:port"',
        })
        .option('redis', {
            type: 'string',
            default: 'localhost:6379',
            description: 'redis "host:port"',
        })
        .option('debug', {
            type: 'boolean',
            default: false,
            description: 'debug flag',
        })
        .help()
        .alias('help', 'h')
        .argv;

    m_options.debug = argv.debug;

    let m_msg_queue = [];

    const redis = require('redis');
    const client = redis.createClient({
        url: `redis://${argv.redis}`
    });
    client.on('error', (err) => {
        console.error('redis error:', err);
        m_redis_client = null;
    });
    client.connect().then(() => {
        console.log('redis connected:');
        m_redis_client = client;
    });

    const subscriber = client.duplicate();
    subscriber.connect().then(() => {
        console.log('redis subscriber connected:');

        subscriber.subscribe('pserver-pantilt-ctl', (data, key) => {
            try{
                var obj = JSON.parse(data);
                switch (obj.cmd) {
                    case "move":
                        if(m_options.debug){
                            console.log(`"${data}" subscribed.`);
                        }
                        const packet = `MOVE PACKET ${JSON.stringify(obj)}`;
                        m_msg_queue.push(packet);
                        break;
                }
            }catch(err){
                console.log(err);
            }
        });
    });

    let last_data_ts = Date.now();

    const [HOST, PORT] = argv.pantilt.split(':');
    const socket = new net.Socket();

    socket.connect(Number(PORT), HOST, () => {
        console.log(`Connected to ${argv.pantilt}`);
        const packet = 'Hello, server!';
        m_msg_queue.push(packet);
    });

    socket.on('close', () => {
        console.log('Connection closed');
    });

    socket.on('error', (err) => {
        console.log('Client error:', err);
    });

    setInterval(() => {
        let now = Date.now();

        var size = 0;
        for(var i in m_msg_queue){
            const msg = m_msg_queue[i];
            size += msg.length;
            if(size > 512){//limit bytes of once
                m_msg_queue = m_msg_queue.slice(i);
                return;
            }
            if(m_options.debug){
                console.log("Send:", msg.toString());
            }
            socket.write(msg, (err) => {
                if (err) {
                    return console.log('Error on write:', err.message);
                }
            });
        }
        m_msg_queue = [];
    }, 50);

    socket.on('data', (data) => {
        if(m_options.debug){
            console.log('Received:', data.toString());
        }
    });

}

if (require.main === module) {
    main();
}