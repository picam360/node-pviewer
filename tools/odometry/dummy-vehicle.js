
const async = require('async');
const fs = require("fs");
const os = require('os');
const { exec } = require('child_process');
const path = require('path');
const { SerialPort } = require('serialport');
const { ReadlineParser } = require('@serialport/parser-readline');
const net = require('net');
const yargs = require('yargs');

var m_options = {};
function main() {
    const argv = yargs
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

    const redis = require('redis');
    const client = redis.createClient({
        host: 'localhost',
        port: 6379,
    });
    client.on('error', (err) => {
        console.error('redis error:', err);
        m_redis_client = null;
    });
    client.connect().then(() => {
        console.log('redis connected:');
        m_redis_client = client;
    });

    const odom = {
        x : 0,
        y : 0,
        heading : 0,
    };
    setInterval(() => {

        const timestamp = Date.now() / 1000;
        client.publish('pserver-odometry-info', JSON.stringify({
            "mode" : "LOCALIZATION",
            "state" : "UPDATE_ODOMETRY",
            odom,
            timestamp,
        }));

    }, 200);
    {
        const subscriber = client.duplicate();
        subscriber.connect().then(() => {
            console.log('redis subscriber connected:');

            subscriber.subscribe('pserver-vehicle-wheel', (data, key) => {
                var params = data.trim().split(' ');
                switch (params[0]) {
                    case "CMD":
                        const req = data.substring(4);
                        if(m_options.debug){
                            console.log(`"${data}" subscribed.`, req);
                        }
                        const speed = 0.001;
                        const turn_speed = 0.1;
                        if(req.startsWith("REQ MOVE_FORWARD")){
                            odom.x += speed * Math.sin(odom.heading);
                            odom.y += speed * Math.cos(odom.heading);
                        }else if(req.startsWith("REQ MOVE_BACKWARD")){
                            odom.x -= speed * Math.sin(odom.heading);
                            odom.y -= speed * Math.cos(odom.heading);
                        }else if(req.startsWith("REQ TURN_LEFT")){
                            odom.heading -= turn_speed;
                        }else if(req.startsWith("REQ TURN_RIGHT")){
                            odom.heading += turn_speed;
                        }
                        break;
                }
            });
        });
    }
}

if (require.main === module) {
    main();
}