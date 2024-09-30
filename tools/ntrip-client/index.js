
const async = require('async');
const fs = require("fs");
const os = require('os');
const { exec } = require('child_process');
const path = require('path');
const { SerialPort } = require('serialport');
const { ReadlineParser } = require('@serialport/parser-readline');
const net = require('net');

var m_options = {
    "enabled": true,
    "host": "rtk2go.com",
    "port": 2101,
    "username": "info@example.com",
    "password": "none",
    "mountpoint": "DoshishaUniv",
//    "mountpoint": "OCU_okujo",
//    "mountpoint": "MIE_UNIV",
//    "mountpoint": "NEAR-JPNs",
//    "mountpoint": "geosense_f9p_rtcm",
};

function connect_ntrip(conf, data_callback, close_callback) {

    const client = new net.Socket();

    client.on('data', (data) => {
        //console.log('ntrip server data', data.toString('utf-8'));
        if (data_callback) {
            data_callback(data);
        }
    });

    client.on('end', () => {
        console.log('ntrip server ended');
    });

    client.on('close', () => {
        console.log('ntrip server closed');
        if (close_callback) {
            close_callback();
        }
    });

    client.on('timeout', () => {
      console.error('ntrip server closed timeout');
      client.destroy();
    });

    client.on('error', (err) => {
        console.error('ntrip server error:', err);
        client.destroy();
    });

    client.connect(conf.port, conf.host, () => {
        client.setTimeout(10000);

        console.log(`Connected to ${conf.host}: ${conf.mountpoint}`);

        const auth = Buffer.from(`${conf.username}:${conf.password}`).toString('base64');
        var req = `GET /${conf.mountpoint} HTTP/1.0\r\n` +
            `Authorization: Basic ${auth}\r\n` +
            `User-Agent: NTRIP YourClient/1.0\r\n` +
            `Accept: */*\r\n` +
            `Connection: close\r\n`;

        client.write(req);
    });
}

function main() {

    const redis = require('redis');
    const client = redis.createClient({
        host: 'localhost',
        port: 6379,
    });
    client.on('error', (err) => {
        console.error('redis error:', err);
    });
    client.connect().then(() => {
        console.log('redis connected:');
    });

    connect_ntrip(m_options, (data) => {
        //if(data[0] != 211){//0xD3
        //    console.log("rtcm need to start preamble", data.slice(0, 128).toString('utf-8'));
        //    return;
        //}

        //const now_ms = new Date();
        //if(now_ms - m_last_rtcm_time_ms > 1000){
        //    m_last_rtcm_time_ms = now_ms;

            client.publish('pserver-rtcm', data, (err, reply) => {
                if (err) {
                    console.error('Error publishing message:', err);
                } else {
                    //console.log(`Message published to ${reply} subscribers.`);
                }
            });
        //}else{
        //    console.log("skip rtcm data", data.length);
        //}
    }, () => {
        console.log("error");
        process.exit();
    });
}

if (require.main === module) {
    main();
}
