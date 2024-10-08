
const async = require('async');
const fs = require("fs");
const os = require('os');
const { exec } = require('child_process');
const path = require('path');
const { SerialPort } = require('serialport');
const { ReadlineParser } = require('@serialport/parser-readline');
const net = require('net');

var pstcore = require('node-pstcore');

var m_options = {};
var PLUGIN_NAME = "serial_interface";

var m_ntrip_conf = {
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

const getIPAddress = (callback, callback_arg) => {
    const networkInterfaces = os.networkInterfaces();
    for (const interfaceName in networkInterfaces) {
        const addresses = networkInterfaces[interfaceName];
        for (const address of addresses) {
            if (address.family === 'IPv4' && !address.internal) {
                callback(address.address, callback_arg);
                return;
            }
        }
    }
    callback('IP_NOT_FOUND', callback_arg);
};

const getSSID = (callback, callback_arg) => {
    const platform = os.platform();

    let command;
    if (platform === 'win32') {
        command = 'netsh wlan show interfaces';
    } else if (platform === 'darwin') {
        command = '/System/Library/PrivateFrameworks/Apple80211.framework/Versions/Current/Resources/airport -I';
    } else if (platform === 'linux') {
        command = 'nmcli -t -f active,ssid dev wifi | egrep \'^yes:\' | cut -d\\: -f2';
    } else {
        console.log('Unsupported platform:', platform);
        return;
    }

    exec(command, (err, stdout, stderr) => {
        if (err) {
            console.error('Error executing command:', err);
            callback('ERROR_OCURED', callback_arg);
            return;
        }

        if (platform === 'win32') {
            const match = stdout.match(/SSID\s*:\s*(.+)/);
            if (match && match[1]) {
                callback(match[1].trim(), callback_arg);
            } else {
                callback('SSID_NOT_FOUND', callback_arg);
            }
        } else if (platform === 'darwin') {
            const match = stdout.match(/ SSID: (.+)/);
            if (match && match[1]) {
                callback(match[1].trim(), callback_arg);
            } else {
                callback('SSID_NOT_FOUND', callback_arg);
            }
        } else if (platform === 'linux') {
            if (stdout) {
                callback(stdout.trim(), callback_arg);
            } else {
                callback('SSID_NOT_FOUND', callback_arg);
            }
        }
    });
};

const resetWifi = (callback, callback_arg) => {
    let command = `bash ${__dirname}/../../docker/jetson/wifi/reset_wifi.sh`;

    exec(command, (err, stdout, stderr) => {
        if (err || stderr) {
            console.error('Error executing command:', err, stderr);
            callback(false, callback_arg);
            return;
        }
        console.log('Command executed successfully');
        console.log('stdout:', stdout);
        callback(true, callback_arg);
    });
};

const enableAPMode = (ippaddress, ssid, password, callback, callback_arg) => {
    let command = `bash ${__dirname}/../../docker/jetson/wifi/setup_wifi_host.sh ${ippaddress} ${ssid} ${password}`;

    exec(command, (err, stdout, stderr) => {
        if (err || stderr) {
            console.error('Error executing command:', err, stderr);
            callback(false, callback_arg);
            return;
        }
        console.log('Command executed successfully');
        console.log('stdout:', stdout);
        callback(true, callback_arg);
    });
};

const connectWifi = (ssid, password, callback, callback_arg) => {
    const platform = os.platform();
    let command;

    if (platform === 'win32') {
        // Windows
        command = `netsh wlan add profile filename="wifi-profile.xml" & netsh wlan connect name="${ssid}" ssid="${ssid}" interface="Wi-Fi"`;
        // Windows用のWiFiプロファイルXMLを作成
        const wifiProfile = `
      <WLANProfile xmlns="http://www.microsoft.com/networking/WLAN/profile/v1">
        <name>${ssid}</name>
        <SSIDConfig>
          <SSID>
            <name>${ssid}</name>
          </SSID>
        </SSIDConfig>
        <connectionType>ESS</connectionType>
        <connectionMode>auto</connectionMode>
        <MSM>
          <security>
            <authEncryption>
              <authentication>WPA2PSK</authentication>
              <encryption>AES</encryption>
              <useOneX>false</useOneX>
            </authEncryption>
            <sharedKey>
              <keyType>passPhrase</keyType>
              <protected>false</protected>
              <keyMaterial>${password}</keyMaterial>
            </sharedKey>
          </security>
        </MSM>
      </WLANProfile>
    `;
        require('fs').writeFileSync('wifi-profile.xml', wifiProfile);
    } else if (platform === 'darwin') {
        // macOS
        command = `networksetup -setairportnetwork en0 "${ssid}" "${password}"`;
    } else if (platform === 'linux') {
        // Linux
        if (!password) {
            command = `nmcli connection up "${ssid}"`;
        } else {
            command = `nmcli dev wifi connect "${ssid}" password "${password}"`;
        }
    } else {
        console.log('Unsupported platform:', platform);
        return;
    }

    exec(command, (err, stdout, stderr) => {
        if (err || stderr) {
            console.error('Error executing command:', err, stderr);
            callback(false, callback_arg);
            return;
        }
        console.log('Command executed successfully');
        console.log('stdout:', stdout);
        callback(true, callback_arg);
    });
};

const getWifiNetworks = (callback, callback_arg) => {
    const platform = os.platform();
    let command;

    if (platform === 'win32') {
        // Windows
        command = 'netsh wlan show networks mode=Bssid';
    } else if (platform === 'darwin') {
        // macOS
        command = '/System/Library/PrivateFrameworks/Apple80211.framework/Versions/Current/Resources/airport -s';
    } else if (platform === 'linux') {
        // Linux
        command = 'nmcli -t -f SSID,SIGNAL dev wifi';
    } else {
        console.log('Unsupported platform:', platform);
        return;
    }

    exec(command, (err, stdout, stderr) => {
        if (err) {
            console.error('Error executing command:', err);
            return;
        }

        let networks = [];
        if (platform === 'win32') {
            // Windowsの場合のパース
            const ssidRegex = /SSID \d+ : (.+)/g;
            const signalRegex = /Signal\s*:\s*(\d+)%/g;
            let ssidMatch, signalMatch;
            while ((ssidMatch = ssidRegex.exec(stdout)) && (signalMatch = signalRegex.exec(stdout))) {
                networks.push({ ssid: ssidMatch[1], signal: parseInt(signalMatch[1]) });
            }
        } else if (platform === 'darwin') {
            // macOSの場合のパース
            const lines = stdout.split('\n').slice(1);
            lines.forEach(line => {
                const parts = line.split(/\s+/).filter(part => part !== '');
                const ssid = parts[0];
                const signal = parseInt(parts[parts.length - 1]);
                if (ssid && !isNaN(signal)) {
                    networks.push({ ssid, signal });
                }
            });
        } else if (platform === 'linux') {
            // Linuxの場合のパース
            const lines = stdout.split('\n');
            lines.forEach(line => {
                const [ssid, signal] = line.split(':');
                if (ssid && signal) {
                    networks.push({ ssid, signal: parseInt(signal) });
                }
            });
        }
        callback(networks, callback_arg);
    });
};

const MAX_CHUNK_SIZE = 512;
const CHUNK_META_SIZE = 2;
function calculateChecksum(bytes) {
    return bytes.reduce((acc, byte) => acc ^ byte, 0);
}
function chunkDataWithSequenceAndChecksum(data, chunkSize) {
    var sequenceNumber = 0;
    var chunks = [];

    for (var offset = 0; offset < data.byteLength; offset += chunkSize) {
        var end = Math.min(offset + chunkSize, data.byteLength);
        var chunkData = new Uint8Array(data.slice(offset, end));

        // シーケンス番号（先頭）とチェックサム（末尾）を追加
        var chunkWithMetadata = new Uint8Array(end - offset + CHUNK_META_SIZE);
        chunkWithMetadata[0] = sequenceNumber;
        chunkWithMetadata.set(chunkData, 1);
        chunkWithMetadata[chunkWithMetadata.length - 1] = calculateChecksum(chunkData);

        chunks.push(chunkWithMetadata);

        sequenceNumber = (sequenceNumber + 1) & 0xFF; // シーケンス番号を1バイトで循環
    }

    return chunks;
}

var self = {
    create_plugin: function (plugin_host) {
        m_plugin_host = plugin_host;
        m_rtcm_data = "";
        m_msg_queue = [];
        m_last_rtcm_time_ms = 0;
        m_last_nmea = [];
        console.log("create host plugin");
        var plugin = {
            name: PLUGIN_NAME,
            init_options: function (options) {
                m_options = options["serial_interface"];

                if(!m_options || !m_options.enabled){
                    return;
                }

                if (m_options.path) {
                    plugin.handle_serial_interface(m_options.path);
                }

                if (m_ntrip_conf.enabled) {
                    plugin.handle_ntrip();
                }
            },
            pst_started: function (pstcore, pst) {
            },
            pst_stopped: function (pstcore, pst) {
            },
            command_handler: function (cmd, conn) {
            },
            handle_ntrip: () => {
                // const { NtripClient } = require('ntrip-client');

                // const client = new NtripClient(m_ntrip_conf);

                // client.on('data', (data) => {
                //     if(data[0] != 211){//0xD3
                //         console.log("rtcm need to start preamble", data.toString('utf-8'));
                //         return;
                //     }
                //     //console.log(data);
                //     var chunks = chunkDataWithSequenceAndChecksum(data, 256);
                //     m_msg_queue.push("RES GET_RTCM start\n");
                //     for(const chunk of chunks){
                //         var base64str = Buffer.from(chunk).toString('base64');
                //         m_msg_queue.push(`RES GET_RTCM ${base64str}\n`);
                //     }
                //     m_msg_queue.push("RES GET_RTCM end\n");
                // });
                
                // client.on('close', () => {
                //     console.log("ntrip-clientt closed");
                // });
                
                // client.on('error', (err) => {
                //     console.log("ntrip-clientt error", err);
                // });
                
                // client.run();

                connect_ntrip(m_ntrip_conf, (data) => {
                    //if(data[0] != 211){//0xD3
                    //    console.log("rtcm need to start preamble", data.slice(0, 128).toString('utf-8'));
                    //    return;
                    //}

                    //const now_ms = new Date();
                    //if(now_ms - m_last_rtcm_time_ms > 1000){
                    //    m_last_rtcm_time_ms = now_ms;

                        var chunks = chunkDataWithSequenceAndChecksum(data, 64);
                        m_msg_queue.push("RES GET_RTCM start\n");
                        for(const chunk of chunks){
                            var base64str = Buffer.from(chunk).toString('base64');
                            m_msg_queue.push(`RES GET_RTCM ${base64str}\n`);
                        }
                        m_msg_queue.push("RES GET_RTCM end\n");
                    //}else{
                    //    console.log("skip rtcm data", data.length);
                    //}
                }, () => {
                    if(m_ntrip_conf.timeout){
                       clearTimeout(m_ntrip_conf.timeout); 
                    }
                    m_ntrip_conf.timeout = setTimeout(() => {
                        plugin.handle_ntrip();
                        m_ntrip_conf.timeout = 0;
                    }, 5000);
                });
            },
            handle_serial_interface: (path) => {

                const port = new SerialPort({
                    path,
                    baudRate: 115200,
                });

                const parser = port.pipe(new ReadlineParser({ delimiter: '\n' }));

                function reconnect(port) {
                    if (port.timeout) {
                        return;
                    }
                    port.timeout = setTimeout(() => {
                        plugin.handle_serial_interface(path);
                    }, 5000);
                }

                port.on('open', () => {
                    console.log('Serial port opened');
                });
                port.on('close', () => {
                    console.log('Serial port closed');
                    reconnect(port);
                });
                port.on('error', (err) => {
                    console.log('Serial port error ' + err);
                    reconnect(port);
                });

                setInterval(() => {
                    var size = 0;
                    for(var i in m_msg_queue){
                        const msg = m_msg_queue[i];
                        size += msg.length;
                        if(size > 512){//limit bytes of once
                            m_msg_queue = m_msg_queue.slice(i);
                            return;
                        }
                        port.write(msg, (err) => {
                            if (err) {
                                return console.log('Error on write:', err.message, res);
                            }
                        });
                    }
                    m_msg_queue = [];
                }, 50);

                parser.on('data', (data) => {
                    const options = {
                        header : "",
                    };
                    if (data.startsWith("CAMRX ")) {
                        const strs = data.split(/ (.+)/);
                        options.header = "CAMTX ";
                        data = strs[1];
                    }
                    if (data.startsWith("ECH ")) {
                        //console.log(data);
                    } else if (data.startsWith("DBG ")) {
                        console.log(data);
                    } else if (data.startsWith("INF ")) {
                        //console.log(data);
                    } else if (data.startsWith("ERR ")) {
                        console.log(data);
                    } else if (data.startsWith("REQ ")) {
                        var params = data.trim().split(' ');
                        switch (params[1]) {
                            case "GET_RTCM"://from esp32
                                break;
                            case "GET_IP"://from esp32
                                getIPAddress((ip_address, options) => {
                                    m_msg_queue.push(options.header + `RES GET_IP ${ip_address}\n`);
                                }, options);
                                break;
                            case "GET_SSID"://from esp32
                                getSSID((ssid, options) => {
                                    m_msg_queue.push(options.header + `RES GET_SSID ${ssid}\n`);
                                }, options);
                                break;
                            case "SET_NMEA"://from esp32
                                try {
                                    //console.log(stat);
                                    m_msg_queue.push(options.header + `RES SET_NMEA\n`);
                                    if(!params[2]){
                                        return;
                                    }
                                    const nmea = params[2].split(',');

                                    if(m_last_nmea[6] != nmea[6]){
                                        console.log("FIX changed", m_last_nmea[6], nmea[6]);
                                    }
                                    m_last_nmea = nmea;

                                    if(m_plugin_host.get_redis_client){
                                        const client = m_plugin_host.get_redis_client();
                                        if(client){
                                            client.publish('pserver-nmea', params[2], (err, reply) => {
                                                if (err) {
                                                    console.error('Error publishing message:', err);
                                                } else {
                                                    //console.log(`Message published to ${reply} subscribers.`);
                                                }
                                            });
                                        }
                                    }
                                } catch (err) {
                                    //console.log(err);
                                }
                                break;
                            case "RESET_WIFI"://from ble
                                resetWifi((succeeded, options) => {
                                    m_msg_queue.push(options.header + `RES RESET_WIFI ${succeeded ? "SUCCEEDED" : "FAILED"}\n`);
                                }, options);
                                break;
                            case "CONNECT_WIFI"://from ble
                                connectWifi(params[2], params[3], (succeeded, options) => {
                                    m_msg_queue.push(options.header + `RES CONNECT_WIFI ${succeeded ? "SUCCEEDED" : "FAILED"}\n`);
                                }, options);
                                break;
                            case "ENABLE_APMODE"://from ble
                                enableAPMode(params[2] || "1", params[3] || "", params[4] || "", (succeeded, options) => {
                                    m_msg_queue.push(options.header + `RES ENABLE_APMODE ${succeeded ? "SUCCEEDED" : "FAILED"}\n`);
                                }, options);
                                break;
                            case "GET_WIFI_NETWORKS"://from ble
                                getWifiNetworks((list, options) => {
                                    list.sort((a, b) => b.signal - a.signal);
                                    let list_str = list.map(net => net.ssid).join(' ');
                                    if (list_str.length > 500) {
                                        list_str = list_str.substring(0, 500);
                                        list_str = list_str.substring(0, list_str.lastIndexOf(' '));
                                    }
                                    m_msg_queue.push(options.header + `RES GET_WIFI_NETWORKS ${list_str}\n`);
                                }, options);
                                break;
                        }
                    }else{
                        console.log('Received data:', data);
                    }
                });
            },
        };
        return plugin;
    }
};
module.exports = self;
