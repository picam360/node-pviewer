
const async = require('async');
const fs = require("fs");
const os = require('os');
const { exec } = require('child_process');
const path = require('path');
const { SerialPort } = require('serialport');
const { ReadlineParser } = require('@serialport/parser-readline');
const net = require('net');

var m_options = {};

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
function main() {
    const path = "/dev/ttyACM0";
    let m_msg_queue = [];
    let m_last_nmea = [];

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

    if(true){
        const subscriber = client.duplicate();
        subscriber.connect().then(() => {
            console.log('redis subscriber connected:');

            subscriber.subscribe('pserver-rtcm', (message, key) => {
                const data = Buffer.from(message);

                var chunks = chunkDataWithSequenceAndChecksum(data, 64);
                m_msg_queue.push("RES GET_RTCM start\n");
                for(const chunk of chunks){
                    var base64str = Buffer.from(chunk).toString('base64');
                    m_msg_queue.push(`RES GET_RTCM ${base64str}\n`);
                }
                m_msg_queue.push("RES GET_RTCM end\n");

            });
        });
    }

    const port = new SerialPort({
        path,
        baudRate: 115200,
    });

    const parser = port.pipe(new ReadlineParser({ delimiter: '\n' }));

    port.on('open', () => {
        console.log('Serial port opened');
    });
    port.on('close', () => {
        console.log('Serial port closed');
        process.exit();
    });
    port.on('error', (err) => {
        console.log('Serial port error ' + err);
        process.exit();
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

                        client.publish('pserver-nmea', params[2], (err, reply) => {
                            if (err) {
                                console.error('Error publishing message:', err);
                            } else {
                                //console.log(`Message published to ${reply} subscribers.`);
                            }
                        });
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
}

if (require.main === module) {
    main();
}