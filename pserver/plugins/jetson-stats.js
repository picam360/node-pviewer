const PLUGIN_NAME = "jetson-stats";
const fs = require('fs');
const { exec, execSync } = require('child_process');

function readValue(path, scale = 1) {
  try {
    const data = fs.readFileSync(path, 'utf8').trim();
    return parseFloat(data) / scale;
  } catch (e) {
    return null;
  }
}
function getArpTable(callback) {
    exec('arp -an',
        { encoding: 'utf8' },
        (error, stdout, stderr) => {

            // const et = performance.now();
            // const elapsed_ms = et - st;
            // if (elapsed_ms > 100) {
            //     console.log(`getSSID too much slow: ${elapsed_ms.toFixed(2)} ms`);
            // }

            if (error) {
                callback(error, null);
                return;
            }

            if (stderr) {
                callback(new Error(stderr), null);
                return;
            }

            const map = {};

            stdout.split('\n').forEach(line => {
                const match = line.match(/\((.*?)\) at ([0-9a-f:]{17})/i);
                if (!match) return;

                const ip = match[1];
                const mac = match[2].toLowerCase();

                map[mac] = ip;
            });

            callback(null, map);
        }
    );
}

function getSSID(callback) {
    //const st = performance.now();
    exec('nmcli -t -f active,ssid dev wifi | egrep \'^yes:\' | cut -d\\: -f2',
        { encoding: 'utf8' },
        (error, stdout, stderr) => {

            // const et = performance.now();
            // const elapsed_ms = et - st;
            // if (elapsed_ms > 100) {
            //     console.log(`getSSID too much slow: ${elapsed_ms.toFixed(2)} ms`);
            // }

            if (error) {
                callback(error, null);
                return;
            }

            if (stderr) {
                callback(new Error(stderr), null);
                return;
            }

            callback(null, stdout.trim());
        }
    );
}

function getStations(wifi_device, callback) {
    getArpTable((err, arpMap) => {
        if (err) {
            console.error(err);
            callback(err, null);
            return;
        }

        //const st = performance.now();
        exec(`iw dev ${wifi_device} station dump`,
            { encoding: 'utf8' },
            (error, stdout, stderr) => {
    
                // const et = performance.now();
                // const elapsed_ms = et - st;
                // if (elapsed_ms > 100) {
                //     console.log(`getSSID too much slow: ${elapsed_ms.toFixed(2)} ms`);
                // }
    
                if (error) {
                    callback(error, null);
                    return;
                }
    
                if (stderr) {
                    callback(new Error(stderr), null);
                    return;
                }

                const stations = {};

                const blocks = stdout.split('Station ').slice(1);
        
                blocks.forEach(block => {
                    const lines = block.split('\n').map(l => l.trim()).filter(Boolean);
        
                    const mac = lines[0].split(' ')[0].toLowerCase();
                    const data = {};
        
                    lines.slice(1).forEach(line => {
                        const match = line.match(/^([^:]+):\s*(.+)$/);
                        if (!match) return;
        
                        const key = match[1].trim();
                        const value = match[2].trim();
        
                        data[key] = value;
                    });
        
                    const ip = arpMap[mac];
        
                    if (ip) {
                        stations[ip] = {
                            mac,
                            ...data
                        };
                    } else {
                        stations[mac] = {
                            mac,
                            ...data,
                            ip: null
                        };
                    }
                });
        
                callback(null, stations);
            }
        );
    });
}

const self = {
    create_plugin: function (plugin_host) {
        const m_plugin_host = plugin_host;
        let m_options = {};
        console.log("create host plugin");
        const plugin = {
            name: PLUGIN_NAME,
            ssid: "NOT_INITIALIZED",
            wifi_stations: {},
            init_options: function (options) {
                m_options = options["jetson-stats"] || {};

                const wifi_device = m_options.wifi_device || "wlan0";

                let last_aws_publish_date = Date.now();
                setInterval(() => {
                    getSSID((err, ssid) => {
                        if (err) {
                            console.error(err);
                            return;
                        }
                        
                        self.ssid = ssid;
                    });
                    getStations(wifi_device, (err, wifi_stations) => {
                        if (err) {
                            console.error(err);
                            return;
                        }
                        
                        self.wifi_stations = wifi_stations;
                    });
                }, 1000);
                setInterval(() => {
                    const now = Date.now();
                    if(!m_plugin_host.get_redis_client){
                        return;
                    }
					const client = m_plugin_host.get_redis_client();
					if (client) {
                        const voltage = readValue(
                          '/sys/bus/i2c/drivers/ina3221/1-0040/hwmon/hwmon2/in1_input',
                          1000
                        );
                        
                        const current = readValue(
                          '/sys/bus/i2c/drivers/ina3221/1-0040/hwmon/hwmon2/curr1_input',
                          1000
                        );
                        
                        const tempCPU = readValue(
                          '/sys/class/thermal/thermal_zone0/temp',
                          1000
                        );

                        const camdevs = {};
                        if(m_options["camdevs"]){
                            for(const [key, value] of Object.entries(m_options["camdevs"])){
                                if(!value.pstdef || !value.replacement){
                                    console.log("camdev needs pstdef and replacement", key);
                                    continue;
                                }
                                const ctx = m_plugin_host.get_pstdef(value.pstdef);
                                if(!ctx || !ctx.replacements[value.replacement]){
                                    console.log("no replacement", value);
                                    continue;
                                }
                                camdevs[key] = fs.existsSync(ctx.replacements[value.replacement]);
                            }
                        }

                        const stats = {
                            voltage,
                            current,
                            tempCPU,
                            ssid: self.ssid,
                            wifi_stations: self.wifi_stations,
                            camdevs,
                        };

						client.publish('pserver-jetson-stats', JSON.stringify(stats), (err, reply) => {
							if (err) {
								console.error('Error publishing message:', err);
							} else {
								//console.log(`Message published to ${reply} subscribers.`);
							}
						});
					}
                }, 1000);
            },
            pst_stopped: function (pstcore, pst) {
            },
            command_handler: function (cmd, conn) {
            },
        };
        return plugin;
    }
};
module.exports = self;