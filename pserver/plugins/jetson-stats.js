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
function getArpTable() {
    const stdout = execSync('arp -an', { encoding: 'utf8' });

    const map = {};

    stdout.split('\n').forEach(line => {
        const match = line.match(/\((.*?)\) at ([0-9a-f:]{17})/i);
        if (!match) return;

        const ip = match[1];
        const mac = match[2].toLowerCase();

        map[mac] = ip;
    });

    return map;
}


function getSSID() {
    const st = performance.now();
    const stdout = execSync('nmcli -t -f active,ssid dev wifi | egrep \'^yes:\' | cut -d\\: -f2', {
        encoding: 'utf8'
    });
    const et = performance.now();
    const elapsed_ms = et - st;
    if(elapsed_ms > 100){
        console.log(`getSSID too match slow: ${elapsed_ms.toFixed(2)} ms`);
    }
    return stdout.trim();
};

function getStations() {
    const stdout = execSync('iw dev wlan0 station dump', {
        encoding: 'utf8'
    });

    const arpMap = getArpTable();

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

    return stations;
}

const self = {
    create_plugin: function (plugin_host) {
        const m_plugin_host = plugin_host;
        console.log("create host plugin");
        const plugin = {
            name: PLUGIN_NAME,
            init_options: function (options) {
                m_options = options["jetson-stats"];

                setInterval(() => {
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

                        const ssid = getSSID();

                        const wifi_stations = getStations();

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
                            ssid,
                            wifi_stations,
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