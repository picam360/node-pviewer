const PLUGIN_NAME = "influxdb";
const fs = require('fs');

const { InfluxDB, Point } = require("@influxdata/influxdb-client");


const self = {
    create_plugin: function (plugin_host) {
        const m_plugin_host = plugin_host;
        let m_options = {};
        console.log("create host plugin");
        const plugin = {
            name: PLUGIN_NAME,
            init_options: function (options) {
                m_options = options["influxdb"] || {};

                if(m_options.subscribe_aws && m_plugin_host.get_aws_iot_mqtt_client){
                    setTimeout(() => {
    
                        const mqtt_client = m_plugin_host.get_aws_iot_mqtt_client();

                        const influx = new InfluxDB({
                            url: m_options.url,
                            token: m_options.token,
                        });

                        const writeApi = influx.getWriteApi(
                            "picam360",
                            "jetson-stats",
                            "ms"
                        );
                        
                        mqtt_client.subscribe(
                            `devices/${m_options.device}/pserver-jetson-stats`,
                            { qos: 0 },
                            (err) => {
                                if (err) {
                                    console.log(err);
                                } else {
                                    console.log("Subscribed");
                                }
                            }
                        );

                        mqtt_client.on("message", (topic, payload) => {

                            try {

                                const json = JSON.parse(payload.toString());

                                // devices/nenkoumuseum-001/pserver-jetson-stats
                                const parts = topic.split("/");
                                const device = parts[1];

                                const point = new Point("jetson_stats")
                                    .tag("device", device);

                                if (json.tempCPU != null)
                                    point.floatField("tempCPU", json.tempCPU);

                                if (json.voltage != null)
                                    point.floatField("voltage", json.voltage);

                                if (json.current != null)
                                    point.floatField("current", json.current);

                                if (json.ssid)
                                    point.tag("ssid", json.ssid);

                                writeApi.writePoint(point);

                                console.log(device, json);

                            } catch (e) {
                                console.log(e);
                            }
                        });

                    }, 3000);
                }
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