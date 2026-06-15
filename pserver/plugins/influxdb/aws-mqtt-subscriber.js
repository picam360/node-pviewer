const PLUGIN_NAME = "influxdb-aws-mqtt-subscriber";
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
                m_options = options["influxdb-aws-mqtt-subscriber"] || {};

                if(m_options.aws_topics && m_plugin_host.get_aws_iot_mqtt_client){
                    setTimeout(() => {
    
                        const mqtt_client = m_plugin_host.get_aws_iot_mqtt_client();

                        const influx = new InfluxDB({
                            url: m_options.url,
                            token: m_options.token,
                        });

                        const writeApis = {};
                        
                        for(const topic of Object.keys(m_options.aws_topics)){

                            writeApis[topic] = influx.getWriteApi(
                                "picam360",
                                topic,
                                "ms"
                            );

                            mqtt_client.subscribe(
                                `devices/${m_options.device}/${topic}`,
                                { qos: 0 },
                                (err) => {
                                    if (err) {
                                        console.log(err);
                                    } else {
                                        console.log("Subscribed", topic);
                                    }
                                }
                            );
                        }

                        mqtt_client.on("message", (full_topic, payload) => {

                            try {

                                const json = JSON.parse(payload.toString());

                                // devices/nenkoumuseum-001/pserver-jetson-stats
                                const parts = full_topic.split("/");
                                const device = parts[1];
                                const topic = parts[2];

                                const point = new Point(topic)
                                    .tag("device", device);

                                for(const field of m_options.aws_topics[topic]){
                                    if(json[field] != null){
                                        point.floatField(field, json[field]);
                                    }
                                }

                                writeApis[topic].writePoint(point);

                                //console.log(device, json);

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