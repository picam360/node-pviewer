const mqtt = require("mqtt");

var PLUGIN_NAME = "tb_mqtt";

var m_plugin_host = null;
var m_options = null;
var m_mqtt_client = null;
var m_connected = false;

var self = {
    create_plugin: function (plugin_host) {
        m_plugin_host = plugin_host;
        console.log("create thingsboard mqtt plugin");

        var plugin = {
            name: PLUGIN_NAME,

            init_options: function (options) {
                m_options = options["tb_mqtt"];

                if (!m_options || !m_options.enabled) {
                    return;
                }

                /*
                {
                    "enabled": true,
                    "host": "demo.thingsboard.io",
                    "port": 1883,
                    "accessToken": "xxxxxxxxxxxxxxxxxxxx",
                    "useSSL": false,
                    "redis_channels":[
                        "camera/status",
                        "camera/image"
                    ]
                }
                */

                const protocol = m_options.useSSL ? "mqtts" : "mqtt";

                const url = `${protocol}://${m_options.host}:${m_options.port}`;

                m_mqtt_client = mqtt.connect(url, {
                    username: m_options.accessToken,
                    password: "",
                    reconnectPeriod: 5000
                });

                m_mqtt_client.on("connect", () => {
                    console.log("thingsboard mqtt connected");
                    m_connected = true;
                });

                m_mqtt_client.on("reconnect", () => {
                    console.log("thingsboard reconnecting...");
                    m_connected = false;
                });

                m_mqtt_client.on("close", () => {
                    console.log("thingsboard connection closed");
                    m_connected = false;
                });

                m_mqtt_client.on("offline", () => {
                    console.log("thingsboard offline");
                    m_connected = false;
                });

                m_mqtt_client.on("error", (err) => {
                    console.error(err);
                    m_connected = false;
                });

                m_plugin_host.get_tb_mqtt_client = () => {
                    return m_mqtt_client;
                };

                //
                // Telemetry Publish
                //
                m_plugin_host.tb_publish = (payload) => {

                    if (!m_connected) {
                        console.warn("thingsboard not connected");
                        return false;
                    }

                    if (typeof payload !== "string") {
                        payload = JSON.stringify(payload);
                    }

                    m_mqtt_client.publish(
                        "v1/devices/me/telemetry",
                        payload,
                        { qos: 0 }
                    );

                    return true;
                };

                //
                // Attribute Publish
                //
                m_plugin_host.tb_publish_attributes = (payload) => {

                    if (!m_connected) {
                        return false;
                    }

                    if (typeof payload !== "string") {
                        payload = JSON.stringify(payload);
                    }

                    m_mqtt_client.publish(
                        "v1/devices/me/attributes",
                        payload,
                        { qos: 0 }
                    );

                    return true;
                };

                //
                // Redis ? ThingsBoard
                //
                if (m_options.redis_channels && m_plugin_host.get_redis_client) {
                    setTimeout(() => {
                        for (const channel of m_options.redis_channels) {
                            const client = m_plugin_host.get_redis_client();
                            const subscriber = client.duplicate();
                            subscriber.connect().then(() => {
                                console.log("redis subscriber connected");
                                subscriber.subscribe(channel, (data) => {
                                    let payload;
                                    try {
                                        payload = JSON.parse(data);
                                    } catch (e) {
                                        payload = {};
                                        payload[channel] = data;
                                    }
                                    m_plugin_host.tb_publish(payload);
                                });
                            });
                        }
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