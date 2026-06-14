const awsIot = require('aws-iot-device-sdk');

var PLUGIN_NAME = "aws_iot_mqtt";

var m_plugin_host = null;
var m_options = null;
var m_mqtt_client = null;
var m_connected = false;

var self = {
    create_plugin: function (plugin_host) {
        m_plugin_host = plugin_host;
        console.log("create aws iot mqtt plugin");

        var plugin = {
            name: PLUGIN_NAME,

            init_options: function (options) {
                m_options = options["aws_iot_mqtt"];

                if (!m_options || !m_options.enabled) {
                    return;
                }

                /*
                m_options example:
                {
                    "enabled": true,
                    "clientId": "sdk-nodejs-nenkoumuseum-001",
                    "keyPath": "/home/picam360/github/node-pviewer/pserver/certs/aws_iot/nenkoumuseum-001.private.key",
                    "certPath": "/home/picam360/github/node-pviewer/pserver/certs/aws_iot/nenkoumuseum-001.cert.pem",
                    "caPath": "/home/picam360/github/node-pviewer/pserver/certs/aws_iot/root-CA.crt",
                    "endpoint": "a23ifs2uf7b3dq-ats.iot.ap-northeast-1.amazonaws.com",
                    "baseTopic": "devices/nenkoumuseum-001"
                }
                */
                if (!m_options.clientId) {
                    m_options.clientId = "picam360-" + uuidgen();
                }

                m_mqtt_client = awsIot.device({
                    keyPath: m_options.keyPath,
                    certPath: m_options.certPath,
                    caPath: m_options.caPath,
                    clientId: m_options.clientId,
                    host: m_options.endpoint,
                    protocol: 'mqtts',
                    port: 8883,
                    reconnectPeriod: 5000
                });

                m_mqtt_client.on('connect', () => {
                    console.log('aws iot mqtt connected');
                    m_connected = true;
                });

                m_mqtt_client.on('reconnect', () => {
                    console.log('aws iot mqtt reconnecting...');
                    m_connected = false;
                });

                m_mqtt_client.on('close', () => {
                    console.log('aws iot mqtt closed');
                    m_connected = false;
                });

                m_mqtt_client.on('offline', () => {
                    console.log('aws iot mqtt offline');
                    m_connected = false;
                });

                m_mqtt_client.on('error', (err) => {
                    console.error('aws iot mqtt error:', err);
                    m_connected = false;
                });

                m_plugin_host.get_aws_iot_mqtt_client = () => {
                    return m_mqtt_client;
                };

                m_plugin_host.aws_iot_publish = (_topic, payload) => {
                    if (!m_mqtt_client || !m_connected) {
                        console.warn("aws iot mqtt not connected");
                        return false;
                    }

                    if (typeof payload !== "string") {
                        payload = JSON.stringify(payload);
                    }

                    const topic = `${m_options.baseTopic}/${_topic}`;
                    m_mqtt_client.publish(topic, payload, {
                        qos: 0
                    });

                    return true;
                };

                if(m_options.redis_channels && m_plugin_host.get_redis_client){

                    setTimeout(() => {
                        for(const channel of m_options.redis_channels){
                            const client = m_plugin_host.get_redis_client();
                            const subscriber = client.duplicate();
                            subscriber.connect().then(() => {
                                console.log('redis subscriber connected:');
                                
                                subscriber.subscribe(channel, (data, key) => {
                                    m_plugin_host.aws_iot_publish(channel, data);
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