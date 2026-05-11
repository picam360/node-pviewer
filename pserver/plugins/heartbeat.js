const PLUGIN_NAME = "heartbeat";

const self = {
    create_plugin: function (plugin_host) {
        m_plugin_host = plugin_host;
        console.log("create host plugin");
        const plugin = {
            name: PLUGIN_NAME,
            heartbeat_count : 0,
            init_options: function (options) {
                m_options = options["heartbeat"];

                setInterval(() => {
                    if(!m_plugin_host.get_redis_client){
                        return;
                    }
					const client = m_plugin_host.get_redis_client();
					if (client) {
                        plugin.heartbeat_count++;
						client.publish('pserver-heartbeat', `${plugin.heartbeat_count}`, (err, reply) => {
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