module.exports = {
	create_plugin: function (plugin_host) {
		console.log("create wheel plugin");

		let m_options = {};
		var m_duty = 50;// %

		var plugin = {
			name: "wheel",
			init_options: function (options) {
				m_options = options["wheel"];

				if(!m_options){
					m_options = {};
				}
			},
			command_handler: function (cmd) {
				if(!m_options.enabled){
					return;
				}

				var split = cmd.split(' ');
				cmd = split[0];
				
				if (m_plugin_host.get_redis_client) {
					const client = m_plugin_host.get_redis_client();
					if (client) {
						client.publish('pserver-vehicle-wheel', `CMD ${cmd}`, (err, reply) => {
							if (err) {
								console.error('Error publishing message:', err);
							} else {
								//console.log(`Message published to ${reply} subscribers.`);
							}
						});
					}
				}
			}
		};
		return plugin;
	}
};
