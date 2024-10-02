module.exports = {
	create_plugin: function (plugin_host) {
		console.log("create encoder_y2kb plugin");
    const i2c = require('i2c-bus');
    let m_options = {};

		var plugin = {
			name: "encoder_y2kb",
			init_options: function (options) {
				m_options = options["encoder_y2kb"];

				if (m_options && m_options.enabled) {
					plugin.start_read_counter();
				}
			},
      init_counter: () => {
        // Register addresses
        const ECONF_REG_ADDR = 0x0B;

        const i2cBus = i2c.openSync(m_options.bus_num);

        try {
      
          for (const key in m_options.addrs) {
            const addr = m_options.addrs[key];
            // Set encoder config (ECONF: Data type=int32, Count dir=Up, Multiply=x2_A, Phase=A,B)
            const econf = 0x07;
            i2cBus.writeByteSync(addr, ECONF_REG_ADDR, econf);
          }
        
        } catch (error) {
          console.error("Error:", error);
        }

        i2cBus.closeSync();
      },
      start_read_counter: () => {
        // Register addresses
        const CVAL_REG_ADDR = 0x03;

        // Periodically read the encoder count
        setInterval(() => {
          // Read 4 bytes from the encoder count register

          const counts = {};
          
          let i2cBus;
          try {

            i2cBus = i2c.openSync(m_options.bus_num);
          
            for (const key in m_options.addrs) {
              const addr = parseInt(m_options.addrs[key]);
              const buffer = Buffer.alloc(4);
              i2cBus.readI2cBlockSync(addr, CVAL_REG_ADDR, buffer.byteLength, buffer);
              const count = buffer.readInt32LE(0); // Convert to signed 32-bit integer
            
              if(m_options.debug){
                console.log(`${addr}, Count: ${count}`);
              }

              counts[key] = count;
            }
      
          } catch (error) {
            console.error("Error:", error);
          } finally {
            if (i2cBus) {
              try {
                i2cBus.closeSync();
              } catch (closeError) {
                console.error("Failed to close I2C bus:", closeError);
              }
            }
          }

          if (m_plugin_host.get_redis_client) {
            const client = m_plugin_host.get_redis_client();
            if (client) {
              client.publish(`pserver-encoder`, JSON.stringify(counts), (err, reply) => {
                  if (err) {
                      console.error('Error publishing message:', err);
                  } else {
                      //console.log(`Message published to ${reply} subscribers.`);
                  }
              });
            }
          }
        }, 100); // Read every 100 ms
      },
		};
		return plugin;
	}
};
