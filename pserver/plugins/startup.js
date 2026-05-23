
var fs = require("fs");
var sprintf = require('sprintf-js').sprintf;

var m_options = {};
var PLUGIN_NAME = "startup";

var self = {
    create_plugin: function (plugin_host) {
        console.log(`create plugin : ${PLUGIN_NAME}`);
        var plugin = {
            name: PLUGIN_NAME,
            init_options: function (options) {
                m_options = options["startup"];
                if (!Array.isArray(m_options)) {
                    m_options = [m_options];
                }

                m_options.forEach((opt) => {
                    if (opt.enabled && opt.cmd) {
                        setTimeout(() => {
                            plugin_host.send_command(
                                opt.cmd,
                                opt.cmd_args,
                                (res) => {
                                    console.log("done");
                                }
                            );
                        }, opt.delay_ms || 1000);
                    }
                });
            },
            pst_started: function (pstcore, pst) {
            },
            pst_stopped: function (pstcore, pst) {
            },
            event_handler : function(sender, event) {
            },
            command_handler: function (cmd) {
            },
        };
        return plugin;
    }
};
module.exports = self;