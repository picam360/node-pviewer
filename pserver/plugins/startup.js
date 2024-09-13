
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
                if(m_options.enabled && m_options.cmd){
                    setTimeout(() => {
                        plugin_host.send_command(
                            m_options.cmd,
                            m_options.cmd_args,
                            (res) => {
                                console.log("done");
                            }
                        );
                    }, m_options.delay_ms || 1000);
                }
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