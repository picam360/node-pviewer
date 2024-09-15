
const async = require('async');
const fs = require("fs");
const os = require('os');
const { exec } = require('child_process');
const path = require('path');

var pstcore = require('node-pstcore');

var PLUGIN_NAME = "start_stream";

var self = {
    create_plugin: function (plugin_host) {
        m_plugin_host = plugin_host;
        console.log(`create plugin : ${PLUGIN_NAME}`);
        var plugin = {
            name: PLUGIN_NAME,
            init_options: function (options) {
            },
            command_handler: function (cmd, args, conn) {
                switch(cmd){
                case "start_stream":
                    plugin.start_stream(options.name, args, () => {
                        console.log("done");
                    });
                    break;
                default:
                    break;
                }
            },
            start_stream : (name, params, callback) => {
                m_plugin_host.build_pstreamer(name, (pst) => {
                    var eob = true;
                    pstcore.pstcore_set_dequeue_callback(pst, (data)=>{
                        if(data == null){//eob
                            if(eob){//eos
                                pstcore.pstcore_destroy_pstreamer(pst);
                                if(params && params.callback){
                                    params.callback();
                                }
                            }else{
                                eob = true;
                            }
                        }else{
                            eob = false;
                        }
                    });
                    pstcore.pstcore_start_pstreamer(pst);
                });
            },
        };
        return plugin;
    }
};
module.exports = self;