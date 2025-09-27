
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
            command_handler: function (cmd, _args, conn) {
                switch(cmd){
                case "start_stream":
                    const args_ary = (Array.isArray(_args) ? _args : [_args]);
                    for (const args of args_ary) {
                        const pstdef = (typeof args === "string") ? args : args.pstdef;
                        const delay_ms = (typeof args === "string") ? 0 : args.delay_ms;
                        setTimeout(() => {
                            plugin.start_stream(pstdef, () => {
                                console.log("done");
                            });
                        }, delay_ms || 0);
                    }
                    break;
                default:
                    break;
                }
            },
            start_stream : (pstdef, callback) => {
                m_plugin_host.build_pstreamer(pstdef, (res) => {
                    for(var key in res.params) {
                        var dotpos = key.lastIndexOf(".");
                        var name = key.substr(0, dotpos);
                        var param = key.substr(dotpos + 1);
                        var value = res.params[key];
                        if(!name || !param || !value){
                            continue;
                        }
                        pstcore.pstcore_set_param(res.pst, name, param, value);
                    }

                    var eob = true;
                    pstcore.pstcore_set_dequeue_callback(res.pst, (data)=>{
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
                    pstcore.pstcore_start_pstreamer(res.pst);
                });
            },
        };
        return plugin;
    }
};
module.exports = self;