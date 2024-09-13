
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
            apply_params : (str, params) => {
                if(!str){
                    return "";
                }
                for(var key in params) {
                    str = str.toString().replace(new RegExp(key, "g"), params[key]);
                }
                return str;
            },
            start_stream : (name, params, callback) => {
                var stream_params = null;
                if(m_options && m_options["stream_params"] && m_options["stream_params"][name]){
                    stream_params = m_options["stream_params"][name];
                }
                if(!stream_params || !stream_params[""]){
                    return;
                }
                var def = stream_params[""];
                def = plugin.apply_params(def, params);
                pstcore.pstcore_build_pstreamer(def, pst => {
                    if(!pst){
                        console.log("something wrong!", def);
                        return;
                    }
                    for(var key in stream_params) {
                        if(key === ""){
                            continue;
                        }
                        var dotpos = key.lastIndexOf(".");
                        var name = key.substr(0, dotpos);
                        var param = key.substr(dotpos + 1);
                        var value = stream_params[key];
                        value = plugin.apply_params(value, params);
                        if(!name || !param || !value){
                            continue;
                        }
                        pstcore.pstcore_set_param(pst, name, param, value);
                    }
        
                    var eob = true;
                    pstcore.pstcore_set_dequeue_callback(pst, (data)=>{
                        if(data == null){//eob
                            if(eob){//eos
                                pstcore.pstcore_destroy_pstreamer(pst);
                                if(callback){
                                    callback();
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