
const async = require('async');
const fs = require("fs");
const os = require('os');
const { exec } = require('child_process');
const path = require('path');

var pstcore = require('node-pstcore');

var m_options = {};
var m_base_path = "./";
var PLUGIN_NAME = "take_picture";

var m_target_filename = "";

var self = {
    create_plugin: function (plugin_host) {
        m_plugin_host = plugin_host;
        console.log("create host plugin");

        function formatDate(date) {
            const year = date.getFullYear();
            const month = String(date.getMonth() + 1).padStart(2, '0');
            const day = String(date.getDate()).padStart(2, '0');
            const hours = String(date.getHours()).padStart(2, '0');
            const minutes = String(date.getMinutes()).padStart(2, '0');
            const seconds = String(date.getSeconds()).padStart(2, '0');
        
            return `${year}-${month}-${day}_${hours}-${minutes}-${seconds}`;
        }

        var plugin = {
            name: PLUGIN_NAME,
            init_options: function (options) {
                m_options = options["take_picture"];

                m_base_path = 'userdata/pvf';
                if(options["take_picture"] && options["take_picture"]["base_path"]){
                    m_base_path = options["take_picture"]["base_path"] + "/";
                }else if(options["http_pvf"] && options["http_pvf"]["base_path"]){
                    m_base_path = options["http_pvf"]["base_path"] + "/";
                }
            },
            command_handler: function (cmd, args, conn) {
                switch(cmd){
                case "take_picture":
                    plugin.start_stream(options.name, {
                        "FILE_PATH" : args
                    }, () => {
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
                    str = str.toString().replace(new RegExp("@" + key + "@", "g"), params[key]);
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