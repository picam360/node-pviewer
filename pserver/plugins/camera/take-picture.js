
const async = require('async');
const fs = require("fs");
const os = require('os');
const { exec } = require('child_process');
const path = require('path');

var pstcore = require('node-pstcore');

var m_options = {};
var m_base_path = "userdata/pvf";
var m_pstdef = "take-picture-ptpvf";
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
                if(!m_options){
                    return;
                }

                if(options["take_picture"] && options["take_picture"]["base_path"]){
                    m_base_path = options["take_picture"]["base_path"] + "/";
                }else if(options["http_pvf"] && options["http_pvf"]["base_path"]){
                    m_base_path = options["http_pvf"]["base_path"] + "/";
                }
                
                if(options["pstdef"]){
                    m_pstdef = options["pstdef"];
                }
            },
            command_handler: function (cmd, args, conn) {
                switch(cmd){
                case "take_picture":
                    if(args && args.file_path){
                        m_target_filename = args.file_path;
                    }else{
                        m_target_filename = formatDate(new Date()) + ".pvf";
                    }
                    plugin.take_picture(m_pstdef, m_base_path + m_target_filename, () => {
                        console.log("done");
                    });
                    break;
                default:
                    break;
                }
            },
            take_picture : (pstdef, file_path, callback) => {
                m_plugin_host.build_pstreamer({
                    "base" : pstdef,
                    "replacements" : {
                        "@FILE_PATH@" : file_path
                    }
                }, (res) => {
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
                    pstcore.pstcore_set_dequeue_callback(res.pst, (data) => {
                        if(data == null){//eob
                            if(eob){//eos
                                pstcore.pstcore_destroy_pstreamer(res.pst);
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
                    pstcore.pstcore_start_pstreamer(res.pst);
                });
            },
        };
        return plugin;
    }
};
module.exports = self;