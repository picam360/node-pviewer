
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
                    if(args && args.file_path){
                        m_target_filename = args.file_path;
                    }else{
                        m_target_filename = formatDate(new Date()) + ".pvf";
                    }
                    plugin.start_stream("start_stream.start_stream", {
                        "stream" : "take-picture-ptpvf",
                        "replacements" : {
                            "@FILE_PATH@" : m_base_path + m_target_filename
                        }
                    }, () => {
                        console.log("done");
                    });
                    break;
                default:
                    break;
                }
            },
        };
        return plugin;
    }
};
module.exports = self;