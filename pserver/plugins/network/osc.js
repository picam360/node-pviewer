
const async = require('async');
const fs = require("fs");
const os = require('os');
const { exec } = require('child_process');
const path = require('path');

var pstcore = require('node-pstcore');

var m_options = {};
var m_base_path = "./";
var PLUGIN_NAME = "ocs";

var m_target_filename = "";


var self = {
    create_plugin: function (plugin_host) {
        m_plugin_host = plugin_host;
        console.log("create host plugin");
        var plugin = {
            name: PLUGIN_NAME,
            init_options: function (options) {
                m_options = options["osc"];

                m_base_path = 'userdata/pvf';
                if(options["osc"] && options["osc"]["base_path"]){
                    m_base_path = options["osc"]["base_path"] + "/";
                }else if(options["http_pvf"] && options["http_pvf"]["base_path"]){
                    m_base_path = options["http_pvf"]["base_path"] + "/";
                }

                function formatDate(date) {
                    const year = date.getFullYear();
                    const month = String(date.getMonth() + 1).padStart(2, '0');
                    const day = String(date.getDate()).padStart(2, '0');
                    const hours = String(date.getHours()).padStart(2, '0');
                    const minutes = String(date.getMinutes()).padStart(2, '0');
                    const seconds = String(date.getSeconds()).padStart(2, '0');
                
                    return `${year}-${month}-${day}_${hours}-${minutes}-${seconds}`;
                }

                var express_app = m_plugin_host.get_express_app();
                
                express_app.all(/^\/osc\/.*$/, function(req, res) {
                    var url = req.url.split("?")[0];
                    var query = req.url.split("?")[1];
                    var filename = url.substr(5);
                    console.log(url);
                    console.log(query);
                    console.log(filename);
                    var mime = "application/json";
                    var data = {};
                    switch(filename){
                    case "info":
                        data = {"type":"info"};
                        break;
                    case "commands/execute":
                        if(req.method == 'POST'){
                            var options = req.body;
                            switch(options.name){
                            case "camera.takePicture":
                                m_target_filename = formatDate(new Date()) + ".pvf";
                                data = {"id":m_target_filename.toString()};
                                plugin_host.send_command(
                                    "take_picture.take_picture",
                                    {
                                        "file_path" : m_base_path + m_target_filename
                                    },
                                    (res) => {
                                        console.log("done");
                                    }
                                );
                                break;
                            case "pserver.generatePsf":
                                m_target_filename = formatDate(new Date()) + ".psf";
                                data = {"id":m_target_filename.toString()};
                                plugin_host.send_command(
                                    "generate_psf.generate_psf",
                                    {
                                        "FILE_PATH" : m_base_path + m_target_filename,
                                        "psf_config" : options.psf_config,
                                    },
                                    (res) => {
                                        console.log("done");
                                    }
                                );
                                break;
                            default:
                                data = {"err":"unknown cmd"};
                                break;
                            }
                        }else{
                            data = {"err":"need POST request"};
                        }
                        break;
                    case "commands/status":
                        if(req.method == 'POST'){
                            var options = req.body;
                            if(options.id == m_target_filename){
                                if (fs.existsSync(m_base_path + m_target_filename)) {
                                    data = {
                                        "state" : "done",
                                        "results" : {
                                            "fileUrl" : "pvf/" + m_target_filename
                                        }
                                    };
                                }else{
                                    data = {
                                        "state" : "processing"
                                    };
                                }
                            }
                        }else{
                            data = {"err":"need POST request"};
                        }
                        break;
                    default:
                        res.writeHead(403);
                        res.end();
                        console.log("403");
                        return;
                        break;
                    }
                    var content = JSON.stringify(data);
                    res.writeHead(200, {
                        'Content-Type': mime,
                        'Content-Length': content.length,
                        'Cache-Control': 'private, no-cache, no-store, must-revalidate',
                        'Expires': '-1',
                        'Pragma': 'no-cache',
                    });
                    res.end(content);
                    console.log("200");
                });
            },
            command_handler: function (cmd, conn) {
            },
        };
        return plugin;
    }
};
module.exports = self;