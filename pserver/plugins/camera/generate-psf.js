
const async = require('async');
const fs = require("fs");
const os = require('os');
const { exec } = require('child_process');
const path = require('path');

var pstcore = require('node-pstcore');

var m_options = {};
var m_base_path = "./";
var PLUGIN_NAME = "generate_psf";

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
                m_options = options["generate_psf"];

                m_base_path = 'userdata/pvf';
                if(options["generate_psf"] && options["generate_psf"]["base_path"]){
                    m_base_path = options["generate_psf"]["base_path"] + "/";
                }else if(options["http_pvf"] && options["http_pvf"]["base_path"]){
                    m_base_path = options["http_pvf"]["base_path"] + "/";
                }
            },
            command_handler: function (cmd, conn) {
                switch(cmd){
                case "generate_psf":
                    m_target_filename = formatDate(new Date()) + ".psf";
                    data = {"id":m_target_filename.toString()};
                    plugin.generate_psf(m_base_path + m_target_filename, options.psf_config, () => {
                        console.log("done");
                    });
                    break;
                default:
                    data = {"err":"unknown cmd"};
                    break;
                }
            },
            generate_psf : (filepath, config, callback) => {
                var tmp_dir = filepath + ".tmp";
                fs.mkdirSync(tmp_dir);
                fs.mkdirSync(tmp_dir + "/pvf");
                fs.writeFileSync(tmp_dir + "/config.json", JSON.stringify(config));

                function copy_pvf(idx, cb){
                    if(idx >= config.points.length){
                        cb();
                        return;
                    }
                    var src = m_base_path + path.basename(config.points[idx].path);
                    var dst = tmp_dir + "/" + config.points[idx].path;
                    fs.copyFile(src, dst, (err) => {
                        if (err) {
                            console.error('error', err);
                        }
                        copy_pvf(idx + 1, cb);
                    });
                }
                copy_pvf(0, () => {const { spawn } = require('child_process');
                    const cmd = `(cd ${tmp_dir} && zip -0r - ./*) > ${filepath + ".zip"}`;
                    exec(cmd, (error, stdout, stderr) => {
                        if (error) {
                          console.error(`error: ${error.message}`);
                          return;
                        }

                        console.log(`done ${cmd}`);
                        fs.renameSync(filepath + ".zip", filepath);
                        if(callback){
                            callback();
                        }
                    });
                });
            },
        };
        return plugin;
    }
};
module.exports = self;