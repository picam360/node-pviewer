
const async = require('async');
const fs = require("fs");
const sprintf = require('sprintf-js').sprintf;
const uuidgen = require('uuid/v4');
const url = require("url");
const EventEmitter = require('eventemitter3');
const WebSocket = require("ws");

const PLUGIN_NAME = "ws_bridge";


const bind_ws = (ws1, ws2, options) => {
    options = options || {};

    ws1.name = "src";
    ws2.name = "dst";

    {// ws1 -> ws2
        const src = ws1;
        const dst = ws2;
        src.onopen = (event) => {
            console.log(src.name + " : opened.");
            if(src.pendings){
                for(const data of src.pendings){
                    console.log(src.name + " : send pending", data.length, src.readyState);
                    dst.send(data);
                }
                src.pendings = undefined;
            }
        };
        src.onmessage = (event) => {
            const data = event.data;
            if(options.debug){
                console.log(src.name + " to " + dst.name, data.length);
            }
            if (dst.readyState == 0) {
                dst.pendings = dst.pendings || [];
                dst.pendings.push(data);
                return;
            }
            dst.send(data);
        };
        src.onclose = (event) => {
            console.log("ws closed.");
            dst.close();
        };
    }

    {// ws2 -> ws1
        const src = ws2;
        const dst = ws1;
        src.onopen = (event) => {
            console.log(src.name + " : opened.");
            if(src.pendings){
                for(const data of src.pendings){
                    console.log(src.name + " : send pending", data.length, src.readyState);
                    dst.send(data);
                }
                src.pendings = undefined;
            }
        };
        src.onmessage = (event) => {
            const data = event.data;
            if(options.debug){
                console.log(src.name + " to " + dst.name, data.length);
            }
            if (dst.readyState == 0) {
                dst.pendings = dst.pendings || [];
                dst.pendings.push(data);
                return;
            }
            dst.send(data);
        };
        src.onclose = (event) => {
            console.log("ws closed.");
            dst.close();
        };
    }

};

function start_websocket(callback) {
    // websocket
    for(var server of [m_plugin_host.get_ws_server(), m_plugin_host.get_wss_server()]){
        if(!server){
            continue;
        }
        server.on("connection", (ws1, request) => {
            const parsed = url.parse(request.url, true);
            if(parsed.pathname != "/ws_bridge"){
                return;
            }
            if(!parsed.query.port){
                console.log("ws_bridge require port number");
                return;
            }
            const protocol = (parsed.query.protocol ? parsed.query.protocol : "ws");
            const host = (parsed.query.host ? parsed.query.host : "localhost");
            const ws2_url = `${protocol}://${host}:${parsed.query.port}`;
            const ws2 = new WebSocket(ws2_url);
            bind_ws(ws1, ws2);
        });
    }
    callback(null);
}

function filerequest_handler(filename, key, conn) {
    fs.readFile("../pviewer/" + filename, function(err, data) {
        if (err) {
            var header_str = "<picam360:file name=\"" + filename +
                "\" key=\"" + key + "\" status=\"404\" />";
            data = Buffer.alloc(0);
            console.log("unknown :" + filename + ":" + key);
            var header = Buffer.from(header_str, 'ascii');
            var len = 2 + header.length;
            var buffer = Buffer.alloc(len);
            buffer.writeUInt16BE(header.length, 0);
            header.copy(buffer, 2);
//					var pack = rtp.build_packet(buffer, PT_FILE);
//					rtp.send_packet(pack);
        } else {
            send_file(filename, key, conn, data);
        }
    });
}

var self = {
    create_plugin: function (plugin_host) {
        m_plugin_host = plugin_host;
        console.log("create host plugin");
        var plugin = {
            name: PLUGIN_NAME,
            init_options: function (_options) {
                options = _options;
                start_websocket(() => {
                    console.log("host initiation done!");
                });
            },
            pst_started: function (pstcore, pst) {
            },
            pst_stopped: function (pstcore, pst) {
            },
            command_handler: function (cmd, conn) {
            },
        };
        return plugin;
    }
};
module.exports = self;