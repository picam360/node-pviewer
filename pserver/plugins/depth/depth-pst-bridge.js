
const fs = require("fs");
const path = require("path");
const { execSync } = require('child_process');
const { spawn } = require('child_process');
const pif_utils = require('./pif-utils');

const PLUGIN_NAME = "depth-pst-bridge";
const m_depth = {};

function launchDepth() {
    const vord_path = "/home/picam360/github/picam360-depth";
    const vord_options = "";
    const command = `
        source /home/picam360/miniconda3/etc/profile.d/conda.sh && \
        conda activate dpvo_py310 && \
        python ${vord_path}/foundation-stereo.py ${vord_options}
    `;
    // const command = `
    //     source /home/picam360/miniconda3/etc/profile.d/conda.sh && \
    //     conda activate raft_stereo_py310 && \
    //     python ${vord_path}/raft-stereo.py ${vord_options}
    // `;
    // const command = `
    // 	source /home/picam360/miniconda3/etc/profile.d/conda.sh && \
    // 	conda activate raft_stereo_py310 && \
    // 	python ${vord_path}/stereo-sgbm.py ${vord_options}
    // `;
    const vslam_process = spawn(command, { shell: '/bin/bash', cwd: path.resolve(vord_path) });
    vslam_process.stdout.on('data', (data) => {
        console.log(`PICAM360_DEPTH STDOUT: ${data}`);
    });

    vslam_process.stderr.on('data', (data) => {
        console.error(`PICAM360_DEPTH STDERR: ${data}`);
    });

    vslam_process.on('close', (code) => {
        console.log(`PICAM360_DEPTH STDOUT CLOSED(: ${code})`);
    });
}

function killDepth() {
    const processName = "picam360-depth";

    try {
        const output = execSync(`ps -eo pid,comm,args`, { encoding: "utf-8" });

        const lines = output.trim().split("\n").slice(1);

        const matchingPids = lines
            .map(line => line.trim().split(/\s+/, 3)) // [pid, comm, args]
            .filter(([pid, comm, args]) =>
                comm === processName || args.split(" ")[0] === processName
            )
            .map(([pid]) => parseInt(pid));

        // kill
        for (const pid of matchingPids) {
            console.log(`Killing PID ${pid} (${processName})`);
            process.kill(pid);
        }

        if (matchingPids.length === 0) {
            console.log(`No matching process found for "${processName}"`);
        }

    } catch (err) {
        console.error("Error while killing process:", err.message);
    }
}
function set_subscriber(client) {
    const subscriber = client.duplicate();
    subscriber.connect().then(() => {
        console.log('redis subscriber connected:');

        const img_queue = [];

        let tmp_img = [];
        subscriber.subscribe(m_options.pst_in_channel, (data, key) => {
            const now = Date.now();
            m_options.last_pst_ts = now;
            if (data.length == 0 && tmp_img.length != 0) {
                if (tmp_img.length == 3) {
                    const jpeg_data = tmp_img[2];

                    if (m_depth.state == 1 && now - m_depth.st > 500) {//2fps
                        m_depth.state = 2;
                        m_depth.st = now;

                        client.publish('picam360-depth', JSON.stringify({
                            "cmd": "disparity",
                            "test": false,
                            "show": m_options["depth_debug"] || false,
                            "binning": m_options["depth_binning"] || 2,
                            "jpeg_data": jpeg_data.toString("base64"),
                            "user_data": { },
                        }));
                    }
                    img_queue.push(tmp_img);
                }

                tmp_img = [];
            } else {
                tmp_img.push(Buffer.from(data, 'base64'));
            }
        });
    
        subscriber.subscribe('picam360-depth-output', (data, key) => {
            //console.log(data);
            const params = JSON.parse(data);
            const now = Date.now();

            if (params['type'] == 'info') {
                if (params['msg'] == 'startup') {
                    m_depth.state = 1;
                    m_depth.st = now;
                }
            } else if (params['type'] == 'disparity') {
                m_depth.state = 1;
                m_depth.et = now;
                const buffer = Buffer.from(params['disparity'], 'base64');
                //m_depth.disparity = cv.imdecode(buffer, cv.IMREAD_UNCHANGED);//16bit png
                m_depth.min_val = params['min_val'];
                m_depth.max_val = params['max_val'];
                const elapsed = m_depth.et - m_depth.st;
                console.log(`depth updated in ${elapsed}ms`);

                const src_img = img_queue.pop();
                if(!src_img){
                    return;
                }
                if(!m_options.pst_out_channel){
                    return;
                }
                pif_utils.read_pif_from_buffer(src_img, (img_dom, [header, meta, pixels]) => {
                    img_dom["picam360:image"].img_type = 'PNG';
                    img_dom["picam360:image"].width = `${buffer.length},0,0`;
                    img_dom["picam360:image"].height = `${1},0,0`;
                    img_dom["picam360:image"].stride = `${buffer.length},0,0`;
                    const xml = img_dom.build();
                    const buff1 = Buffer.alloc(4 + xml.length);
                    buff1.write('PI', 'utf-8');
                    buff1.writeUInt16BE(xml.length, 2);
                    buff1.write(xml, 4, 'utf-8');
                    const buff2 = meta;
                    const buff3 = buffer;
                    for(const buff of [buff1, buff2, buff3]){
                        client.publish(m_options.pst_out_channel, buff);
                    }
                });
            }
        });

        killDepth();
        launchDepth();
    });
}

var self = {
    create_plugin: function (plugin_host) {
        const m_plugin_host = plugin_host;
        console.log("create host plugin");
        var plugin = {
            name: PLUGIN_NAME,
            init_options: function (options) {
                m_options = options["depth-pst-bridge"];

                if(m_options["enabled"]){
                    setTimeout(() => {
                        const client = m_plugin_host.get_redis_client();
                        if(client){
                            set_subscriber(client);
                        }
                    }, 1000);
                }
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