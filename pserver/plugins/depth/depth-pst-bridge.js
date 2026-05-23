const redis = require('redis');
var PLUGIN_NAME = "redis";

function launchDepth() {
    const vord_path = "/home/picam360/github/picam360-depth";
    const vord_options = "";
    const command = `
        source /home/picam360/miniconda3/etc/profile.d/conda.sh && \
        conda activate raft_stereo_py310 && \
        python ${vord_path}/raft-stereo.py ${vord_options}
    `;
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

var self = {
    create_plugin: function (plugin_host) {
        const m_plugin_host = plugin_host;
        const m_depth = {};
        console.log("create host plugin");
        var plugin = {
            name: PLUGIN_NAME,
            init_options: function (options) {
                m_options = options["depth-pst-bridge"];

                if(!m_options["enabled"]){
                    return;
                }

                const client = m_plugin_host.get_redis_client();
                const subscriber = client.duplicate();
                subscriber.connect().then(() => {
                    console.log('redis subscriber connected:');

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

                                    m_client.publish('picam360-depth', JSON.stringify({
                                        "cmd": "disparity",
                                        "test": false,
                                        "show": m_options["depth_debug"],
                                        "binning": m_options["depth_binning"],
                                        "jpeg_data": jpeg_data.toString("base64"),
                                        "user_data": { direction },
                                    }));
                                }
                            }

                            tmp_img = [];
                        } else {
                            tmp_img.push(Buffer.from(data, 'base64'));
                        }
                    });
                
                    subscriber.subscribe('picam360-depth-output', (data, key) => {
                        //console.log(data);
                        const params = JSON.parse(data);
            
                        if (params['type'] == 'info') {
                            if (params['msg'] == 'startup') {
                                m_depth.state = 1;
                            }
                        } else if (params['type'] == 'disparity') {
                            const now = Date.now();
                            m_depth.state = 1;
                            m_depth.et = now;
                            const buffer = Buffer.from(params['disparity'], 'base64');
                            m_depth.disparity = cv.imdecode(buffer, cv.IMREAD_UNCHANGED);//16bit png
                            m_depth.min_val = params['min_val'];
                            m_depth.max_val = params['max_val'];
                            const elapsed = m_depth.et - m_depth.st;
                            //console.log(`depth (${direction}) updated in ${elapsed}ms`);
                        }
                    });

                    killDepth();
                    launchDepth();
                });
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