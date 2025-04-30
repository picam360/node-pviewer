
console.log("create adb-screenshot plugin");

const yargs = require('yargs');
const { spawn } = require('child_process');
const redis = require('redis');
const sharp = require('sharp');

let m_options = {};

m_options = {};

function screencapture(device, callback) {
    const args = [];

    if (device) {
        args.push('-s', device);
    }

    args.push('exec-out', 'screencap', '-p');

    const adb = spawn('adb', args);

    let chunks = [];

    adb.stdout.on('data', (chunk) => {
        chunks.push(chunk);
    });

    adb.stderr.on('data', (data) => {
        console.error(`stderr: ${data}`);
    });

    adb.on('close', (code) => {
        if (code !== 0) {
            console.error(`adb process exited with code ${code}`);
            return;
        }

        const pngBuffer = Buffer.concat(chunks);

		callback(pngBuffer);
    });
}

function main() {

    const argv = yargs.option('s', {
        alias: 'target',
        array: true,
        default: [null],
        describe: 'Target devices (e.g., IP:port or serial)',
        type: 'string'
    }).argv;

    const client = redis.createClient({
        host: 'localhost',
        port: 6379,
    });
    client.on('error', (err) => {
        console.error('redis error:', err);
    });
    client.connect().then(() => {
        console.log('redis connected:');
	});

    const width = 512;
    const quality = 80;
	setInterval(() => {
        for (let i=0;i<argv.s.length;i++) {
            const device = argv.s[i];
            screencapture(device, async (data) => {
                try {
                    const resizedBuffer = await sharp(data)
                        .resize({ width })
                        .png({ quality })
                        .toBuffer();

                    const dataUrl = `data:image/png;base64,${resizedBuffer.toString('base64')}`;
        
                    const channel = 'adb-screenshot-' + i;
                    const count = await client.publish(channel, `{"enabled":true,"url":"${dataUrl}"}`);
                    console.log(`${channel} published device=${device}, bytes=${resizedBuffer.length}B, count=${count}`);
                } catch (err) {
                    console.error('Sharp processing error:', err);
                }
            });
        }
	}, 1000);
}

if (require.main === module) {
    main();
}
