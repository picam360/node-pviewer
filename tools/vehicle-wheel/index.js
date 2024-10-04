
console.log("create wheel plugin");
var { PythonShell } = require('python-shell');

let m_options = {};
var m_cmd_timer = 0;
var m_duty = 50;// %
var pyshell = null;

m_options = {};

function init() {
	pyshell = new PythonShell(__dirname + '/wheel.py');
	pyshell.on('message', function (message) {
		console.log("wheel.py : " + message);
	});
	pyshell.send('init');
}

function send_command(cmd) {
	if(!pyshell){
		return;
	}

	var split = cmd.split(' ');
	cmd = split[0];
	pyshell.send(cmd);

	clearTimeout(m_cmd_timer);
	m_cmd_timer = setTimeout(() => {
		pyshell.send("stop");
	}, m_options.cmd_effective_period || 500);
}

function main() {

    const redis = require('redis');
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

	init();

	const subscriber = client.duplicate();
	subscriber.connect().then(() => {
		console.log('redis subscriber connected:');

		subscriber.subscribe('pserver-vehicle-wheel', (data, key) => {
			var params = data.trim().split(' ');
			switch (params[0]) {
				case "CMD":
					if(m_options.debug){
						console.log(`"${data}" subscribed.`);
					}
					send_command(params[1]);
					break;
			}
		});
	});
}

if (require.main === module) {
    main();
}
