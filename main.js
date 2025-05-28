const { app, BrowserWindow } = require('electron');
const path = require('path');
const fs = require('fs');
const yargs = require('yargs/yargs');
const { hideBin } = require('yargs/helpers');
const argv = yargs(hideBin(process.argv)).argv;
const query = argv.query || ''; // "param1=value1&param2=value2"
const devTools = argv.devTools || false;

const createWindow = () => {
    let appIcon;
    if (fs.existsSync(`${__dirname}/img/app.png`)) {
        appIcon = `${__dirname}/img/app.png`;
    } else if (fs.existsSync(`${__dirname}/img/icon.png`)) {
        appIcon = `${__dirname}/img/icon.png`;
    } else {
        appIcon = `${__dirname}/img/logo.png`;
    }
    console.log(appIcon);

    const win = new BrowserWindow({
        width: 800,
        height: 600,
        icon: appIcon,
        webPreferences: {
            devTools: devTools,
            nodeIntegration: false,
            contextIsolation: false,
            enableRemoteModule: true,
            preload: path.join(__dirname, 'preload.js'),
        },
        hasShadow: false,
        transparent: true,
        backgroundColor: "#01000000",
        fullscreen: false,
        frame: false,
    });

    if(devTools){
        win.webContents.openDevTools();
    }
    win.loadURL(`file://${__dirname}/pviewer/index.html?${query}`);
};

app.commandLine.appendSwitch('enable-transparent-visuals');
app.commandLine.appendSwitch('disable-gpu');

app.whenReady().then(() => {
    setTimeout(() => {
        createWindow();
    }, 3000);//avoid ubuntu transparent issue
});